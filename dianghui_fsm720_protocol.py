"""
巅慧快反串口通信协议封装模块

串口参数: 115200 baud, 8N1（默认，可配置）
最小通信间隔: 1ms（发送一帧指令后立即收到状态帧，交互对称）

指令帧格式 (13 bytes):
  byte  0-1 : 帧头 0x7E 0xE7
  byte  2   : 指令码
               0x00 = 待命
               0x01 = 闭环
               0x02 = 自检
               0x03 = 开环
               0x04 = 快反镜协议X轴正弦扫描
               0x05 = 快反镜协议Y轴正弦扫描
               0x06 = 快反镜XY轴同时正弦扫描
  byte  3-4 : 闭环/开环/待命/自检: 协议 X 轴设定值 (int16, BE)  [协议X ↔ 物理Y]
              正弦扫描 (0x04/05/06): 正弦频率 (int16, BE)，1 Hz/LSB，范围 0~5000
  byte  5-6 : 闭环/开环/待命/自检: 协议 Y 轴设定值 (int16, BE)  [协议Y ↔ 物理X]
              正弦扫描 (0x04/05/06): 正弦幅值 (int16, BE)，峰值（非峰峰值）
  byte  7-8 : 协议 X 轴限位值 (uint16, big-endian)
  byte  9-10: 协议 Y 轴限位值 (uint16, big-endian)
  byte 11   : 保留 0x00
  byte 12   : 校验位 = (~sum(byte[2:12])) & 0xFF

状态帧格式 (10 bytes):
  byte  0-1 : 帧头 0x7E 0xE7
  byte  2   : 状态字节（各位含义，bit7 = MSB）:
               bit7: 通讯校验状态  (0=正常, 1=异常)
               bit6: 协议Y轴位移超限 → 物理X轴  (0=未超限, 1=超限)
               bit5: 协议X轴位移超限 → 物理Y轴
               bit4: 协议Y轴指令超限 → 物理X轴
               bit3: 协议X轴指令超限 → 物理Y轴
               bit2: 自检状态        (0=正常, 1=异常)
               bit1: 协议Y轴使能状态 → 物理X轴  (0=使能, 1=未使能)
               bit0: 协议X轴使能状态 → 物理Y轴
  byte  3-4 : 协议 X 轴位移计读 (int16, big-endian) [→ 物理Y]
  byte  5-6 : 协议 Y 轴位移计读 (int16, big-endian) [→ 物理X]
  byte  7-8 : 保留 0x0000
  byte  9   : 校验位 = (~sum(byte[2:9])) & 0xFF

校验算法: checksum = (~sum(待校验字节列表)) & 0xFF
  "除帧头以外的字节，按字节求和取反取低8位"
  指令帧: 待校验 = byte[2:12]（10字节）
  状态帧: 待校验 = byte[2:9] （7字节）

轴映射说明（透明，所有公开接口使用物理轴坐标）:
  物理X轴 (channel 0) ↔ 协议Y字段 / 协议正弦指令 0x05
  物理Y轴 (channel 1) ↔ 协议X字段 / 协议正弦指令 0x04
  原因: 设备实际接线与期望运动轴相反，协议层统一处理交换以对外透明。
"""

from __future__ import annotations

import struct
import threading
from dataclasses import dataclass
from time import perf_counter, sleep
from typing import Optional

import serial


# ---------------------------------------------------------------------------
# 帧常量
# ---------------------------------------------------------------------------

FRAME_HEADER   = bytes([0x7E, 0xE7])

CMD_STANDBY     = 0x00  # 待命：自由态，X/Y 轴均不使能
CMD_CLOSED_LOOP = 0x01  # 闭环：X/Y 轴使能，闭环位置控制（发送位置 / 读取状态）
CMD_SELFCHECK   = 0x02  # 自检
CMD_OPEN_LOOP   = 0x03  # 开环：轴不使能
CMD_SINE_X      = 0x04  # 协议X轴正弦扫描 → 物理Y
CMD_SINE_Y      = 0x05  # 协议Y轴正弦扫描 → 物理X
CMD_SINE_XY     = 0x06  # XY轴同时正弦扫描

# 公开接口使用物理轴：'x' / 'y' / 'xy'
SINE_CMD_PHYS = {
    "x":  CMD_SINE_Y,   # 物理X ↔ 协议Y
    "y":  CMD_SINE_X,   # 物理Y ↔ 协议X
    "xy": CMD_SINE_XY,
}

TX_LEN = 13   # 指令帧总长度（bytes）
RX_LEN = 10   # 状态帧总长度（bytes）

DEFAULT_LIMIT  = 5000   # 初始化时默认限位值
SETPOINT_LIMIT = 25000  # 程序软限位：X/Y 轴设定值不超过 ±25000
SINE_FREQ_MAX  = 5000   # 正弦频率上限（Hz），1 Hz/LSB

# ---------------------------------------------------------------------------
# 发送间隔配置
# ---------------------------------------------------------------------------
# 模拟位置序列：500 Hz（2ms）。控制器单次闭环约 0.2ms，500Hz 留有余量。
ANALOG_INTERVAL_MS: int = 2
# 模拟发送每个点连发帧数（1 = 单帧，>1 = 连发）。
ANALOG_BURST_N: int = 5
# 硬件正弦保活：1 kHz（1ms），与模拟发送独立。
WAVE_INTERVAL_MS: int = 1


# ---------------------------------------------------------------------------
# 状态数据类（物理轴坐标，已做轴交换）
# ---------------------------------------------------------------------------

@dataclass
class DianghuiStatus:
    """
    巅慧状态帧解析结果。

    所有字段已换算为物理轴坐标（物理X = channel 0 = 用户期望的X轴），
    与协议中的 X/Y 字段定义相反（协议X → 物理Y，协议Y → 物理X）。
    """
    comm_error:      bool  # 通讯校验状态 (True = 异常)
    x_disp_over:    bool  # 物理X轴位移超限
    y_disp_over:    bool  # 物理Y轴位移超限
    x_cmd_over:     bool  # 物理X轴指令超限
    y_cmd_over:     bool  # 物理Y轴指令超限
    selfcheck_error: bool  # 自检状态 (True = 异常)
    x_enabled:      bool  # 物理X轴已使能 (True = 使能)
    y_enabled:      bool  # 物理Y轴已使能 (True = 使能)
    x_feedback:     int   # 物理X轴位移计读 (int16 原始码值)
    y_feedback:     int   # 物理Y轴位移计读 (int16 原始码值)


# ---------------------------------------------------------------------------
# 自定义异常
# ---------------------------------------------------------------------------

class DianghuiError(Exception):
    """巅慧快反通信异常。"""


# ---------------------------------------------------------------------------
# 底层帧工具（内部使用，参数均为协议坐标）
# ---------------------------------------------------------------------------

def _spin_until(deadline: float) -> None:
    """忙等到 deadline。Windows 的 sleep 分辨率约 1ms，无法用于 0.2ms 节拍。"""
    while perf_counter() < deadline:
        pass


def _calc_checksum(data: bytes | list[int]) -> int:
    """计算校验位: (~sum(data)) & 0xFF"""
    return (~sum(data)) & 0xFF


def _build_tx_frame(
    cmd: int,
    proto_x_set: int,
    proto_y_set: int,
    proto_x_lim: int,
    proto_y_lim: int,
) -> bytes:
    """
    构造指令帧（协议坐标，调用方负责做轴交换）。

    :param cmd:         指令码
    :param proto_x_set: 协议X轴设定值 (int16)，即物理Y
    :param proto_y_set: 协议Y轴设定值 (int16)，即物理X
    :param proto_x_lim: 协议X轴限位值 (uint16)
    :param proto_y_lim: 协议Y轴限位值 (uint16)
    """
    proto_x_set = max(-SETPOINT_LIMIT, min(SETPOINT_LIMIT, int(proto_x_set)))
    proto_y_set = max(-SETPOINT_LIMIT, min(SETPOINT_LIMIT, int(proto_y_set)))
    proto_x_lim = max(0, min(65535, int(proto_x_lim)))
    proto_y_lim = max(0, min(65535, int(proto_y_lim)))

    # body: cmd(B) + x_set(h) + y_set(h) + x_lim(H) + y_lim(H) + reserved(B)
    body = struct.pack('>BhhHHB', cmd, proto_x_set, proto_y_set,
                       proto_x_lim, proto_y_lim, 0x00)
    checksum = _calc_checksum(body)
    return FRAME_HEADER + body + bytes([checksum])


def _build_sine_tx_frame(
    cmd: int,
    frequency: int,
    amplitude: int,
    proto_x_lim: int,
    proto_y_lim: int,
) -> bytes:
    """
    构造正弦控制帧（频率/幅值不走轴交换；限位仍为协议坐标）。

    :param cmd:       0x04 / 0x05 / 0x06
    :param frequency: 正弦频率（Hz），1 Hz/LSB，范围 0~5000
    :param amplitude: 正弦峰值（与位置设定值同单位，非峰峰值）
    """
    frequency = max(0, min(SINE_FREQ_MAX, int(frequency)))
    amplitude = max(-SETPOINT_LIMIT, min(SETPOINT_LIMIT, int(amplitude)))
    proto_x_lim = max(0, min(65535, int(proto_x_lim)))
    proto_y_lim = max(0, min(65535, int(proto_y_lim)))

    body = struct.pack('>BhhHHB', cmd, frequency, amplitude,
                       proto_x_lim, proto_y_lim, 0x00)
    checksum = _calc_checksum(body)
    return FRAME_HEADER + body + bytes([checksum])


def _parse_rx_frame(data: bytes) -> Optional[DianghuiStatus]:
    """
    解析状态帧，返回 DianghuiStatus（物理轴坐标），校验失败返回 None。
    """
    if len(data) != RX_LEN:
        return None
    if data[0:2] != FRAME_HEADER:
        return None
    # 校验: byte[2:9] 求和取反低8位 = byte[9]
    if _calc_checksum(data[2:9]) != data[9]:
        return None

    sb = data[2]  # status byte
    # 状态位（MSB优先）：协议Y → 物理X；协议X → 物理Y
    comm_error       = bool(sb & 0x80)
    proto_y_disp_ov  = bool(sb & 0x40)  # 协议Y位移超限 → 物理X
    proto_x_disp_ov  = bool(sb & 0x20)  # 协议X位移超限 → 物理Y
    proto_y_cmd_ov   = bool(sb & 0x10)  # 协议Y指令超限 → 物理X
    proto_x_cmd_ov   = bool(sb & 0x08)  # 协议X指令超限 → 物理Y
    selfcheck_error  = bool(sb & 0x04)
    proto_y_disabled = bool(sb & 0x02)  # 协议Y未使能 → 物理X未使能
    proto_x_disabled = bool(sb & 0x01)  # 协议X未使能 → 物理Y未使能

    # 位移计读 int16 big-endian: 协议X[3:5] → 物理Y, 协议Y[5:7] → 物理X
    proto_x_fb, proto_y_fb = struct.unpack('>hh', data[3:7])

    return DianghuiStatus(
        comm_error      = comm_error,
        x_disp_over     = proto_y_disp_ov,    # 协议Y → 物理X
        y_disp_over     = proto_x_disp_ov,    # 协议X → 物理Y
        x_cmd_over      = proto_y_cmd_ov,     # 协议Y → 物理X
        y_cmd_over      = proto_x_cmd_ov,     # 协议X → 物理Y
        selfcheck_error  = selfcheck_error,
        x_enabled       = not proto_y_disabled,  # bit=0 表示使能 → 物理X
        y_enabled       = not proto_x_disabled,  # bit=0 表示使能 → 物理Y
        x_feedback      = proto_y_fb,         # 协议Y → 物理X
        y_feedback      = proto_x_fb,         # 协议X → 物理Y
    )


# ---------------------------------------------------------------------------
# 控制器
# ---------------------------------------------------------------------------

class DianghuiController:
    """
    巅慧快反串口控制器。

    所有公开接口使用物理轴坐标（物理X = channel 0，物理Y = channel 1），
    内部透明处理协议轴交换（物理X → 协议Y，物理Y → 协议X）。
    线程安全（内置 Lock，可在主线程和波形线程中并发调用）。

    使用示例::

        with DianghuiController(port='COM3') as dh:
            dh.send_selfcheck()
            time.sleep(1.0)
            dh.send_control(x_phys=0, y_phys=0, x_lim=5000, y_lim=5000)
    """

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        read_timeout: float = 0.05,
    ):
        self._port = port
        self._baudrate = baudrate
        self._timeout = read_timeout
        self._serial: Optional[serial.Serial] = None
        self._lock = threading.Lock()

        # 当前物理轴状态（发送帧时合并双轴）
        self._x_phys: int = 0
        self._y_phys: int = 0
        self._x_lim: int = DEFAULT_LIMIT
        self._y_lim: int = DEFAULT_LIMIT

    # ------------------------------------------------------------------
    # 串口生命周期
    # ------------------------------------------------------------------

    def open(self) -> None:
        """打开串口。"""
        self._serial = serial.Serial(
            port      = self._port,
            baudrate  = self._baudrate,
            bytesize  = serial.EIGHTBITS,
            parity    = serial.PARITY_NONE,
            stopbits  = serial.STOPBITS_ONE,
            timeout   = self._timeout,
        )

    def close(self) -> None:
        """关闭串口。"""
        if self._serial and self._serial.is_open:
            self._serial.close()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *_):
        self.close()

    # ------------------------------------------------------------------
    # 内部发送/接收（调用方须持有锁）
    # ------------------------------------------------------------------

    def _send_recv_unlocked(self, frame: bytes) -> Optional[DianghuiStatus]:
        assert self._serial and self._serial.is_open, "串口未打开"
        # 清空接收缓冲区中因只写模式积压的旧响应帧，
        # 避免 read() 读到错位的旧数据导致校验失败进而触发虚假断连。
        try:
            self._serial.reset_input_buffer()
        except Exception:
            pass  # 极少数 USB CDC 驱动不支持，忽略
        self._serial.write(frame)
        data = self._serial.read(RX_LEN)
        return _parse_rx_frame(data)

    def _build_closed_loop_frame(self) -> bytes:
        """
        用当前物理轴状态构造闭环控制帧（轴交换透明）。调用方须持有锁。
        指令码固定 CMD_CLOSED_LOOP (0x01)——唯一能使能轴并设置位置的指令。
        """
        return _build_tx_frame(
            cmd         = CMD_CLOSED_LOOP,
            proto_x_set = self._y_phys,   # 物理Y → 协议X
            proto_y_set = self._x_phys,   # 物理X → 协议Y
            proto_x_lim = self._y_lim,
            proto_y_lim = self._x_lim,
        )

    def _build_sine_frame(self, cmd: int, frequency: int, amplitude: int) -> bytes:
        """构造正弦控制帧。调用方须持有锁。限位做轴交换。"""
        return _build_sine_tx_frame(
            cmd         = cmd,
            frequency   = frequency,
            amplitude   = amplitude,
            proto_x_lim = self._y_lim,
            proto_y_lim = self._x_lim,
        )

    # ------------------------------------------------------------------
    # 公开控制接口（物理轴坐标）
    # ------------------------------------------------------------------

    def write_control_only(self, x_phys: int, y_phys: int) -> None:
        """
        仅写入控制帧，不等待设备响应（高速只写模式）。

        去掉读等待后，单次调用耗时从 ~2ms 降至 ~0.1ms，
        可实现真正的 1ms 发送间隔。
        设备仍会返回状态帧，由 drain_read_buffer() 周期性批量读取。

        :param x_phys: 物理X轴设定值 (int16)
        :param y_phys: 物理Y轴设定值 (int16)
        """
        with self._lock:
            self._x_phys = max(-SETPOINT_LIMIT, min(SETPOINT_LIMIT, int(x_phys)))
            self._y_phys = max(-SETPOINT_LIMIT, min(SETPOINT_LIMIT, int(y_phys)))
            assert self._serial and self._serial.is_open, "串口未打开"
            self._serial.write(self._build_closed_loop_frame())

    def write_control_paced(
        self,
        pairs: list[tuple[int, int]],
        slot_s: float,
        t0: float,
    ) -> None:
        """
        在 1ms 窗口内按均匀节拍逐帧发送（默认 5 帧 → 每 0.2ms 一帧）。

        第 i 帧目标时刻为 ``t0 + i * slot_s``。若已迟到则立即发送，不额外等待。
        使用忙等而不是 sleep：Windows 无法稳定 sleep 200µs。

        注意：USB-CDC 适配器仍可能把若干 URB 挤进同一个 1ms USB 帧；
        本方法保证的是主机侧提交 write 的时刻均匀。
        """
        if not pairs:
            return
        now = perf_counter()
        # 若本毫秒起点已经错过一整格，重定 t0，避免 5 帧挤成一坨发出去
        if now > t0 + slot_s:
            t0 = now
        for i, (x_phys, y_phys) in enumerate(pairs):
            target = t0 + i * slot_s
            if perf_counter() < target:
                _spin_until(target)
            self.write_control_only(x_phys, y_phys)

    def write_control_burst(self, pairs: list[tuple[int, int]]) -> None:
        """
        一次串口写入连续多帧闭环指令（模拟连发）。

        组帧在内存中完成，只调用一次 serial.write，UART 上背靠背发出。
        """
        if not pairs:
            return
        blob = bytearray()
        with self._lock:
            assert self._serial and self._serial.is_open, "串口未打开"
            for x_phys, y_phys in pairs:
                self._x_phys = max(-SETPOINT_LIMIT, min(SETPOINT_LIMIT, int(x_phys)))
                self._y_phys = max(-SETPOINT_LIMIT, min(SETPOINT_LIMIT, int(y_phys)))
                blob += self._build_closed_loop_frame()
            self._serial.write(blob)

    def drain_read_buffer(self) -> Optional[DianghuiStatus]:
        """
        读取并清空串口接收缓冲区，返回最新一帧有效状态（非阻塞）。

        在只写模式下周期性调用：
        - 防止 OS 接收缓冲区溢出
        - 获取最新位置反馈用于 GUI 显示
        返回 None 表示缓冲区中无完整有效帧。
        """
        if not (self._serial and self._serial.is_open):
            return None
        with self._lock:
            n = self._serial.in_waiting
            if n < RX_LEN:
                return None
            raw = self._serial.read(n)

        # 从所有字节中扫描最后一个校验通过的状态帧
        last_status: Optional[DianghuiStatus] = None
        for start in range(len(raw) - RX_LEN + 1):
            if raw[start] == 0x7E and raw[start + 1] == 0xE7:
                status = _parse_rx_frame(bytes(raw[start:start + RX_LEN]))
                if status is not None:
                    last_status = status
        return last_status

    def send_control(
        self,
        x_phys: int,
        y_phys: int,
        x_lim: Optional[int] = None,
        y_lim: Optional[int] = None,
    ) -> Optional[DianghuiStatus]:
        """
        发送控制指令（物理坐标，内部自动做轴交换）。

        :param x_phys: 物理X轴设定值 (int16, -32768 ~ 32767)
        :param y_phys: 物理Y轴设定值 (int16, -32768 ~ 32767)
        :param x_lim:  物理X轴限位值 (uint16)；None = 保持上次值
        :param y_lim:  物理Y轴限位值 (uint16)；None = 保持上次值
        :return: 解析后的状态帧；校验失败返回 None
        """
        with self._lock:
            self._x_phys = int(x_phys)
            self._y_phys = int(y_phys)
            if x_lim is not None:
                self._x_lim = int(x_lim)
            if y_lim is not None:
                self._y_lim = int(y_lim)
            return self._send_recv_unlocked(self._build_closed_loop_frame())

    def send_readback(self) -> Optional[DianghuiStatus]:
        """
        以闭环指令读取当前状态（维持闭环、不改变设定值）。
        空闲时用 0x01 取状态；硬件正弦运行期间由保活线程 drain，勿在此时调用本方法。
        """
        with self._lock:
            return self._send_recv_unlocked(self._build_closed_loop_frame())

    def send_selfcheck(self) -> Optional[DianghuiStatus]:
        """
        发送自检指令。自检一般在 1 秒内完成，期间持续轮询状态即可。
        """
        with self._lock:
            frame = _build_tx_frame(
                cmd         = CMD_SELFCHECK,
                proto_x_set = 0,
                proto_y_set = 0,
                proto_x_lim = DEFAULT_LIMIT,
                proto_y_lim = DEFAULT_LIMIT,
            )
            return self._send_recv_unlocked(frame)

    def send_standby(self) -> Optional[DianghuiStatus]:
        """
        切换为待机模式（0x00）：轴进入自由态，不再使能。
        正常使用时不调用此方法；send_control 始终使用闭环指令。
        """
        with self._lock:
            frame = _build_tx_frame(
                cmd         = CMD_STANDBY,
                proto_x_set = 0,
                proto_y_set = 0,
                proto_x_lim = DEFAULT_LIMIT,
                proto_y_lim = DEFAULT_LIMIT,
            )
            return self._send_recv_unlocked(frame)

    def send_open_loop(self) -> Optional[DianghuiStatus]:
        """切换为开环模式（0x03）：轴不使能。"""
        with self._lock:
            frame = _build_tx_frame(
                cmd         = CMD_OPEN_LOOP,
                proto_x_set = 0,
                proto_y_set = 0,
                proto_x_lim = DEFAULT_LIMIT,
                proto_y_lim = DEFAULT_LIMIT,
            )
            return self._send_recv_unlocked(frame)

    def write_sine_only(
        self,
        mode: str,
        frequency: int,
        amplitude: int,
    ) -> None:
        """
        仅写入正弦控制帧，不等待设备响应（高速只写）。

        :param mode:      物理轴 'x' / 'y' / 'xy'
        :param frequency: 频率 Hz（1 Hz/LSB，0~5000）
        :param amplitude: 峰值（与位置设定值同单位；峰峰值需由调用方 /2）
        """
        cmd = SINE_CMD_PHYS.get(mode)
        if cmd is None:
            raise ValueError(f"正弦模式须为 'x'/'y'/'xy'，收到: {mode}")
        with self._lock:
            assert self._serial and self._serial.is_open, "串口未打开"
            self._serial.write(self._build_sine_frame(cmd, frequency, amplitude))

    def send_sine(
        self,
        mode: str,
        frequency: int,
        amplitude: int,
    ) -> Optional[DianghuiStatus]:
        """
        发送正弦扫描指令并等待状态帧。

        正弦指令不支持偏置；幅值为峰值。停止正弦请改发闭环位置指令。
        """
        cmd = SINE_CMD_PHYS.get(mode)
        if cmd is None:
            raise ValueError(f"正弦模式须为 'x'/'y'/'xy'，收到: {mode}")
        with self._lock:
            return self._send_recv_unlocked(
                self._build_sine_frame(cmd, frequency, amplitude)
            )

    # ------------------------------------------------------------------
    # 状态查询属性
    # ------------------------------------------------------------------

    @property
    def x_setpoint(self) -> int:
        """当前物理X轴设定值（上次 send_control 传入的值）。"""
        return self._x_phys

    @property
    def y_setpoint(self) -> int:
        """当前物理Y轴设定值（上次 send_control 传入的值）。"""
        return self._y_phys


# ---------------------------------------------------------------------------
# 后台发送管理器（模拟位置序列 + 硬件正弦保活）
# ---------------------------------------------------------------------------

class DianghuiWaveManager:
    """
    循环向 DianghuiController 只写指令并定时排空接收缓冲。

    两种互斥工作模式（发送间隔独立）：
      1. 模拟位置序列：500 Hz（ANALOG_INTERVAL_MS）循环下发闭环位置点列，
         每个点 ANALOG_BURST_N 帧连发（当前 10 帧；硬件正弦不走连发）
      2. 硬件正弦保活：1 kHz（WAVE_INTERVAL_MS）循环下发正弦扫描帧（0x04/0x05/0x06）

    设备每帧都会回状态，必须定时 drain，否则 USB 接收缓冲会溢出。
    """

    _INTERVAL: float = WAVE_INTERVAL_MS / 1000.0
    _ANALOG_INTERVAL: float = ANALOG_INTERVAL_MS / 1000.0

    # 每 50ms 解析一次最新状态帧并清空接收缓冲
    _STATUS_READ_INTERVAL: float = 0.05

    def __init__(self, controller: DianghuiController):
        self._ctrl = controller
        self._lock = threading.Lock()

        self._pts_x:    Optional[list[int]] = None
        self._pts_y:    Optional[list[int]] = None
        self._static_x: int = 0
        self._static_y: int = 0
        self._idx_x:    int = 0
        self._idx_y:    int = 0
        self._last_status: Optional[DianghuiStatus] = None

        self._sine_mode: Optional[str] = None  # None | 'x' | 'y' | 'xy'
        self._sine_freq: int = 0
        self._sine_amp:  int = 0

        self._thread:     Optional[threading.Thread] = None
        self._stop_event: threading.Event = threading.Event()

        self._last_status_read_t: float = 0.0

        self.invert_x: bool = False
        self.invert_y: bool = False
        self.analog_burst: bool = True        # True=每点连发 ANALOG_BURST_N 帧；False=单帧
        self.analog_interval_ms: int = ANALOG_INTERVAL_MS  # 发送间隔(ms)：2=500Hz，1=1000Hz
        self.analog_interpolate: bool = False  # True=连发时线性插值；False=重复同一点

        self._rec_lock:   threading.Lock = threading.Lock()
        self._rec_active: bool           = False
        self._rec_t0:     float          = 0.0
        self._rec_times:  list[float]    = []
        self._rec_x:      list[int]      = []
        self._rec_y:      list[int]      = []

    @property
    def is_running(self) -> bool:
        return bool(self._thread and self._thread.is_alive())

    @property
    def last_status(self) -> Optional[DianghuiStatus]:
        with self._lock:
            return self._last_status

    @property
    def sine_active(self) -> bool:
        with self._lock:
            return self._sine_mode is not None

    def start_wave_x(self, setpoints: list):
        """启动物理X轴模拟位置序列。"""
        pts = [max(-SETPOINT_LIMIT, min(SETPOINT_LIMIT, round(p))) for p in setpoints]
        with self._lock:
            self._sine_mode = None
            self._pts_x = pts
            self._idx_x = 0
        self._ensure_running()

    def start_wave_y(self, setpoints: list):
        """启动物理Y轴模拟位置序列。"""
        pts = [max(-SETPOINT_LIMIT, min(SETPOINT_LIMIT, round(p))) for p in setpoints]
        with self._lock:
            self._sine_mode = None
            self._pts_y = pts
            self._idx_y = 0
        self._ensure_running()

    def start_wave_xy(self, setpoints_x: list, setpoints_y: list):
        """同时启动双轴模拟位置序列（索引同时归零）。"""
        pts_x = [max(-SETPOINT_LIMIT, min(SETPOINT_LIMIT, round(p))) for p in setpoints_x]
        pts_y = [max(-SETPOINT_LIMIT, min(SETPOINT_LIMIT, round(p))) for p in setpoints_y]
        with self._lock:
            self._sine_mode = None
            self._pts_x = pts_x
            self._idx_x = 0
            self._pts_y = pts_y
            self._idx_y = 0
        self._ensure_running()

    def start_hw_sine(self, mode: str, frequency: int, amplitude: int):
        """
        启动硬件正弦扫描，并按 1 kHz 保活下发。

        :param mode:      物理轴 'x' / 'y' / 'xy'
        :param frequency: 频率 Hz（0~5000）
        :param amplitude: 峰值（界面峰峰值须先 /2）
        """
        if mode not in SINE_CMD_PHYS:
            raise ValueError(f"正弦模式须为 'x'/'y'/'xy'，收到: {mode}")
        freq = max(0, min(SINE_FREQ_MAX, int(round(frequency))))
        amp  = max(-SETPOINT_LIMIT, min(SETPOINT_LIMIT, int(round(amplitude))))
        with self._lock:
            self._pts_x = None
            self._pts_y = None
            self._sine_mode = mode
            self._sine_freq = freq
            self._sine_amp  = amp
        self._ensure_running()

    def stop_wave_x(self, static_value: int = 0):
        """停止物理X轴模拟序列，改为发送固定静态值。"""
        with self._lock:
            self._pts_x    = None
            self._static_x = max(-SETPOINT_LIMIT, min(SETPOINT_LIMIT, int(static_value)))
            self._idx_x    = 0

    def stop_wave_y(self, static_value: int = 0):
        """停止物理Y轴模拟序列，改为发送固定静态值。"""
        with self._lock:
            self._pts_y    = None
            self._static_y = max(-SETPOINT_LIMIT, min(SETPOINT_LIMIT, int(static_value)))
            self._idx_y    = 0

    def update_static_x(self, value: int):
        with self._lock:
            self._static_x = max(-SETPOINT_LIMIT, min(SETPOINT_LIMIT, int(value)))

    def update_static_y(self, value: int):
        with self._lock:
            self._static_y = max(-SETPOINT_LIMIT, min(SETPOINT_LIMIT, int(value)))

    def start_recording(self) -> None:
        with self._rec_lock:
            self._rec_times.clear()
            self._rec_x.clear()
            self._rec_y.clear()
            self._rec_t0   = perf_counter()
            self._rec_active = True

    def stop_recording(self) -> None:
        with self._rec_lock:
            self._rec_active = False

    @property
    def recording_active(self) -> bool:
        return self._rec_active

    @property
    def recording_sample_count(self) -> int:
        with self._rec_lock:
            return len(self._rec_times)

    def get_recording(self) -> tuple[list[float], list[int], list[int]]:
        with self._rec_lock:
            return (
                [t * 1000.0 for t in self._rec_times],
                list(self._rec_x),
                list(self._rec_y),
            )

    def save_recording_csv(self, path: str) -> int:
        import csv as _csv
        times_ms, x_data, y_data = self.get_recording()
        with open(path, 'w', newline='', encoding='utf-8-sig') as f:
            w = _csv.writer(f)
            w.writerow(['时间_ms', 'X轴反馈_counts', 'Y轴反馈_counts'])
            for t, x, y in zip(times_ms, x_data, y_data):
                w.writerow([f'{t:.3f}', x, y])
        return len(times_ms)

    def _push_record(self, status: DianghuiStatus) -> None:
        if not self._rec_active:
            return
        t = perf_counter() - self._rec_t0
        with self._rec_lock:
            self._rec_times.append(t)
            self._rec_x.append(status.x_feedback)
            self._rec_y.append(status.y_feedback)

    def stop_all(self):
        """停止所有波形/正弦保活和后台线程（阻塞等待至多 1 秒）。"""
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        with self._lock:
            self._pts_x = None
            self._pts_y = None
            self._sine_mode = None
        self._stop_event.clear()

    def _ensure_running(self):
        if not self.is_running:
            self._stop_event.clear()
            self._thread = threading.Thread(
                target=self._run, daemon=True, name="DianghuiWave"
            )
            self._thread.start()

    def _run(self):
        """
        后台发送循环（perf_counter 补偿）。

        硬件正弦：1 kHz 循环下发正弦帧，下位机自行振荡（单帧，不连发）。
        模拟位置：500 Hz 取点，每个点连发 ANALOG_BURST_N 帧闭环指令。
        两种都只写不等待，定时 drain 接收缓冲，防止溢出。
        """
        next_t = perf_counter()
        while not self._stop_event.is_set():
            with self._lock:
                sine_mode = self._sine_mode
                sine_freq = self._sine_freq
                sine_amp  = self._sine_amp
                active_x  = self._pts_x is not None
                active_y  = self._pts_y is not None

                if sine_mode is None and not active_x and not active_y:
                    break

                x_cur = x_next = 0
                y_cur = y_next = 0
                if sine_mode is None:
                    if active_x:
                        x_cur        = self._pts_x[self._idx_x]
                        x_next       = self._pts_x[(self._idx_x + 1) % len(self._pts_x)]
                        self._idx_x  = (self._idx_x + 1) % len(self._pts_x)
                    else:
                        x_cur = x_next = self._static_x
                    if active_y:
                        y_cur        = self._pts_y[self._idx_y]
                        y_next       = self._pts_y[(self._idx_y + 1) % len(self._pts_y)]
                        self._idx_y  = (self._idx_y + 1) % len(self._pts_y)
                    else:
                        y_cur = y_next = self._static_y
                    # 极性反转（对当前点和下一点都处理，保证插值方向正确）
                    if self.invert_x:
                        x_cur  = max(-SETPOINT_LIMIT, min(SETPOINT_LIMIT, -x_cur))
                        x_next = max(-SETPOINT_LIMIT, min(SETPOINT_LIMIT, -x_next))
                    if self.invert_y:
                        y_cur  = max(-SETPOINT_LIMIT, min(SETPOINT_LIMIT, -y_cur))
                        y_next = max(-SETPOINT_LIMIT, min(SETPOINT_LIMIT, -y_next))
                # 在锁内快照动态配置，避免发送途中被主线程修改
                do_burst       = self.analog_burst
                do_interpolate = self.analog_interpolate
                analog_iv_ms   = self.analog_interval_ms

            try:
                if sine_mode is not None:
                    # 硬件正弦保活：单帧，1 kHz
                    self._ctrl.write_sine_only(sine_mode, sine_freq, sine_amp)
                    interval = self._INTERVAL
                    drain_iv = self._STATUS_READ_INTERVAL
                elif do_burst:
                    # 连发模式：ANALOG_BURST_N 帧 / 点
                    if do_interpolate:
                        # 线性插值：当前点 → 下一点，生成 N 个不同值
                        n = ANALOG_BURST_N
                        pairs: list[tuple[int, int]] = []
                        for i in range(n):
                            t  = i / n
                            xi = round(x_cur + t * (x_next - x_cur))
                            yi = round(y_cur + t * (y_next - y_cur))
                            xi = max(-SETPOINT_LIMIT, min(SETPOINT_LIMIT, xi))
                            yi = max(-SETPOINT_LIMIT, min(SETPOINT_LIMIT, yi))
                            pairs.append((xi, yi))
                    else:
                        # 重复同一点
                        pairs = [(x_cur, y_cur)] * ANALOG_BURST_N
                    self._ctrl.write_control_burst(pairs)
                    interval = analog_iv_ms / 1000.0
                    drain_iv = min(self._STATUS_READ_INTERVAL, 0.02)
                else:
                    # 单帧模式
                    self._ctrl.write_control_only(x_phys=x_cur, y_phys=y_cur)
                    interval = analog_iv_ms / 1000.0
                    drain_iv = self._STATUS_READ_INTERVAL

                now = perf_counter()
                if now - self._last_status_read_t >= drain_iv:
                    self._last_status_read_t = now
                    status = self._ctrl.drain_read_buffer()
                    if status is not None:
                        with self._lock:
                            self._last_status = status
                        self._push_record(status)
            except Exception:
                break

            next_t += interval
            remaining = next_t - perf_counter()
            if remaining > 0.002:
                sleep(remaining)
            elif remaining > 0:
                _spin_until(next_t)

        try:
            self._ctrl.drain_read_buffer()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# 帧调试工具（可选）
# ---------------------------------------------------------------------------

def frame_to_hex(frame: bytes) -> str:
    """将帧字节串格式化为易读十六进制字符串，如 '7E E7 00 ...'。"""
    return ' '.join(f'{b:02X}' for b in frame)


def preview_tx_frame(
    cmd: int,
    x_phys: int,
    y_phys: int,
    x_lim: int = DEFAULT_LIMIT,
    y_lim: int = DEFAULT_LIMIT,
) -> str:
    """
    预览（不发送）控制帧内容（已做轴交换）。
    用于调试，确认帧结构正确。
    """
    frame = _build_tx_frame(
        cmd         = cmd,
        proto_x_set = y_phys,   # 物理Y → 协议X
        proto_y_set = x_phys,   # 物理X → 协议Y
        proto_x_lim = y_lim,
        proto_y_lim = x_lim,
    )
    return frame_to_hex(frame)
