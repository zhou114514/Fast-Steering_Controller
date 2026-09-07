"""
巅慧快反串口通信协议封装模块

串口参数: 115200 baud, 8N1（默认，可配置）
最小通信间隔: 1ms（发送一帧指令后立即收到状态帧，交互对称）

指令帧格式 (13 bytes):
  byte  0-1 : 帧头 0x7E 0xE7
  byte  2   : 指令码
               0x00 = 控制 (Control)
               0x01 = 回读 (Readback)
               0x04 = 复位 (Reset)
               0x05 = 自检 (Self-check)
  byte  3-4 : 协议 X 轴设定值 (int16, big-endian)  [注: 协议X ↔ 物理Y]
  byte  5-6 : 协议 Y 轴设定值 (int16, big-endian)  [注: 协议Y ↔ 物理X]
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
  物理X轴 (channel 0) ↔ 协议Y字段
  物理Y轴 (channel 1) ↔ 协议X字段
  原因: 设备实际接线与期望运动轴相反，协议层统一处理交换以对外透明。
"""

from __future__ import annotations

import math
import struct
import threading
from dataclasses import dataclass
from math import gcd
from time import perf_counter, sleep
from typing import Optional

import serial


# ---------------------------------------------------------------------------
# 帧常量
# ---------------------------------------------------------------------------

FRAME_HEADER   = bytes([0x7E, 0xE7])

CMD_STANDBY    = 0x00  # 待机：自由态，X/Y 轴均不使能
CMD_CLOSED_LOOP = 0x01  # 闭环：X/Y 轴使能，闭环位置控制（发送位置 / 读取状态）
CMD_OPEN_LOOP   = 0x04  # 开环：轴不使能
CMD_SELFCHECK   = 0x05  # 自检

TX_LEN = 13   # 指令帧总长度（bytes）
RX_LEN = 10   # 状态帧总长度（bytes）

DEFAULT_LIMIT  = 5000   # 初始化时默认限位值
SETPOINT_LIMIT = 25000  # 程序软限位：X/Y 轴设定值不超过 ±25000

# ---------------------------------------------------------------------------
# 实际波形发送间隔配置
# ---------------------------------------------------------------------------
# USB CDC 串口驱动的单次收发往返延迟通常约 2ms（取决于 USB 轮询间隔和驱动缓冲）。
# perf_counter 补偿算法在 send_control 本身耗时 > 目标间隔时失效，
# 因此必须把目标间隔设为实际可达值，否则频率会系统性偏低。
#
# 调整方法：查看录制 CSV 中时间戳的最小间隔，将该值填入 WAVE_INTERVAL_MS。
# 协议允许最小 1ms，但 USB 驱动通常只能稳定达到 2ms。
WAVE_INTERVAL_MS: int = 1                             # 实际发送间隔（ms）；按硬件调整
_WAVE_SAMPLE_RATE_HZ: int = 1000 // WAVE_INTERVAL_MS  # 有效采样率（Hz）


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
        0x01 既是闭环控制帧也是唯一能获取状态帧的方式。
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
        """切换为开环模式（0x04）：轴不使能。"""
        with self._lock:
            frame = _build_tx_frame(
                cmd         = CMD_OPEN_LOOP,
                proto_x_set = 0,
                proto_y_set = 0,
                proto_x_lim = DEFAULT_LIMIT,
                proto_y_lim = DEFAULT_LIMIT,
            )
            return self._send_recv_unlocked(frame)

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
# 正弦设定值生成（巅慧专用，1ms 采样间隔，输出有符号整型）
# ---------------------------------------------------------------------------

def generate_sine_setpoints(
    peak_peak: float,
    frequency: float,
    offset: float = 0.0,
) -> list[int]:
    """
    生成巅慧协议所需的正弦位置设定值列表（1ms 采样，输出有符号整数）。

    算法设计原则
    ------------
    1. **峰峰值精确命中**：使用余弦（cosine）而非正弦，第 0 帧恰好落在正峰值
       ``offset + peak_peak/2``，保证极值被完整表达，无采样截断误差。

    2. **序列长度确定**：

       - 若单周期 ``T = 1000 / frequency`` 恰好为整数 ms
         → 序列长度 = T（1 个完整周期）

       - 若 T 不是整数 ms（如 80Hz→12.5ms, 120Hz→8.333ms）
         → 序列长度 = 1000（整整 1 秒），覆盖 ``frequency`` 个完整周期，
         循环点处连续无跳变

    3. **循环点连续**：余弦在 ``t = N × T`` 处值恒为 ``amplitude``（与 t=0 相同），
       loop 衔接点无阶跃。

    各常用频率的序列长度（以 WAVE_INTERVAL_MS=2ms, 采样率500Hz 为例）::

        20  Hz → T=50ms（整数）→   25 点（1 个周期，25帧×2ms）
        80  Hz → T=12.5ms       →  500 点（80 个周期，500帧×2ms=1s）
        120 Hz → T=8.333ms      →  500 点（120 个周期）
        140 Hz → T=7.143ms      →  500 点（140 个周期）
        160 Hz → T=6.25ms       →  500 点（160 个周期）

    注：WAVE_INTERVAL_MS 修改后，序列长度自动跟随调整。

    :param peak_peak: 峰峰值（int16 原始码值，须 ≥ 0；建议 ≤ 2 × SETPOINT_LIMIT）
    :param frequency: 正弦频率（Hz），须为正整数
    :param offset:    中心偏置（int16 码值），默认 0
    :return:          有符号整数列表，已夹限到 ±SETPOINT_LIMIT；
                      第 0 个元素恰好等于 ``clamp(round(offset + peak_peak/2))``
    :raises ValueError: 频率为非正整数
    """
    if frequency <= 0:
        raise ValueError(f"频率必须为正数，收到: {frequency}")
    if peak_peak < 0:
        raise ValueError(f"峰峰值不能为负，收到: {peak_peak}")

    freq_int = int(round(frequency))
    if not math.isclose(frequency, freq_int, abs_tol=1e-9):
        raise ValueError(
            f"频率须为整数 Hz（以便与 1ms 采样对齐），收到: {frequency}"
        )

    # ── 序列长度 ──────────────────────────────────────────────────────────
    # 若单周期恰好为整数 ms → 一个周期即可完美循环
    # 否则 → 用 1000 点（1 秒）覆盖 freq 个完整周期
    T_ms = _WAVE_SAMPLE_RATE_HZ / freq_int          # 单周期 ms（可能非整数）
    T_ms_rounded = round(T_ms)
    if math.isclose(T_ms, T_ms_rounded, abs_tol=1e-9):
        n_points = T_ms_rounded                      # 整数 ms → 最短闭合长度
    else:
        n_points = _WAVE_SAMPLE_RATE_HZ              # 非整数 ms → 整 1 秒

    # ── 采样 ──────────────────────────────────────────────────────────────
    # 使用余弦：i=0 时 cos(0)=1 → 第 0 帧精确等于正峰值
    amplitude = peak_peak / 2.0
    dt = 1.0 / _WAVE_SAMPLE_RATE_HZ                 # 0.001 s
    return [
        max(-SETPOINT_LIMIT, min(SETPOINT_LIMIT, round(
            offset + amplitude * math.cos(2.0 * math.pi * freq_int * i * dt)
        )))
        for i in range(n_points)
    ]


# ---------------------------------------------------------------------------
# 三角波设定值生成（巅慧专用，恒速线性往复）
# ---------------------------------------------------------------------------

def generate_triangle_setpoints(
    peak_peak: float,
    frequency: float,
    offset: float = 0.0,
) -> list[int]:
    """
    生成三角波位置设定值列表（恒速线性往复，有符号整数输出）。

    **一个完整周期的波形**（从中心 offset 出发）::

        0 ──上升──► +peak_peak/2 ──下降──► 0 ──下降──► -peak_peak/2 ──上升──► 0

    **序列长度**：固定 1 秒（_WAVE_SAMPLE_RATE_HZ 点），包含恰好
    ``frequency`` 个完整周期，循环发送即可实现持续往复运动。

    **相位计算**：使用整数运算 ``raw = (i × freq) % sample_rate``
    避免浮点累积误差，每条周期的边界精确对齐到 0（中心位置）。

    与 generate_sine_setpoints 的对比::

        正弦波：变速，端点速度=0（平滑）；中点速度最大
        三角波：匀速线性扫描；端点处速度瞬间反向

    :param peak_peak: 峰峰值（int16 原始码值，须 ≥ 0）
    :param frequency: 频率（Hz），须为正整数
    :param offset:    中心偏置，默认 0；第 0 帧恰好输出 offset
    :return:          有符号整数列表（长度 = _WAVE_SAMPLE_RATE_HZ），
                      已夹限到 ±SETPOINT_LIMIT
    :raises ValueError: 频率为非正整数
    """
    if frequency <= 0:
        raise ValueError(f"频率必须为正数，收到: {frequency}")
    if peak_peak < 0:
        raise ValueError(f"峰峰值不能为负，收到: {peak_peak}")

    freq_int = int(round(frequency))
    if not math.isclose(frequency, freq_int, abs_tol=1e-9):
        raise ValueError(
            f"频率须为整数 Hz（以便与 {WAVE_INTERVAL_MS}ms 采样对齐），收到: {frequency}"
        )

    # ── 序列长度：固定 1 秒，包含 freq_int 个完整周期 ─────────────────────
    n = _WAVE_SAMPLE_RATE_HZ   # e.g. 500 @ 2ms → 1 second

    # ── 整数相位三角波 ────────────────────────────────────────────────────
    # raw = (i × freq_int) % n  → 整数，表示当前帧在 [0, n) 内的相位计数
    # 归一化 phase = raw / n ∈ [0, 1)
    # 分四段线性插值（周期从 0 出发）：
    #   [0.00, 0.25)  → 上升：0 → +amplitude
    #   [0.25, 0.75)  → 下降：+amplitude → -amplitude
    #   [0.75, 1.00)  → 上升：-amplitude → 0
    amplitude = peak_peak / 2.0
    Q = n // 4   # 四分之一周期的整数相位步数（n=500 时 Q=125）

    result = []
    for i in range(n):
        raw = (i * freq_int) % n          # 整数相位 [0, n)
        if raw < Q:                        # 上升段：0 → +amplitude
            val = offset + amplitude * raw / Q
        elif raw < 3 * Q:                  # 下降段：+amplitude → -amplitude
            val = offset + amplitude * (2.0 - raw / Q)
        else:                              # 上升段：-amplitude → 0
            val = offset + amplitude * (raw / Q - 4.0)
        result.append(max(-SETPOINT_LIMIT, min(SETPOINT_LIMIT, round(val))))

    return result


# ---------------------------------------------------------------------------
# 方波设定值生成（巅慧专用）
# ---------------------------------------------------------------------------

def generate_square_setpoints(
    peak_peak: float,
    frequency: float,
    offset: float = 0.0,
) -> list[int]:
    """
    生成方波位置设定值列表（前半周期正峰值，后半周期负峰值）。

    一个完整周期::

        +peak_peak/2 ... +peak_peak/2 │ -peak_peak/2 ... -peak_peak/2
        ←── 前半周期（峰值）──────────│──── 后半周期（谷值）──────────→

    序列长度固定 1 秒（_WAVE_SAMPLE_RATE_HZ 点），包含 ``frequency`` 个周期。
    第 0 帧从正峰值开始。相位使用整数运算 ``(i × freq) % n``。

    :param peak_peak: 峰峰值（int16 原始码值，须 ≥ 0）
    :param frequency: 频率（Hz），须为正整数
    :param offset:    中心偏置，默认 0
    :return:          有符号整数列表（长度 = _WAVE_SAMPLE_RATE_HZ），
                      每个元素为 offset ± peak_peak/2（已夹限）
    :raises ValueError: 频率为非正整数
    """
    if frequency <= 0:
        raise ValueError(f"频率必须为正数，收到: {frequency}")
    if peak_peak < 0:
        raise ValueError(f"峰峰值不能为负，收到: {peak_peak}")

    freq_int = int(round(frequency))
    if not math.isclose(frequency, freq_int, abs_tol=1e-9):
        raise ValueError(
            f"频率须为整数 Hz（以便与 {WAVE_INTERVAL_MS}ms 采样对齐），收到: {frequency}"
        )

    n         = _WAVE_SAMPLE_RATE_HZ          # 固定 1 秒
    amplitude = peak_peak / 2.0
    half_n    = n // 2                         # 相位计数半周期阈值

    pos_val = max(-SETPOINT_LIMIT, min(SETPOINT_LIMIT, round(offset + amplitude)))
    neg_val = max(-SETPOINT_LIMIT, min(SETPOINT_LIMIT, round(offset - amplitude)))

    return [
        pos_val if (i * freq_int) % n < half_n else neg_val
        for i in range(n)
    ]


# ---------------------------------------------------------------------------
# 软件正弦波管理器（巅慧专用）
# ---------------------------------------------------------------------------

class DianghuiWaveManager:
    """
    以 1ms 为目标间隔，循环向 DianghuiController 发送预生成的整数设定值列表。

    双轴独立配置：X 轴和 Y 轴可各自运行不同频率/幅度的正弦，也可只运行一轴。
    每次 send_control 的返回状态缓存到 last_status，供主线程轮询读取（避免主线程
    与发送线程争抢串口）。

    典型用法::

        from new_fsc_protocol import generate_sine_setpoints, SAMPLE_INTERVAL_MS_DIANGHUI

        mgr = DianghuiWaveManager(ctrl)
        pts = generate_sine_setpoints(1000, 20, offset=0, sample_interval_ms=1)
        mgr.start_wave_x(pts)          # 启动X轴正弦
        # ... 运行 ...
        mgr.stop_wave_x(static_value=0)  # 停止并归零
        mgr.stop_all()                  # 停止后台线程
    """

    _INTERVAL: float = WAVE_INTERVAL_MS / 1000.0  # 目标发送间隔（秒），与采样率同步

    # ── 只写模式（Write-Only）──────────────────────────────────────────────
    # 发送时不等待设备响应，可将实际间隔从 ~2ms 降至 ~1ms。
    # 若需要 1ms 控制，同时将顶部 WAVE_INTERVAL_MS 改为 1。
    WRITE_ONLY_MODE: bool = True   # True=只写（快速）；False=发送+等待响应（~2ms）

    # 是否在波形发送期间解析状态帧并更新状态显示。
    # True（默认）：每 _STATUS_READ_INTERVAL 秒 drain + 解析，状态灯刷新。
    # False：每 _DISCARD_INTERVAL 秒仅丢弃缓冲区（不解析），状态灯停止刷新，
    #        发送线程完全专注写入；注意：即使 False 也必须定期丢弃接收缓冲区，
    #        否则设备响应以 ~5KB/s（@2ms）积压 0.8-1.6s 后缓冲溢出、USB 通信中断、波形停止。
    DRAIN_DURING_WAVE:    bool  = True
    _STATUS_READ_INTERVAL: float = 0.5   # DRAIN_DURING_WAVE=True  时的解析 + 丢弃周期（秒）
    _DISCARD_INTERVAL:    float = 0.05   # DRAIN_DURING_WAVE=False 时的仅丢弃周期（秒）

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

        self._thread:     Optional[threading.Thread] = None
        self._stop_event: threading.Event = threading.Event()

        self._last_status_read_t: float = 0.0  # 只写模式：上次读取状态的时间

        # ── 数据记录 ───────────────────────────────────────────────────────
        self._rec_lock:   threading.Lock = threading.Lock()
        self._rec_active: bool           = False
        self._rec_t0:     float          = 0.0
        self._rec_times:  list[float]    = []   # 相对时间（秒）
        self._rec_x:      list[int]      = []   # 物理X反馈
        self._rec_y:      list[int]      = []   # 物理Y反馈

    # ------------------------------------------------------------------
    # 属性
    # ------------------------------------------------------------------

    @property
    def is_running(self) -> bool:
        return bool(self._thread and self._thread.is_alive())

    @property
    def last_status(self) -> Optional[DianghuiStatus]:
        """最近一次 send_control 收到的状态帧（线程安全只读）。"""
        with self._lock:
            return self._last_status

    # ------------------------------------------------------------------
    # 控制接口
    # ------------------------------------------------------------------

    def start_wave_x(self, setpoints: list):
        """
        启动物理X轴正弦波。浮点列表自动四舍五入为 int16。
        如后台线程未在运行则自动启动。
        """
        pts = [max(-SETPOINT_LIMIT, min(SETPOINT_LIMIT, round(p))) for p in setpoints]
        with self._lock:
            self._pts_x = pts
            self._idx_x = 0
        self._ensure_running()

    def start_wave_y(self, setpoints: list):
        """启动物理Y轴正弦波。"""
        pts = [max(-SETPOINT_LIMIT, min(SETPOINT_LIMIT, round(p))) for p in setpoints]
        with self._lock:
            self._pts_y = pts
            self._idx_y = 0
        self._ensure_running()

    def start_wave_xy(self, setpoints_x: list, setpoints_y: list):
        """
        原子性同时启动双轴波形，保证相位完全同步（零相位差）。

        与分别调用 start_wave_x / start_wave_y 相比，此方法在同一把锁内
        同时将两轴索引归零，消除顺序调用时线程先行推进 X 轴导致的相位偏差。

        例：80Hz 下两轴各差 1 帧（2ms）= 57.6° 相位差，会产生明显椭圆；
        使用此方法保证 0° 相位差，呈现理想的斜线扫描。
        """
        pts_x = [max(-SETPOINT_LIMIT, min(SETPOINT_LIMIT, round(p))) for p in setpoints_x]
        pts_y = [max(-SETPOINT_LIMIT, min(SETPOINT_LIMIT, round(p))) for p in setpoints_y]
        with self._lock:               # 单次锁内同时归零，线程无法插入
            self._pts_x = pts_x
            self._idx_x = 0
            self._pts_y = pts_y
            self._idx_y = 0
        self._ensure_running()

    def stop_wave_x(self, static_value: int = 0):
        """停止物理X轴波形，改为发送固定静态值。"""
        with self._lock:
            self._pts_x    = None
            self._static_x = max(-SETPOINT_LIMIT, min(SETPOINT_LIMIT, int(static_value)))
            self._idx_x    = 0

    def stop_wave_y(self, static_value: int = 0):
        """停止物理Y轴波形，改为发送固定静态值。"""
        with self._lock:
            self._pts_y    = None
            self._static_y = max(-SETPOINT_LIMIT, min(SETPOINT_LIMIT, int(static_value)))
            self._idx_y    = 0

    def update_static_x(self, value: int):
        """更新X轴静态目标值（不中断正在运行的波形序列）。"""
        with self._lock:
            self._static_x = max(-SETPOINT_LIMIT, min(SETPOINT_LIMIT, int(value)))

    def update_static_y(self, value: int):
        """更新Y轴静态目标值（不中断正在运行的波形序列）。"""
        with self._lock:
            self._static_y = max(-SETPOINT_LIMIT, min(SETPOINT_LIMIT, int(value)))

    # ------------------------------------------------------------------
    # 数据记录
    # ------------------------------------------------------------------

    def start_recording(self) -> None:
        """开始记录每帧的位置反馈数据，会清空上次的记录。"""
        with self._rec_lock:
            self._rec_times.clear()
            self._rec_x.clear()
            self._rec_y.clear()
            self._rec_t0   = perf_counter()
            self._rec_active = True

    def stop_recording(self) -> None:
        """停止记录。已记录的数据可通过 get_recording / save_recording_csv 获取。"""
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
        """
        返回已记录数据的副本：(时间列表_ms, X轴反馈列表, Y轴反馈列表)。
        时间单位为毫秒，以第一帧为 0 基准。
        """
        with self._rec_lock:
            return (
                [t * 1000.0 for t in self._rec_times],
                list(self._rec_x),
                list(self._rec_y),
            )

    def save_recording_csv(self, path: str) -> int:
        """
        将记录数据保存为 CSV 文件。
        列：时间_ms, X轴反馈_counts, Y轴反馈_counts
        返回已保存的样本数。
        """
        import csv as _csv
        times_ms, x_data, y_data = self.get_recording()
        with open(path, 'w', newline='', encoding='utf-8-sig') as f:
            w = _csv.writer(f)
            w.writerow(['时间_ms', 'X轴反馈_counts', 'Y轴反馈_counts'])
            for t, x, y in zip(times_ms, x_data, y_data):
                w.writerow([f'{t:.3f}', x, y])
        return len(times_ms)

    def _push_record(self, status: DianghuiStatus) -> None:
        """（内部）线程中记录一帧位置数据。"""
        if not self._rec_active:
            return
        t = perf_counter() - self._rec_t0
        with self._rec_lock:
            self._rec_times.append(t)
            self._rec_x.append(status.x_feedback)
            self._rec_y.append(status.y_feedback)

    def stop_all(self):
        """停止所有波形和后台线程（阻塞等待至多 1 秒）。"""
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        with self._lock:
            self._pts_x = None
            self._pts_y = None
        self._stop_event.clear()

    # ------------------------------------------------------------------
    # 后台线程
    # ------------------------------------------------------------------

    def _ensure_running(self):
        if not self.is_running:
            self._stop_event.clear()
            self._thread = threading.Thread(
                target=self._run, daemon=True, name="DianghuiWave"
            )
            self._thread.start()

    def _run(self):
        """
        后台发送循环（1ms 间隔，perf_counter 补偿累积误差）。

        线程仅在至少一轴有活跃波形序列时运行：
        - 若 _pts_x/_pts_y 不为 None，循环取点（末尾自动回绕）持续发送，
          镜片保持往复运动，直到 stop_wave_x/y 将其清空。
        - 若两轴序列均为 None（波形全部停止），线程立即退出。
          设备在闭环控制下自持当前位置，无需上位机持续下发。
        """
        next_t = perf_counter()
        while not self._stop_event.is_set():
            with self._lock:
                active_x = self._pts_x is not None
                active_y = self._pts_y is not None

                # 双轴均无波形 → 退出线程，设备闭环自持位置
                if not active_x and not active_y:
                    break

                # X 轴取值
                if active_x:
                    x_val       = self._pts_x[self._idx_x]
                    self._idx_x = (self._idx_x + 1) % len(self._pts_x)
                else:
                    x_val = self._static_x

                # Y 轴取值
                if active_y:
                    y_val       = self._pts_y[self._idx_y]
                    self._idx_y = (self._idx_y + 1) % len(self._pts_y)
                else:
                    y_val = self._static_y

            try:
                if self.WRITE_ONLY_MODE:
                    # ── 只写路径（~0.1ms/帧，可达 1ms 控制间隔）────────────
                    self._ctrl.write_control_only(x_phys=x_val, y_phys=y_val)

                    # 定期批量读取接收缓冲区（防溢出 + 刷新状态显示）
                    # DRAIN_DURING_WAVE=False 时跳过，追求最纯粹的发送稳定性
                    now = perf_counter()
                    if self.DRAIN_DURING_WAVE:
                        # 解析模式：drain + 解析状态帧，刷新状态显示
                        if now - self._last_status_read_t >= self._STATUS_READ_INTERVAL:
                            self._last_status_read_t = now
                            status = self._ctrl.drain_read_buffer()
                            if status is not None:
                                with self._lock:
                                    self._last_status = status
                                self._push_record(status)
                    else:
                        # 纯丢弃模式：不解析，仅防止 USB 接收缓冲区溢出导致通信中断
                        # （设备每帧回 10 字节，@2ms=5KB/s，不丢弃则约 1s 内溢出）
                        if now - self._last_status_read_t >= self._DISCARD_INTERVAL:
                            self._last_status_read_t = now
                            try:
                                with self._ctrl._lock:
                                    if (self._ctrl._serial
                                            and self._ctrl._serial.is_open
                                            and self._ctrl._serial.in_waiting > 0):
                                        self._ctrl._serial.reset_input_buffer()
                            except Exception:
                                pass
                else:
                    # ── 完整收发路径（~2ms/帧，含读等待）────────────────────
                    status = self._ctrl.send_control(x_phys=x_val, y_phys=y_val)
                    if status is not None:
                        with self._lock:
                            self._last_status = status
                        self._push_record(status)
            except Exception:
                break  # 串口异常，退出；主线程轮询会发现断连

            next_t += self._INTERVAL
            remaining = next_t - perf_counter()
            if remaining > 0:
                sleep(remaining)

        # ── 线程退出前清空接收缓冲区 ──────────────────────────────────────
        # 波形停止后缓冲区可能残留 ≤100ms 的旧响应帧；
        # 若不清空，随后的 send_readback() 会读到错位旧数据导致虚假断连。
        if self.WRITE_ONLY_MODE:
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
