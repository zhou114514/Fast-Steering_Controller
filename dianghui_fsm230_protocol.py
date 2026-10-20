"""
巅慧 FSM230-L30 快反镜串口通信协议封装模块

与 dianghui_fsm720_protocol.py（FSM720 款）互相独立，对外接口基本保持一致，
上层（fsc_gui.py）可按同一套方式调用两款设备。

物理层: 异步 UART（RS422），921600 / 5 Mbps，8N1，补码，高字节在前（big-endian）
通讯方式: 收发应答式，设备收到有效帧后约 2 us 回一帧当前状态

指令帧（上位机 → 快反镜，8 bytes）:
  byte 0-1 : 帧头 0x5A 0x4A
  byte 2   : 命令  0x00 = 开环控制
                   0x01 = 闭环控制
                   0x02 = 驱动下电模式
  byte 3-4 : 方位轴（X）设定值 (int16, BE)，范围 -20000 ~ +20000，0.1 urad/LSB
  byte 5-6 : 俯仰轴（Y）设定值 (int16, BE)，范围 -20000 ~ +20000，0.1 urad/LSB
  byte 7   : 校验位 = (~sum(byte[2:7])) & 0xFF   （除帧头外所有字节累加取反取低 8 位）

状态帧（快反镜 → 上位机，7 bytes）:
  byte 0-1 : 帧头 0x5A 0x4A
  byte 2-3 : X 轴（方位）当前位移监控 (int16, BE)，0.1 urad/LSB
  byte 4-5 : Y 轴（俯仰）当前位移监控 (int16, BE)，0.1 urad/LSB
  byte 6   : 校验位 = (~sum(byte[2:6])) & 0xFF

与 FSM720 的主要区别:
  * 帧头 / 帧长 / 指令码不同；状态帧只含两轴位移，无使能、超限、自检等状态位
  * 无限位参数，无硬件正弦指令
  * 方位轴 = 物理 X（channel 0），俯仰轴 = 物理 Y（channel 1），协议层不做轴交换
  * 正弦由上位机软件生成：按 1 kHz 逐点下发闭环设定值。
    为保证 1 kHz 间隔稳定，正弦期间不解析 / 不刷新当前位置，只定期清空接收缓冲区。
  * 模拟任意波（位置文件）与 FSM720 的做法一致：按 500 Hz / 1 kHz 下发位置序列，
    可选多帧连发 / 线性插值，并定期解析接收缓冲区中最新状态帧用于显示和记录。

注意（文档存疑项）:
  协议文档写的校验范围为“除帧头外字节（序号 3~6）”。该描述与 RX 帧吻合（byte[2:6]）；
  对 TX 帧，这里按“除帧头外全部字节”处理（含命令字，即 byte[2:7]）。
  如实机验证为仅对数据字节求和，将 TX_CHECKSUM_INCLUDE_CMD 改为 False 即可。
"""

from __future__ import annotations

import math
import struct
import threading
from dataclasses import dataclass
from time import perf_counter, sleep
from typing import Optional

import serial


# ---------------------------------------------------------------------------
# 帧常量
# ---------------------------------------------------------------------------

FRAME_HEADER = bytes([0x5A, 0x4A])

CMD_OPEN_LOOP   = 0x00  # 开环控制
CMD_CLOSED_LOOP = 0x01  # 闭环控制（位置设定）
CMD_POWER_OFF   = 0x02  # 驱动下电模式

TX_LEN = 8   # 指令帧总长度（bytes）
RX_LEN = 7   # 状态帧总长度（bytes）

# 指令帧校验是否包含命令字节（见模块说明）
TX_CHECKSUM_INCLUDE_CMD = True

DEFAULT_BAUDRATE = 921600   # 文档给出 921600 / 5Mbps
URAD_PER_LSB     = 0.1      # 角度分辨率：0.1 urad/LSB
SETPOINT_LIMIT   = 20000    # 设定值范围 ±20000（= ±2000 urad）

# 公开接口使用物理轴：'x' / 'y' / 'xy'
SINE_AXES = ("x", "y", "xy")

# ---------------------------------------------------------------------------
# 发送间隔配置
# ---------------------------------------------------------------------------
# 软件正弦：固定 1 kHz 下发设定点。
SINE_RATE_HZ: int = 1000
# 软件正弦频率上限（Hz）：1 kHz 采样的奈奎斯特频率为 500 Hz，
# 频率越接近上限每周期点数越少（100 Hz = 10 点/周期），实际使用建议不超过 100~200 Hz。
SINE_FREQ_MAX: int = 500
# 模拟位置序列默认 500 Hz（2 ms）；可由上层切换为 1 ms。
ANALOG_INTERVAL_MS: int = 2
# 模拟发送每个点连发帧数（1 = 单帧，>1 = 连发）
ANALOG_BURST_N: int = 5


# ---------------------------------------------------------------------------
# 状态数据类
# ---------------------------------------------------------------------------

@dataclass
class DianghuiFsm230Status:
    """
    FSM230 状态帧解析结果（物理轴坐标：X = 方位轴，Y = 俯仰轴）。

    反馈值为 int16 原始码值，0.1 urad/LSB。
    """
    x_feedback: int
    y_feedback: int

    @property
    def x_urad(self) -> float:
        return self.x_feedback * URAD_PER_LSB

    @property
    def y_urad(self) -> float:
        return self.y_feedback * URAD_PER_LSB


class DianghuiFsm230Error(Exception):
    """巅慧 FSM230 快反通信异常。"""


# ---------------------------------------------------------------------------
# 底层帧工具
# ---------------------------------------------------------------------------

def _spin_until(deadline: float) -> None:
    """忙等到 deadline。Windows 的 sleep 分辨率约 1ms，无法用于亚毫秒节拍。"""
    while perf_counter() < deadline:
        pass


def _clamp_setpoint(value: float) -> int:
    return max(-SETPOINT_LIMIT, min(SETPOINT_LIMIT, int(round(value))))


def _calc_checksum(data: bytes | list[int]) -> int:
    """计算校验位: (~sum(data)) & 0xFF"""
    return (~sum(data)) & 0xFF


def _build_tx_frame(cmd: int, x_set: int, y_set: int) -> bytes:
    """
    构造指令帧。

    :param cmd:   指令码（CMD_*）
    :param x_set: X 轴（方位）设定值 (int16)，自动限幅到 ±SETPOINT_LIMIT
    :param y_set: Y 轴（俯仰）设定值 (int16)，自动限幅到 ±SETPOINT_LIMIT
    """
    body = struct.pack('>Bhh', cmd, _clamp_setpoint(x_set), _clamp_setpoint(y_set))
    checksum = _calc_checksum(body if TX_CHECKSUM_INCLUDE_CMD else body[1:])
    return FRAME_HEADER + body + bytes([checksum])


def _parse_rx_frame(data: bytes) -> Optional[DianghuiFsm230Status]:
    """解析状态帧，帧头 / 长度 / 校验任一不符返回 None。"""
    if len(data) != RX_LEN:
        return None
    if data[0:2] != FRAME_HEADER:
        return None
    if _calc_checksum(data[2:6]) != data[6]:
        return None
    x_fb, y_fb = struct.unpack('>hh', data[2:6])
    return DianghuiFsm230Status(x_feedback=x_fb, y_feedback=y_fb)


def _scan_latest_status(raw: bytes) -> Optional[DianghuiFsm230Status]:
    """从任意字节流中扫描最后一个校验通过的状态帧。"""
    last: Optional[DianghuiFsm230Status] = None
    h0, h1 = FRAME_HEADER
    for start in range(len(raw) - RX_LEN + 1):
        if raw[start] == h0 and raw[start + 1] == h1:
            st = _parse_rx_frame(bytes(raw[start:start + RX_LEN]))
            if st is not None:
                last = st
    return last


# ---------------------------------------------------------------------------
# 控制器
# ---------------------------------------------------------------------------

class DianghuiFsm230Controller:
    """
    巅慧 FSM230 串口控制器。

    所有公开接口使用物理轴坐标（X = 方位 = channel 0，Y = 俯仰 = channel 1）。
    线程安全（内置 Lock，可在主线程和波形线程中并发调用）。

    使用示例::

        with DianghuiFsm230Controller(port='COM3') as dh:
            dh.send_control(x_phys=0, y_phys=0)
            dh.send_control(x_phys=1000, y_phys=-1000)   # 100 urad / -100 urad
    """

    def __init__(
        self,
        port: str,
        baudrate: int = DEFAULT_BAUDRATE,
        read_timeout: float = 0.05,
    ):
        self._port = port
        self._baudrate = baudrate
        self._timeout = read_timeout
        self._serial: Optional[serial.Serial] = None
        self._lock = threading.Lock()

        self._x_phys: int = 0
        self._y_phys: int = 0

    # ------------------------------------------------------------------
    # 串口生命周期
    # ------------------------------------------------------------------

    def open(self) -> None:
        self._serial = serial.Serial(
            port      = self._port,
            baudrate  = self._baudrate,
            bytesize  = serial.EIGHTBITS,
            parity    = serial.PARITY_NONE,
            stopbits  = serial.STOPBITS_ONE,
            timeout   = self._timeout,
        )

    def close(self) -> None:
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

    def _send_recv_unlocked(self, frame: bytes) -> Optional[DianghuiFsm230Status]:
        assert self._serial and self._serial.is_open, "串口未打开"
        # 清掉只写模式下积压的旧状态帧，避免读到错位数据
        try:
            self._serial.reset_input_buffer()
        except Exception:
            pass
        self._serial.write(frame)
        return _parse_rx_frame(self._serial.read(RX_LEN))

    def _closed_loop_frame(self) -> bytes:
        return _build_tx_frame(CMD_CLOSED_LOOP, self._x_phys, self._y_phys)

    # ------------------------------------------------------------------
    # 公开控制接口
    # ------------------------------------------------------------------

    def send_control(
        self,
        x_phys: int,
        y_phys: int,
        x_lim: Optional[int] = None,   # 仅为与 FSM720 接口兼容，FSM230 无限位参数，忽略
        y_lim: Optional[int] = None,
    ) -> Optional[DianghuiFsm230Status]:
        """
        闭环位置控制并等待状态帧。

        :param x_phys: X 轴（方位）设定值，0.1 urad/LSB，自动限幅 ±20000
        :param y_phys: Y 轴（俯仰）设定值
        :return: 解析后的状态帧；超时或校验失败返回 None
        """
        with self._lock:
            self._x_phys = _clamp_setpoint(x_phys)
            self._y_phys = _clamp_setpoint(y_phys)
            return self._send_recv_unlocked(self._closed_loop_frame())

    def send_readback(self) -> Optional[DianghuiFsm230Status]:
        """以闭环指令（保持当前设定值）读取状态。波形线程运行期间勿调用。"""
        with self._lock:
            return self._send_recv_unlocked(self._closed_loop_frame())

    def send_open_loop(self) -> Optional[DianghuiFsm230Status]:
        """开环控制（0x00），设定值沿用当前值。"""
        with self._lock:
            return self._send_recv_unlocked(
                _build_tx_frame(CMD_OPEN_LOOP, self._x_phys, self._y_phys)
            )

    def send_power_off(self) -> Optional[DianghuiFsm230Status]:
        """驱动下电模式（0x02）。再次发闭环指令即可恢复。"""
        with self._lock:
            return self._send_recv_unlocked(_build_tx_frame(CMD_POWER_OFF, 0, 0))

    def write_control_only(self, x_phys: int, y_phys: int) -> None:
        """仅写入闭环控制帧，不等待应答（高速只写）。应答由 drain / clear 处理。"""
        with self._lock:
            self._x_phys = _clamp_setpoint(x_phys)
            self._y_phys = _clamp_setpoint(y_phys)
            assert self._serial and self._serial.is_open, "串口未打开"
            self._serial.write(self._closed_loop_frame())

    def write_control_burst(self, pairs: list[tuple[int, int]]) -> None:
        """一次 serial.write 连续发送多帧闭环指令（模拟连发）。"""
        if not pairs:
            return
        blob = bytearray()
        with self._lock:
            assert self._serial and self._serial.is_open, "串口未打开"
            for x_phys, y_phys in pairs:
                self._x_phys = _clamp_setpoint(x_phys)
                self._y_phys = _clamp_setpoint(y_phys)
                blob += self._closed_loop_frame()
            self._serial.write(blob)

    def drain_read_buffer(self) -> Optional[DianghuiFsm230Status]:
        """
        读取并清空接收缓冲区，返回其中最新一帧有效状态（非阻塞）。
        用于模拟位置序列模式；无完整有效帧返回 None。
        """
        if not (self._serial and self._serial.is_open):
            return None
        with self._lock:
            n = self._serial.in_waiting
            if n < RX_LEN:
                return None
            raw = self._serial.read(n)
        return _scan_latest_status(raw)

    def clear_rx_buffer(self) -> None:
        """
        直接丢弃接收缓冲区内容，不做任何解析。
        用于 1 kHz 软件正弦期间：只防止缓冲区溢出，不占用发送节拍。
        """
        if not (self._serial and self._serial.is_open):
            return
        with self._lock:
            try:
                self._serial.reset_input_buffer()
            except Exception:
                pass

    # ------------------------------------------------------------------
    # 状态属性
    # ------------------------------------------------------------------

    @property
    def x_setpoint(self) -> int:
        return self._x_phys

    @property
    def y_setpoint(self) -> int:
        return self._y_phys


# ---------------------------------------------------------------------------
# 后台发送管理器（模拟位置序列 + 1 kHz 软件正弦）
# ---------------------------------------------------------------------------

class DianghuiFsm230WaveManager:
    """
    后台线程循环向 DianghuiFsm230Controller 只写指令。

    两种互斥工作模式：
      1. 软件正弦（start_sine）：固定 1 kHz，每个 tick 计算一个正弦设定值并发单帧。
         期间 **不解析、不刷新当前位置**（last_status 保持不变，status_available=False），
         仅每 50 ms 清空一次接收缓冲区，避免占用节拍、保证 1 ms 间隔稳定。
      2. 模拟位置序列（start_wave_*）：按 analog_interval_ms（2ms/1ms）逐点下发，
         可选 ANALOG_BURST_N 帧连发 / 线性插值；定期解析接收缓冲区中的最新状态帧
         并用于显示和数据记录。与 FSM720 做法一致。

    对外属性 / 方法名与 DianghuiWaveManager 对齐，便于上层复用。
    """

    _SINE_INTERVAL: float = 1.0 / SINE_RATE_HZ
    _STATUS_READ_INTERVAL: float = 0.05   # 解析最新状态帧的周期（模拟模式）
    _SINE_CLEAR_INTERVAL: float = 0.05    # 清空接收缓冲的周期（正弦模式）

    def __init__(self, controller: DianghuiFsm230Controller):
        self._ctrl = controller
        self._lock = threading.Lock()

        self._pts_x:    Optional[list[int]] = None
        self._pts_y:    Optional[list[int]] = None
        self._static_x: int = 0
        self._static_y: int = 0
        self._idx_x:    int = 0
        self._idx_y:    int = 0
        self._last_status: Optional[DianghuiFsm230Status] = None

        self._sine_mode:   Optional[str] = None   # None | 'x' | 'y' | 'xy'
        self._sine_freq:   float = 0.0
        self._sine_amp:    int = 0
        self._sine_off_x:  int = 0
        self._sine_off_y:  int = 0
        self._sine_phase:  float = 0.0            # 相位累加器（rad），频率变化时相位连续

        self._thread:     Optional[threading.Thread] = None
        self._stop_event: threading.Event = threading.Event()
        self.last_error:  Optional[Exception] = None

        # 动态配置（线程运行中可直接修改）
        self.invert_x: bool = False
        self.invert_y: bool = False
        self.analog_burst: bool = True
        self.analog_interval_ms: int = ANALOG_INTERVAL_MS
        self.analog_interpolate: bool = False

        self._rec_lock:   threading.Lock = threading.Lock()
        self._rec_active: bool           = False
        self._rec_t0:     float          = 0.0
        self._rec_times:  list[float]    = []
        self._rec_x:      list[int]      = []
        self._rec_y:      list[int]      = []

    # ------------------------------------------------------------------
    # 状态属性
    # ------------------------------------------------------------------

    @property
    def is_running(self) -> bool:
        return bool(self._thread and self._thread.is_alive())

    @property
    def last_status(self) -> Optional[DianghuiFsm230Status]:
        with self._lock:
            return self._last_status

    @property
    def sine_active(self) -> bool:
        with self._lock:
            return self._sine_mode is not None

    @property
    def status_available(self) -> bool:
        """
        线程运行期间是否仍在刷新位置。软件正弦期间为 False
        （不解析接收数据，last_status 为旧值，上层不应据此判断断连）。
        """
        with self._lock:
            return self._sine_mode is None

    # ------------------------------------------------------------------
    # 模拟位置序列
    # ------------------------------------------------------------------

    def start_wave_x(self, setpoints: list):
        """启动 X 轴模拟位置序列。"""
        pts = [_clamp_setpoint(p) for p in setpoints]
        with self._lock:
            self._sine_mode = None
            self._pts_x = pts
            self._idx_x = 0
        self._ensure_running()

    def start_wave_y(self, setpoints: list):
        """启动 Y 轴模拟位置序列。"""
        pts = [_clamp_setpoint(p) for p in setpoints]
        with self._lock:
            self._sine_mode = None
            self._pts_y = pts
            self._idx_y = 0
        self._ensure_running()

    def start_wave_xy(self, setpoints_x: list, setpoints_y: list):
        """同时启动双轴模拟位置序列（索引同时归零）。"""
        pts_x = [_clamp_setpoint(p) for p in setpoints_x]
        pts_y = [_clamp_setpoint(p) for p in setpoints_y]
        with self._lock:
            self._sine_mode = None
            self._pts_x = pts_x
            self._idx_x = 0
            self._pts_y = pts_y
            self._idx_y = 0
        self._ensure_running()

    def stop_wave_x(self, static_value: int = 0):
        with self._lock:
            self._pts_x = None
            self._static_x = _clamp_setpoint(static_value)
            self._idx_x = 0

    def stop_wave_y(self, static_value: int = 0):
        with self._lock:
            self._pts_y = None
            self._static_y = _clamp_setpoint(static_value)
            self._idx_y = 0

    def update_static_x(self, value: int):
        with self._lock:
            self._static_x = _clamp_setpoint(value)

    def update_static_y(self, value: int):
        with self._lock:
            self._static_y = _clamp_setpoint(value)

    # ------------------------------------------------------------------
    # 软件正弦
    # ------------------------------------------------------------------

    def start_sine(
        self,
        mode: str,
        frequency: float,
        amplitude: int,
        offset_x: int = 0,
        offset_y: int = 0,
    ):
        """
        启动 1 kHz 软件正弦。

        :param mode:      物理轴 'x' / 'y' / 'xy'；未参与的轴保持静态值（update_static_*）
        :param frequency: 频率 Hz（0~SINE_FREQ_MAX，可为小数）
        :param amplitude: 峰值（非峰峰值；界面峰峰值须先 /2），0.1 urad/LSB
        :param offset_x:  X 轴正弦中心（默认 0）
        :param offset_y:  Y 轴正弦中心（默认 0）
        """
        if mode not in SINE_AXES:
            raise ValueError(f"正弦模式须为 'x'/'y'/'xy'，收到: {mode}")
        freq = max(0.0, min(float(SINE_FREQ_MAX), float(frequency)))
        amp  = max(0, min(SETPOINT_LIMIT, int(round(abs(amplitude)))))
        with self._lock:
            if self._sine_mode is None:
                self._sine_phase = 0.0     # 新启动从零相位开始；运行中改参数则相位连续
            self._pts_x = None
            self._pts_y = None
            self._sine_mode  = mode
            self._sine_freq  = freq
            self._sine_amp   = amp
            self._sine_off_x = _clamp_setpoint(offset_x)
            self._sine_off_y = _clamp_setpoint(offset_y)
        self._ensure_running()

    # 与 DianghuiWaveManager 接口名对齐（FSM230 实为软件正弦）
    start_hw_sine = start_sine

    # ------------------------------------------------------------------
    # 数据记录（仅模拟位置序列模式有反馈数据）
    # ------------------------------------------------------------------

    def start_recording(self) -> None:
        with self._rec_lock:
            self._rec_times.clear()
            self._rec_x.clear()
            self._rec_y.clear()
            self._rec_t0 = perf_counter()
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

    def _push_record(self, status: DianghuiFsm230Status) -> None:
        if not self._rec_active:
            return
        t = perf_counter() - self._rec_t0
        with self._rec_lock:
            self._rec_times.append(t)
            self._rec_x.append(status.x_feedback)
            self._rec_y.append(status.y_feedback)

    # ------------------------------------------------------------------
    # 线程控制
    # ------------------------------------------------------------------

    def stop_all(self):
        """停止所有波形 / 正弦和后台线程（阻塞等待至多 1 秒）。"""
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
            self.last_error = None
            self._thread = threading.Thread(
                target=self._run, daemon=True, name="DianghuiFsm230Wave"
            )
            self._thread.start()

    def _run(self):
        """
        后台发送循环（perf_counter 绝对时刻补偿）。

        正弦：1 kHz 单帧，只清缓冲不读位置。
        模拟：analog_interval_ms 取点，可连发，定期 drain 并解析最新状态。
        """
        next_t = perf_counter()
        last_service_t = 0.0   # 上次清缓冲 / drain 的时刻

        while not self._stop_event.is_set():
            with self._lock:
                sine_mode = self._sine_mode
                active_x  = self._pts_x is not None
                active_y  = self._pts_y is not None

                if sine_mode is None and not active_x and not active_y:
                    break

                if sine_mode is not None:
                    # ---- 软件正弦：本 tick 的设定值 ----
                    s = self._sine_amp * math.sin(self._sine_phase)
                    self._sine_phase = (
                        self._sine_phase
                        + 2.0 * math.pi * self._sine_freq / SINE_RATE_HZ
                    ) % (2.0 * math.pi)
                    x_cur = (self._sine_off_x + s) if sine_mode in ("x", "xy") \
                        else self._static_x
                    y_cur = (self._sine_off_y + s) if sine_mode in ("y", "xy") \
                        else self._static_y
                    x_next = x_cur
                    y_next = y_cur
                else:
                    # ---- 模拟位置序列：当前点与下一点 ----
                    if active_x:
                        x_cur   = self._pts_x[self._idx_x]
                        x_next  = self._pts_x[(self._idx_x + 1) % len(self._pts_x)]
                        self._idx_x = (self._idx_x + 1) % len(self._pts_x)
                    else:
                        x_cur = x_next = self._static_x
                    if active_y:
                        y_cur   = self._pts_y[self._idx_y]
                        y_next  = self._pts_y[(self._idx_y + 1) % len(self._pts_y)]
                        self._idx_y = (self._idx_y + 1) % len(self._pts_y)
                    else:
                        y_cur = y_next = self._static_y

                # 极性反转（当前点与下一点都处理，保证插值方向正确）
                if self.invert_x:
                    x_cur, x_next = -x_cur, -x_next
                if self.invert_y:
                    y_cur, y_next = -y_cur, -y_next

                do_burst       = self.analog_burst
                do_interpolate = self.analog_interpolate
                analog_iv_ms   = self.analog_interval_ms

            try:
                if sine_mode is not None:
                    self._ctrl.write_control_only(x_cur, y_cur)
                    interval = self._SINE_INTERVAL
                    service_iv = self._SINE_CLEAR_INTERVAL
                elif do_burst:
                    n = ANALOG_BURST_N
                    if do_interpolate:
                        pairs = [
                            (
                                x_cur + (i / n) * (x_next - x_cur),
                                y_cur + (i / n) * (y_next - y_cur),
                            )
                            for i in range(n)
                        ]
                    else:
                        pairs = [(x_cur, y_cur)] * n
                    self._ctrl.write_control_burst(pairs)
                    interval = analog_iv_ms / 1000.0
                    service_iv = min(self._STATUS_READ_INTERVAL, 0.02)
                else:
                    self._ctrl.write_control_only(x_cur, y_cur)
                    interval = analog_iv_ms / 1000.0
                    service_iv = self._STATUS_READ_INTERVAL

                now = perf_counter()
                if now - last_service_t >= service_iv:
                    last_service_t = now
                    if sine_mode is not None:
                        # 正弦：只清缓冲，不刷新当前位置
                        self._ctrl.clear_rx_buffer()
                    else:
                        status = self._ctrl.drain_read_buffer()
                        if status is not None:
                            with self._lock:
                                self._last_status = status
                            self._push_record(status)
            except Exception as exc:
                self.last_error = exc
                break

            next_t += interval
            now = perf_counter()
            remaining = next_t - now
            if remaining < -0.005:
                # 落后超过 5 ms（系统调度卡顿）：重定节拍，避免补发一串帧破坏波形
                next_t = now
            elif remaining > 0.002:
                sleep(remaining - 0.001)
                _spin_until(next_t)
            elif remaining > 0:
                _spin_until(next_t)

        try:
            self._ctrl.clear_rx_buffer()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# 帧调试工具（可选）
# ---------------------------------------------------------------------------

def frame_to_hex(frame: bytes) -> str:
    """将帧字节串格式化为易读十六进制字符串，如 '5A 4A 01 ...'。"""
    return ' '.join(f'{b:02X}' for b in frame)


def preview_tx_frame(cmd: int, x_phys: int, y_phys: int) -> str:
    """预览（不发送）指令帧内容，用于调试。"""
    return frame_to_hex(_build_tx_frame(cmd, x_phys, y_phys))
