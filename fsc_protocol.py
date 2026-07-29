"""
芯明天快反控制盒串口通信协议封装模块
CoreMorrow Fast-Steering Controller (FSC) Serial Protocol

串口参数: 115200 baud, 8N1
数据包格式: [0xAA][地址][包长][B3][B4=0x00][数据段...][XOR校验]
"""

import struct
from time import sleep
from typing import Optional
import serial


# ---------------------------------------------------------------------------
# 指令码常量
# ---------------------------------------------------------------------------

class CMD:
    SEND_VOLTAGE          = 0x00   # 单路电压
    SEND_DISPLACEMENT     = 0x01   # 单路位移
    READ_VOLTAGE          = 0x05   # 读单路电压
    READ_DISPLACEMENT     = 0x06   # 读单路位移
    SEND_WAVE_VOLTAGE     = 0x0F   # 发单路并发电压波形
    SEND_WAVE_DISPLACEMENT = 0x10  # 发单路并发位移波形
    STOP_WAVE             = 0x11   # 停单路并发波形
    SET_LOOP              = 0x12   # 开闭环设置
    READ_LOOP             = 0x13   # 读开闭环
    SET_SIGNAL_MODE       = 0x14   # 模拟/数字切换
    READ_SIGNAL_MODE      = 0x15   # 读模拟/数字


# ---------------------------------------------------------------------------
# 控制标志常量
# ---------------------------------------------------------------------------

class Flag:
    OPEN_LOOP   = 0x4F   # 'O' 开环
    CLOSED_LOOP = 0x43   # 'C' 闭环
    ANALOG      = 0x41   # 'A' 模拟
    DIGITAL     = 0x44   # 'D' 数字


class WaveType:
    SINE     = 0x5A   # 'Z' 正弦波
    SQUARE   = 0x46   # 'F' 方波
    TRIANGLE = 0x53   # 'S' 三角波
    SAWTOOTH = 0x4A   # 'J' 锯齿波


# ---------------------------------------------------------------------------
# 数据编解码（协议自定义浮点，非 IEEE 754）
# ---------------------------------------------------------------------------

def float_to_4bytes(value: float) -> list[int]:
    """
    将浮点数编码为协议自定义的 4 字节格式。
    格式: kk[0] 最高位为符号位(1=负), 高7位+kk[1] 为整数部分,
          kk[2],kk[3] 为小数部分 (小数 * 10000, 精度 0.0001)。
    """
    is_neg = value < 0
    v = abs(value)
    integer_part = int(v)
    frac_part = int((v - integer_part) * 10000 + 0.5)  # 四舍五入减少误差

    kk = [
        integer_part // 256,
        integer_part % 256,
        frac_part // 256,
        frac_part % 256,
    ]
    if is_neg:
        kk[0] |= 0x80
    return kk


def bytes4_to_float(kk: bytes | list[int]) -> float:
    """
    将协议自定义 4 字节格式解码为浮点数。
    """
    is_neg = kk[0] & 0x80
    integer_part = ((kk[0] & 0x7F) << 8) | kk[1]
    frac_part = (kk[2] << 8 | kk[3]) * 0.0001
    value = integer_part + frac_part
    return -value if is_neg else value


# ---------------------------------------------------------------------------
# 帧构造工具
# ---------------------------------------------------------------------------

def _calc_xor(data: list[int]) -> int:
    """计算异或校验位（对 data 中所有字节逐一 XOR）。"""
    result = 0
    for b in data:
        result ^= b
    return result


def _build_frame(address: int, cmd: int, payload: list[int]) -> bytes:
    """
    构造完整数据帧。
    payload 为 B5 起的数据段（含通道号等），不含校验位。
    包长 = 1(0xAA) + 1(addr) + 1(len) + 1(B3) + 1(B4) + len(payload) + 1(xor)
    """
    total_len = 6 + len(payload)
    frame = [0xAA, address, total_len, cmd, 0x00] + payload
    frame.append(_calc_xor(frame))
    return bytes(frame)


def _validate_response(data: bytes, expected_len: int, cmd_echo: int) -> bool:
    """校验下位机回复帧：起始位、长度、指令码回显、XOR。"""
    if len(data) < expected_len:
        return False
    if data[0] != 0xAA:
        return False
    if data[2] != expected_len:
        return False
    if data[3] != cmd_echo:
        return False
    xor = _calc_xor(list(data[:expected_len - 1]))
    return xor == data[expected_len - 1]


# ---------------------------------------------------------------------------
# FSCController 主控类
# ---------------------------------------------------------------------------

class FSCController:
    """
    芯明天快反控制盒串口控制器。

    用法示例::

        with FSCController(port='COM5', address=1) as fsc:
            fsc.send_voltage(channel=0, voltage=10.5)
            v = fsc.read_voltage(channel=0)
    """

    def __init__(
        self,
        port: str,
        address: int = 1,
        baudrate: int = 115200,
        read_timeout: float = 0.1,
    ):
        self._port = port
        self._address = address
        self._baudrate = baudrate
        self._timeout = read_timeout
        self._serial: Optional[serial.Serial] = None

    # ------------------------------------------------------------------
    # 串口生命周期
    # ------------------------------------------------------------------

    def open(self) -> None:
        """打开串口。"""
        self._serial = serial.Serial(
            port=self._port,
            baudrate=self._baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self._timeout,
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
    # 内部发送/接收
    # ------------------------------------------------------------------

    def _send(self, frame: bytes) -> None:
        assert self._serial and self._serial.is_open, "串口未打开"
        self._serial.write(frame)

    def _recv(self, length: int) -> bytes:
        assert self._serial and self._serial.is_open, "串口未打开"
        return self._serial.read(length)

    def _send_recv(self, frame: bytes, recv_len: int, delay: float = 0.05) -> bytes:
        self._send(frame)
        sleep(delay)
        return self._recv(recv_len)

    # ------------------------------------------------------------------
    # 单路电压 / 位移控制
    # ------------------------------------------------------------------

    def send_voltage(self, channel: int, voltage: float) -> None:
        """发送单路电压指令（开环）。单位：V。"""
        payload = [channel] + float_to_4bytes(voltage)
        frame = _build_frame(self._address, CMD.SEND_VOLTAGE, payload)
        self._send(frame)

    def send_displacement(self, channel: int, displacement: float) -> None:
        """发送单路位移指令（闭环）。单位：µm 或 mrad（取决于下位机配置）。"""
        payload = [channel] + float_to_4bytes(displacement)
        frame = _build_frame(self._address, CMD.SEND_DISPLACEMENT, payload)
        self._send(frame)

    # ------------------------------------------------------------------
    # 读单路电压 / 位移
    # ------------------------------------------------------------------

    def read_voltage(self, channel: int) -> float:
        """读取单路当前电压。返回 float，失败返回 None。"""
        frame = _build_frame(self._address, CMD.READ_VOLTAGE, [channel])
        data = self._send_recv(frame, recv_len=11)
        if not _validate_response(data, 11, CMD.READ_VOLTAGE):
            raise FSCError(f"读取电压失败，通道 {channel}，原始数据: {data.hex()}")
        return bytes4_to_float(data[6:10])

    def read_displacement(self, channel: int) -> float:
        """读取单路当前位移。返回 float，失败抛出 FSCError。"""
        frame = _build_frame(self._address, CMD.READ_DISPLACEMENT, [channel])
        data = self._send_recv(frame, recv_len=11)
        if not _validate_response(data, 11, CMD.READ_DISPLACEMENT):
            raise FSCError(f"读取位移失败，通道 {channel}，原始数据: {data.hex()}")
        return bytes4_to_float(data[6:10])

    # ------------------------------------------------------------------
    # 并发波形控制
    # ------------------------------------------------------------------

    def send_wave_voltage(
        self,
        channel: int,
        wave_type: int,
        peak_peak: float,
        frequency: float,
        offset: float,
    ) -> None:
        """
        发单路并发电压波形。

        :param channel:   通道号
        :param wave_type: 波形类型，使用 WaveType.SINE / SQUARE / TRIANGLE / SAWTOOTH
        :param peak_peak: 峰峰值（V）
        :param frequency: 频率（Hz）
        :param offset:    偏置电压（V）
        """
        payload = (
            [channel, wave_type]
            + float_to_4bytes(peak_peak)
            + float_to_4bytes(frequency)
            + float_to_4bytes(offset)
        )
        frame = _build_frame(self._address, CMD.SEND_WAVE_VOLTAGE, payload)
        self._send(frame)

    def send_wave_displacement(
        self,
        channel: int,
        wave_type: int,
        peak_peak: float,
        frequency: float,
        offset: float,
    ) -> None:
        """
        发单路并发位移波形。

        :param channel:   通道号
        :param wave_type: 波形类型，使用 WaveType.SINE / SQUARE / TRIANGLE / SAWTOOTH
        :param peak_peak: 峰峰值（µm / mrad）
        :param frequency: 频率（Hz）
        :param offset:    偏置（µm / mrad）
        """
        payload = (
            [channel, wave_type]
            + float_to_4bytes(peak_peak)
            + float_to_4bytes(frequency)
            + float_to_4bytes(offset)
        )
        frame = _build_frame(self._address, CMD.SEND_WAVE_DISPLACEMENT, payload)
        self._send(frame)

    def stop_wave(self, channel: int) -> None:
        """停止单路并发波形。"""
        frame = _build_frame(self._address, CMD.STOP_WAVE, [channel])
        self._send(frame)

    # ------------------------------------------------------------------
    # 开闭环控制
    # ------------------------------------------------------------------

    def set_loop_mode(self, channel: int, closed: bool) -> None:
        """
        设置开/闭环模式。

        :param closed: True = 闭环，False = 开环
        """
        flag = Flag.CLOSED_LOOP if closed else Flag.OPEN_LOOP
        payload = [channel, flag]
        frame = _build_frame(self._address, CMD.SET_LOOP, payload)
        self._send(frame)

    def get_loop_mode(self, channel: int) -> bool:
        """
        读取当前开/闭环状态。

        :return: True = 闭环，False = 开环
        """
        frame = _build_frame(self._address, CMD.READ_LOOP, [channel])
        data = self._send_recv(frame, recv_len=8)
        if not _validate_response(data, 8, CMD.READ_LOOP):
            raise FSCError(f"读取开闭环失败，通道 {channel}，原始数据: {data.hex()}")
        return data[6] == Flag.CLOSED_LOOP

    # ------------------------------------------------------------------
    # 模拟 / 数字模式切换
    # ------------------------------------------------------------------

    def set_signal_mode(self, channel: int, analog: bool) -> None:
        """
        切换模拟/数字信号模式。

        :param analog: True = 模拟，False = 数字
        """
        flag = Flag.ANALOG if analog else Flag.DIGITAL
        payload = [channel, flag]
        frame = _build_frame(self._address, CMD.SET_SIGNAL_MODE, payload)
        self._send(frame)

    def get_signal_mode(self, channel: int) -> bool:
        """
        读取当前模拟/数字状态。

        :return: True = 模拟，False = 数字
        """
        frame = _build_frame(self._address, CMD.READ_SIGNAL_MODE, [channel])
        data = self._send_recv(frame, recv_len=8)
        if not _validate_response(data, 8, CMD.READ_SIGNAL_MODE):
            raise FSCError(f"读取信号模式失败，通道 {channel}，原始数据: {data.hex()}")
        return data[6] == Flag.ANALOG


# ---------------------------------------------------------------------------
# 自定义异常
# ---------------------------------------------------------------------------

class FSCError(Exception):
    """快反控制盒通信异常。"""


# ---------------------------------------------------------------------------
# 帧调试工具（可选）
# ---------------------------------------------------------------------------

def frame_to_hex(frame: bytes) -> str:
    """将帧字节串格式化为易读的十六进制字符串，如 'AA 01 0B 00 00 ...'。"""
    return ' '.join(f'{b:02X}' for b in frame)


def preview_frame(address: int, cmd: int, payload: list[int]) -> str:
    """预览（不发送）某条指令的完整帧内容，用于调试。"""
    frame = _build_frame(address, cmd, payload)
    return frame_to_hex(frame)
