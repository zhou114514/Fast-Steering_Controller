"""
快反控制盒上位机主窗口

支持三种协议（连接栏“协议”下拉框切换，也可改 config.ini [Protocol] type 字段）：
  xinmingtian — 芯明天 FSC，波形由硬件生成，支持数字/模拟切换
  dianhui     — 巅慧 FSM720，闭环位置控制 + 硬件正弦扫描（XY 同时），
                模拟位置序列按 500 Hz 下发（每点 5 帧连发）；具有轴交换透明处理、8 灯状态面板
                （协议文件 dianghui_fsm720_protocol.py）
  dianhui230  — 巅慧 FSM230-L30，闭环位置控制 + 上位机 1 kHz 软件正弦（正弦期间不刷新位置），
                模拟位置序列与 FSM720 类似（协议文件 dianghui_fsm230_protocol.py）
"""

import configparser
import csv
import os
import sys
from typing import Optional, Union

import serial
import serial.tools.list_ports
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLCDNumber,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QSpinBox,
    QStatusBar,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from dianghui_fsm720_protocol import (
    DEFAULT_LIMIT as DH_DEFAULT_LIMIT,
    SETPOINT_LIMIT as DH_SETPOINT_LIMIT,
    SINE_FREQ_MAX as DH_SINE_FREQ_MAX,
    ANALOG_INTERVAL_MS as DH_ANALOG_INTERVAL_MS,
    ANALOG_BURST_N as DH_ANALOG_BURST_N,
    DianghuiController,
    DianghuiError,
    DianghuiStatus,
    DianghuiWaveManager,
)
from dianghui_fsm230_protocol import (
    SETPOINT_LIMIT as DH230_SETPOINT_LIMIT,
    SINE_FREQ_MAX as DH230_SINE_FREQ_MAX,
    ANALOG_INTERVAL_MS as DH230_ANALOG_INTERVAL_MS,
    ANALOG_BURST_N as DH230_ANALOG_BURST_N,
    DianghuiFsm230Controller,
    DianghuiFsm230Error,
    DianghuiFsm230Status,
    DianghuiFsm230WaveManager,
)
from fsc_protocol import FSCController, FSCError, WaveType
from tcp_server import FSCTcpServer

# ---------------------------------------------------------------------------
# 常量
# ---------------------------------------------------------------------------

if getattr(sys, "frozen", False):
    ROOT_PATH = os.path.dirname(sys.executable)
else:
    ROOT_PATH = os.path.dirname(__file__)

CONFIG_PATH = os.path.join(ROOT_PATH, "config.ini")

PROTOCOL_XINMINGTIAN = "xinmingtian"  # 芯明天 FSC
PROTOCOL_DIANHUI     = "dianhui"      # 巅慧 FSM720（配置值保持 dianhui 以兼容旧 config.ini）
PROTOCOL_DIANHUI_230 = "dianhui230"   # 巅慧 FSM230-L30

# 巅慧系列（两款共用大部分界面逻辑）
DH_PROTOCOLS = (PROTOCOL_DIANHUI, PROTOCOL_DIANHUI_230)

# (配置值, 界面显示名)，顺序即下拉框顺序
PROTOCOL_CHOICES = [
    (PROTOCOL_XINMINGTIAN, "芯明天 FSC"),
    (PROTOCOL_DIANHUI,     "巅慧 FSM720"),
    (PROTOCOL_DIANHUI_230, "巅慧 FSM230"),
]
PROTOCOL_LABELS = dict(PROTOCOL_CHOICES)

WAVE_TYPES = {
    "正弦波":  WaveType.SINE,
    "方波":    WaveType.SQUARE,
    "三角波":  WaveType.TRIANGLE,
    "锯齿波":  WaveType.SAWTOOTH,
}

COMMON_FREQS = [20, 80, 120, 140, 160]
BAUD_RATES   = ["5000000", "3000000", "2000000", "1000000",
                "921600", "460800", "115200", "57600", "38400", "19200", "9600"]

# 各协议的默认波特率（首次使用时无 config 记录时自动选中）
_DEFAULT_BAUD = {
    PROTOCOL_DIANHUI:     "921600",
    PROTOCOL_DIANHUI_230: "921600",   # 文档：921600 / 5Mbps
    PROTOCOL_XINMINGTIAN: "115200",
}

# 所有需捕获的通信异常
_COMM_ERRORS = (
    FSCError, DianghuiError, DianghuiFsm230Error,
    AssertionError, serial.SerialException, OSError,
)


# ---------------------------------------------------------------------------
# 状态指示灯（巅慧专用）
# ---------------------------------------------------------------------------

class StatusLed(QWidget):
    """圆形状态指示灯 + 短文字标签。"""

    _TMPL = "QLabel {{ color: {color}; font-size: 22px; }}"

    def __init__(self, label: str, parent=None):
        super().__init__(parent)
        v = QVBoxLayout(self)
        v.setContentsMargins(2, 2, 2, 2)
        v.setSpacing(1)
        v.setAlignment(Qt.AlignCenter)

        self._circle = QLabel("●")
        self._circle.setAlignment(Qt.AlignCenter)
        v.addWidget(self._circle)

        lbl = QLabel(label)
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setWordWrap(True)
        lbl.setFixedWidth(72)
        lbl.setStyleSheet("font-size: 11px;")
        v.addWidget(lbl)

        self.set_unknown()

    def _set_color(self, color: str):
        self._circle.setStyleSheet(self._TMPL.format(color=color))

    def set_ok(self):      self._set_color("#00cc44")
    def set_error(self):   self._set_color("#cc2222")
    def set_unknown(self): self._set_color("#666666")


class DianghuiStatusPanel(QGroupBox):
    """巅慧设备状态面板：一行 8 个指示灯。"""

    # (attr_key, 显示标签, error_when_true)
    _LED_DEFS = [
        ("comm",   "通讯\n校验",  True),
        ("selfck", "自检",        True),
        ("x_en",   "X轴\n使能",   False),  # False → 灯亮 = 使能 = 正常
        ("y_en",   "Y轴\n使能",   False),
        ("x_disp", "X位移\n超限", True),
        ("y_disp", "Y位移\n超限", True),
        ("x_cmd",  "X指令\n超限", True),
        ("y_cmd",  "Y指令\n超限", True),
    ]

    def __init__(self, parent=None):
        super().__init__("设备状态", parent)
        h = QHBoxLayout(self)
        h.setContentsMargins(8, 4, 8, 4)
        h.setSpacing(6)

        self._leds: dict[str, StatusLed] = {}
        self._err_on_true: dict[str, bool] = {}
        for key, label, eot in self._LED_DEFS:
            led = StatusLed(label)
            h.addWidget(led)
            self._leds[key] = led
            self._err_on_true[key] = eot
        h.addStretch()

    def set_unknown(self):
        for led in self._leds.values():
            led.set_unknown()

    def update_status(self, st: DianghuiStatus):
        def _put(key: str, val: bool):
            is_err = val if self._err_on_true[key] else not val
            self._leds[key].set_error() if is_err else self._leds[key].set_ok()

        _put("comm",   st.comm_error)
        _put("selfck", st.selfcheck_error)
        _put("x_en",   st.x_enabled)
        _put("y_en",   st.y_enabled)
        _put("x_disp", st.x_disp_over)
        _put("y_disp", st.y_disp_over)
        _put("x_cmd",  st.x_cmd_over)
        _put("y_cmd",  st.y_cmd_over)


# ---------------------------------------------------------------------------
# 七段码显示控件
# ---------------------------------------------------------------------------

class LcdDisplay(QWidget):
    """黑底白字七段码显示，支持正负号和小数。"""

    def __init__(self, label_text: str, unit: str = "mrad", parent=None):
        super().__init__(parent)
        v = QVBoxLayout(self)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(4)

        title = QLabel(label_text)
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont("Microsoft YaHei", 12, QFont.Bold))
        v.addWidget(title)

        self._lcd = QLCDNumber(self)
        self._lcd.setDigitCount(9)
        self._lcd.setSegmentStyle(QLCDNumber.Flat)
        self._lcd.setSmallDecimalPoint(False)
        self._lcd.setMinimumHeight(70)
        self._lcd.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self._lcd.setStyleSheet(
            "QLCDNumber {"
            "  background-color: #000000; color: #ffffff;"
            "  border: 2px solid #444444; border-radius: 4px;"
            "}"
        )
        self._lcd.display("-.--")
        v.addWidget(self._lcd)

        self._unit_lbl = QLabel(unit)
        self._unit_lbl.setAlignment(Qt.AlignCenter)
        self._unit_lbl.setStyleSheet("color: #888888; font-size: 13px;")
        v.addWidget(self._unit_lbl)

    def set_value(self, value: float) -> None:
        self._lcd.display(f"{value:.1f}" if abs(value) >= 1000 else f"{value:.3f}")

    def set_int_value(self, value: int) -> None:
        self._lcd.display(str(value))

    def set_error(self) -> None:
        self._lcd.display("Err")


# ---------------------------------------------------------------------------
# 单通道波形控制区
# ---------------------------------------------------------------------------

class WaveChannelWidget(QGroupBox):
    """单通道波形参数设置与发送控件。

    巅慧模式下：
      - 峰峰值使用整数码值（0~32767），默认 0，待测试后由用户设置
      - 偏置同为整数码值
      - 波形类型固定为正弦（控件隐藏）
    """

    def __init__(self, channel: int,
                 protocol_type: str = PROTOCOL_XINMINGTIAN,
                 parent=None):
        super().__init__(f"通道 {channel} 波形控制", parent)
        self._channel  = channel
        self._protocol = protocol_type
        self._current_displacement: float = 0.0
        self._last_sent: float = 0.0
        self._send_wave_cb = None
        self._stop_wave_cb = None
        self._build_ui()

    def _build_ui(self):
        is_dh = (self._protocol == PROTOCOL_DIANHUI)
        outer = QVBoxLayout(self)
        outer.setSpacing(8)
        outer.setContentsMargins(10, 14, 10, 10)

        grid = QGridLayout()
        grid.setSpacing(6)
        grid.setColumnMinimumWidth(1, 90)
        row = 0

        # --- 波形类型 ---
        grid.addWidget(QLabel("波形:"), row, 0)
        self._wave_combo = QComboBox()
        self._wave_combo.addItems(list(WAVE_TYPES.keys()))
        self._wave_combo.setMinimumWidth(90)
        grid.addWidget(self._wave_combo, row, 1)
        row += 1

        # --- 峰峰值 ---
        grid.addWidget(QLabel("峰峰值:"), row, 0)
        self._pp_spin = QDoubleSpinBox()
        if is_dh:
            self._pp_spin.setRange(0, 32767)
            self._pp_spin.setDecimals(0)
            self._pp_spin.setSingleStep(10)
            self._pp_spin.setValue(0)
            self._pp_spin.setToolTip("巅慧峰峰值（int16 原始码值），请根据测试结果设置合适值")
            pp_unit = QLabel("counts")
        else:
            self._pp_spin.setRange(0.01, 200.0)
            self._pp_spin.setDecimals(3)
            self._pp_spin.setSingleStep(0.01)
            self._pp_spin.setValue(0.65)
            pp_unit = QLabel("mrad")
        self._pp_spin.setMinimumWidth(90)
        grid.addWidget(self._pp_spin, row, 1)
        grid.addWidget(pp_unit, row, 2)
        row += 1

        # --- 频率 ---
        grid.addWidget(QLabel("频率(Hz):"), row, 0)
        self._freq_spin = QDoubleSpinBox()
        self._freq_spin.setRange(0.1, 10000.0)
        self._freq_spin.setDecimals(1)
        self._freq_spin.setSingleStep(1.0)
        self._freq_spin.setValue(20.0)
        self._freq_spin.setMinimumWidth(90)
        grid.addWidget(self._freq_spin, row, 1)

        freq_row = QHBoxLayout()
        freq_row.setSpacing(5)
        for f in COMMON_FREQS:
            btn = QPushButton(str(f))
            btn.setFixedWidth(42)
            btn.clicked.connect(lambda _checked, v=f: self._freq_spin.setValue(v))
            freq_row.addWidget(btn)
        freq_row.addStretch()
        grid.addLayout(freq_row, row, 2)
        row += 1

        # --- 偏置 ---
        grid.addWidget(QLabel("偏置:"), row, 0)
        self._offset_spin = QDoubleSpinBox()
        if is_dh:
            self._offset_spin.setRange(-DH_SETPOINT_LIMIT, DH_SETPOINT_LIMIT)
            self._offset_spin.setDecimals(0)
            self._offset_spin.setSingleStep(1)
            off_unit = QLabel("counts")
        else:
            self._offset_spin.setRange(-200.0, 200.0)
            self._offset_spin.setDecimals(3)
            self._offset_spin.setSingleStep(0.1)
            off_unit = QLabel("mrad")
        self._offset_spin.setValue(0.0)
        self._offset_spin.setMinimumWidth(90)
        grid.addWidget(self._offset_spin, row, 1)
        grid.addWidget(off_unit, row, 2)

        fill_btn = QPushButton("填充当前位移")
        fill_btn.setToolTip("将当前通道的位移读数填入偏置")
        fill_btn.clicked.connect(self._fill_offset)
        grid.addWidget(fill_btn, row, 3)
        row += 1

        outer.addLayout(grid)
        outer.addStretch()

        # --- 操作按钮 ---
        btn_row = QHBoxLayout()
        btn_row.addStretch()

        send_btn = QPushButton("发送波形")
        send_btn.setMinimumWidth(100)
        send_btn.setStyleSheet(
            "QPushButton{background-color:#1a6b2a;color:white;font-weight:bold;padding:5px 14px;}"
            "QPushButton:hover{background-color:#238c38;}"
            "QPushButton:disabled{background-color:#555;color:#999;}"
        )
        send_btn.clicked.connect(self._on_send_wave)
        btn_row.addWidget(send_btn)

        stop_btn = QPushButton("停止波形")
        stop_btn.setMinimumWidth(100)
        stop_btn.setStyleSheet(
            "QPushButton{background-color:#8b1a1a;color:white;font-weight:bold;padding:5px 14px;}"
            "QPushButton:hover{background-color:#b52424;}"
            "QPushButton:disabled{background-color:#555;color:#999;}"
        )
        stop_btn.clicked.connect(self._on_stop_wave)
        btn_row.addWidget(stop_btn)

        outer.addLayout(btn_row)

    # --- 外部接口 ---

    def set_callbacks(self, send_cb, stop_cb):
        self._send_wave_cb = send_cb
        self._stop_wave_cb = stop_cb

    # --- 参数读取（供同步按钮使用） ---
    def get_wave_type(self):
        """返回当前选中的波形类型。
        芯明天协议返回 WaveType 整数常量。
        """
        return WAVE_TYPES.get(self._wave_combo.currentText(), WaveType.SINE)

    def get_peak_peak(self) -> float:
        return self._pp_spin.value()

    def get_frequency(self) -> float:
        return self._freq_spin.value()

    def get_offset(self) -> float:
        return self._offset_spin.value()

    def update_displacement(self, value: float):
        self._current_displacement = value

    def update_last_sent(self, value: float):
        self._last_sent = value

    def _fill_offset(self):
        self._offset_spin.setValue(self._last_sent)

    def _on_send_wave(self):
        if not self._send_wave_cb:
            return
        wave_type = self.get_wave_type()
        self._send_wave_cb(
            channel   = self._channel,
            wave_type = wave_type,
            peak_peak = self._pp_spin.value(),
            frequency = self._freq_spin.value(),
            offset    = self._offset_spin.value(),
        )

    def _on_stop_wave(self):
        if self._stop_wave_cb:
            self._stop_wave_cb(channel=self._channel, displacement=self._last_sent)


# ---------------------------------------------------------------------------
# 巅慧：XY 同时硬件正弦
# ---------------------------------------------------------------------------

class DhXySineWidget(QGroupBox):
    """
    巅慧 XY 轴同时正弦。

    FSM720：硬件正弦（指令 0x06）。
    FSM230：上位机 1 kHz 软件正弦（逐点下发闭环设定值，正弦期间不刷新位置）。

    界面峰峰值为习惯用法，下发时除以 2 得到幅值（峰值）。
    不支持偏置；停止后由主窗口发闭环位置回到设定值。
    """

    def __init__(self, setpoint_limit: int = DH_SETPOINT_LIMIT,
                 freq_max: int = DH_SINE_FREQ_MAX,
                 title: str = "XY 同时正弦扫描",
                 freq_tip: str = "1 Hz/LSB，范围 0~5000",
                 hint: str = "正弦指令不支持偏置；停止后自动回到位移控制区的设定位置。",
                 parent=None):
        super().__init__(title, parent)
        self._sp_limit = setpoint_limit
        self._freq_max = freq_max
        self._freq_tip = freq_tip
        self._hint     = hint
        self._send_cb = None
        self._stop_cb = None
        self._build_ui()

    def _build_ui(self):
        outer = QVBoxLayout(self)
        outer.setSpacing(8)
        outer.setContentsMargins(10, 14, 10, 10)

        grid = QGridLayout()
        grid.setSpacing(6)
        grid.setColumnMinimumWidth(1, 90)
        row = 0

        grid.addWidget(QLabel("峰峰值:"), row, 0)
        self._pp_spin = QSpinBox()
        self._pp_spin.setRange(0, 2 * self._sp_limit)
        self._pp_spin.setSingleStep(10)
        self._pp_spin.setValue(0)
        self._pp_spin.setMinimumWidth(90)
        self._pp_spin.setToolTip(
            "界面使用峰峰值；下发幅值（峰值）= 峰峰值 / 2，与位置设定值同单位"
        )
        grid.addWidget(self._pp_spin, row, 1)
        grid.addWidget(QLabel("counts"), row, 2)
        row += 1

        grid.addWidget(QLabel("频率(Hz):"), row, 0)
        self._freq_spin = QSpinBox()
        self._freq_spin.setRange(0, self._freq_max)
        self._freq_spin.setSingleStep(1)
        self._freq_spin.setValue(20)
        self._freq_spin.setMinimumWidth(90)
        self._freq_spin.setToolTip(self._freq_tip)
        grid.addWidget(self._freq_spin, row, 1)

        freq_row = QHBoxLayout()
        freq_row.setSpacing(5)
        for f in COMMON_FREQS:
            btn = QPushButton(str(f))
            btn.setFixedWidth(42)
            btn.clicked.connect(lambda _checked, v=f: self._freq_spin.setValue(v))
            freq_row.addWidget(btn)
        freq_row.addStretch()
        grid.addLayout(freq_row, row, 2)
        row += 1

        hint = QLabel(self._hint)
        hint.setStyleSheet("color: #888888; font-size: 12px;")
        hint.setWordWrap(True)
        grid.addWidget(hint, row, 0, 1, 3)

        outer.addLayout(grid)

        btn_row = QHBoxLayout()
        btn_row.addStretch()

        send_btn = QPushButton("发送正弦")
        send_btn.setMinimumWidth(100)
        send_btn.setStyleSheet(
            "QPushButton{background-color:#1a6b2a;color:white;font-weight:bold;padding:5px 14px;}"
            "QPushButton:hover{background-color:#238c38;}"
            "QPushButton:disabled{background-color:#555;color:#999;}"
        )
        send_btn.clicked.connect(self._on_send)
        btn_row.addWidget(send_btn)

        stop_btn = QPushButton("停止正弦")
        stop_btn.setMinimumWidth(100)
        stop_btn.setStyleSheet(
            "QPushButton{background-color:#8b1a1a;color:white;font-weight:bold;padding:5px 14px;}"
            "QPushButton:hover{background-color:#b52424;}"
            "QPushButton:disabled{background-color:#555;color:#999;}"
        )
        stop_btn.clicked.connect(self._on_stop)
        btn_row.addWidget(stop_btn)

        outer.addLayout(btn_row)

    def set_callbacks(self, send_cb, stop_cb):
        self._send_cb = send_cb
        self._stop_cb = stop_cb

    def get_peak_peak(self) -> int:
        return int(self._pp_spin.value())

    def get_frequency(self) -> int:
        return int(self._freq_spin.value())

    def get_amplitude(self) -> int:
        """协议幅值 = 峰峰值 / 2（峰值）。"""
        return int(round(self._pp_spin.value() / 2.0))

    def _on_send(self):
        if self._send_cb:
            self._send_cb()

    def _on_stop(self):
        if self._stop_cb:
            self._stop_cb()


# ---------------------------------------------------------------------------
# 主窗口
# ---------------------------------------------------------------------------

class MainWindow(QMainWindow):
    _sig_mode_changed = pyqtSignal(bool)       # is_analog (FSC 专用)
    _sig_loop_changed = pyqtSignal(bool)       # is_closed_loop (FSC 专用)
    _sig_disp_sent    = pyqtSignal(int, float) # channel, displacement
    _sig_status_msg   = pyqtSignal(str)

    def __init__(self):
        super().__init__()

        # --- 配置 ---
        self._config = configparser.ConfigParser()
        self._load_config()
        self._protocol: str = (
            self._config.get("Protocol", "type", fallback=PROTOCOL_XINMINGTIAN)
            .strip().lower()
        )
        if self._protocol not in PROTOCOL_LABELS:
            self._protocol = PROTOCOL_XINMINGTIAN

        # --- 芯明天状态 ---
        self._fsc: Optional[FSCController] = None
        self._is_analog      = False
        self._is_closed_loop = True

        # --- 巅慧状态（FSM720 / FSM230 共用这两个引用，接口一致）---
        self._dh: Optional[Union[DianghuiController, DianghuiFsm230Controller]] = None
        self._dh_wave_mgr: Optional[
            Union[DianghuiWaveManager, DianghuiFsm230WaveManager]
        ] = None
        self._analog_pos_file: Optional[str]       = None
        self._analog_pos_x:    Optional[list[int]] = None
        self._analog_pos_y:    Optional[list[int]] = None

        # --- 公共状态 ---
        self._displacement      = [0.0, 0.0]
        self._poll_error_count  = 0

        # --- 协议特定 UI 控件（_build_ui 之前先声明为 None） ---
        self._mode_btn:        Optional[QPushButton]          = None
        self._loop_btn:        Optional[QPushButton]          = None
        self._addr_spin:       Optional[QSpinBox]             = None
        self._dh_status_panel: Optional[DianghuiStatusPanel]  = None
        self._analog_file_lbl:   Optional[QLabel]      = None
        self._analog_play_btn:   Optional[QPushButton] = None
        self._analog_stop_btn:   Optional[QPushButton] = None
        self._analog_rate_btn:   Optional[QPushButton] = None
        self._chk_analog_burst:  Optional[QCheckBox]   = None
        self._chk_analog_interp: Optional[QCheckBox]   = None
        # 采样率内部状态（ms），在 _build_analog_pos_bar 里从 config 初始化
        self._dh_analog_interval_ms: int = DH_ANALOG_INTERVAL_MS
        # 原始点列（文件加载后保留，切换采样率时重新抽点）
        self._analog_pos_raw_x:       Optional[list[int]] = None
        self._analog_pos_raw_y:       Optional[list[int]] = None
        self._analog_file_interval_ms: int                = 1
        self._dh_sine_widget:  Optional[DhXySineWidget]       = None
        self._dh_sine_active:  bool                           = False
        # 轴方向反转（巅慧专用）
        self._chk_invert_x:    Optional[QCheckBox]          = None
        self._chk_invert_y:    Optional[QCheckBox]          = None
        # 数据记录（巅慧专用）
        self._rec_btn:         Optional[QPushButton]          = None
        self._rec_count_lbl:   Optional[QLabel]               = None
        self._rec_save_btn:    Optional[QPushButton]          = None
        self._cached_rec_data: Optional[tuple]                = None  # 断连后缓存

        # --- TCP 服务 ---
        self._tcp_server = FSCTcpServer(port=self._tcp_port())
        self._tcp_server.set_log_callback(self._append_tcp_log)
        self._register_tcp_handlers()

        # --- UI ---
        self._build_ui()
        self._refresh_ports()

        # --- 信号 → 主线程槽 ---
        self._sig_mode_changed.connect(self._apply_mode_from_remote)
        self._sig_loop_changed.connect(self._apply_loop_from_remote)
        self._sig_disp_sent.connect(self._apply_disp_from_remote)
        self._sig_status_msg.connect(self._status_bar.showMessage)

        # --- 定时轮询 ---
        self._disp_timer = QTimer(self)
        # 巅慧：100ms（10Hz），波形运行时读缓存无串口开销；
        # 芯明天：1000ms（1Hz），每次需发串口读取指令。
        self._disp_timer.setInterval(self._poll_interval_ms())
        self._disp_timer.timeout.connect(self._poll_displacement)
        self._disp_timer.start()

        if self._config.getboolean("TCP", "auto_start", fallback=True):
            self._start_tcp()

    # ================================================================
    # 配置
    # ================================================================

    def _load_config(self):
        if os.path.exists(CONFIG_PATH):
            self._config.read(CONFIG_PATH, encoding="utf-8")
        for sec in ("Protocol", "Serial", "TCP"):
            if not self._config.has_section(sec):
                self._config.add_section(sec)

    def _save_config(self):
        with open(CONFIG_PATH, "w", encoding="utf-8") as f:
            self._config.write(f)

    def _tcp_port(self) -> int:
        return self._config.getint("TCP", "port", fallback=10014)

    def _tcp_ip(self) -> str:
        return self._config.get("TCP", "ip", fallback="0.0.0.0")

    # ================================================================
    # 协议相关参数（FSM720 / FSM230 差异集中在这里）
    # ================================================================

    @property
    def _is_dh(self) -> bool:
        """当前是否为巅慧系列（FSM720 或 FSM230）。"""
        return self._protocol in DH_PROTOCOLS

    @property
    def _is_dh230(self) -> bool:
        return self._protocol == PROTOCOL_DIANHUI_230

    @property
    def _dh_setpoint_limit(self) -> int:
        return DH230_SETPOINT_LIMIT if self._is_dh230 else DH_SETPOINT_LIMIT

    @property
    def _dh_burst_n(self) -> int:
        return DH230_ANALOG_BURST_N if self._is_dh230 else DH_ANALOG_BURST_N

    @property
    def _dh_default_interval_ms(self) -> int:
        return DH230_ANALOG_INTERVAL_MS if self._is_dh230 else DH_ANALOG_INTERVAL_MS

    @property
    def _dh_unit(self) -> str:
        return "counts (0.1 µrad)" if self._is_dh230 else "counts"

    def _dh_cfg_key(self, name: str) -> str:
        """巅慧配置键：FSM720 保持旧键 dianhui_xxx，FSM230 使用 dianhui230_xxx。"""
        return f"{self._protocol}_{name}"

    def _poll_interval_ms(self) -> int:
        # 巅慧：100ms（10Hz），波形运行时读缓存无串口开销；
        # 芯明天：1000ms（1Hz），每次需发串口读取指令。
        return 100 if self._is_dh else 1000

    # ================================================================
    # UI 构建
    # ================================================================

    def _reset_protocol_widgets(self):
        """重建界面前，丢弃上一个协议界面的控件引用（控件本身随旧 central 一并销毁）。"""
        self._mode_btn = self._loop_btn = self._addr_spin = None
        self._dh_status_panel = None
        self._analog_file_lbl = self._analog_play_btn = None
        self._analog_stop_btn = self._analog_rate_btn = None
        self._chk_analog_burst = self._chk_analog_interp = None
        self._dh_sine_widget = None
        self._chk_invert_x = self._chk_invert_y = None
        self._rec_btn = self._rec_count_lbl = self._rec_save_btn = None
        self._wave_widgets = []

    def _build_ui(self):
        self._reset_protocol_widgets()
        proto_label = PROTOCOL_LABELS[self._protocol]
        self.setWindowTitle(f"快反控制盒上位机  [{proto_label}]")
        self.setMinimumWidth(840)

        central = QWidget()
        self.setCentralWidget(central)
        ml = QVBoxLayout(central)
        ml.setSpacing(8)
        ml.setContentsMargins(10, 10, 10, 10)

        ml.addWidget(self._build_connection_bar())

        if self._is_dh:
            ml.addWidget(self._build_analog_pos_bar())
        else:
            ml.addWidget(self._build_mode_bar())

        ml.addWidget(self._build_displacement_area())

        # 仅 FSM720 的状态帧带 8 个状态位；FSM230 状态帧只有两轴位移，不显示该面板
        if self._protocol == PROTOCOL_DIANHUI:
            self._dh_status_panel = DianghuiStatusPanel()
            ml.addWidget(self._dh_status_panel)

        ml.addWidget(self._build_wave_area())
        ml.addWidget(self._build_tcp_log_area(), stretch=1)

        # 状态栏只创建一次（_sig_status_msg 已绑定到它，切换协议重建界面时不能换掉）
        if not hasattr(self, "_status_bar"):
            self._status_bar = QStatusBar()
            self.setStatusBar(self._status_bar)
            self._status_bar.showMessage("未连接")

    # --- 协议切换 ---
    def _on_protocol_combo_changed(self, index: int):
        new_proto = self._proto_combo.itemData(index)
        if not new_proto or new_proto == self._protocol:
            return
        # 当前槽由即将被销毁的控件触发，延后到事件循环空闲时再重建界面
        QTimer.singleShot(0, lambda p=new_proto: self._switch_protocol(p))

    def _switch_protocol(self, new_proto: str):
        """运行时切换协议：保存配置并重建界面。仅在未连接时允许。"""
        if new_proto == self._protocol or new_proto not in PROTOCOL_LABELS:
            return
        if self._fsc or self._dh:
            self._show_error("请先断开连接再切换协议")
            self._proto_combo.blockSignals(True)
            self._proto_combo.setCurrentIndex(self._proto_combo.findData(self._protocol))
            self._proto_combo.blockSignals(False)
            return

        # 保留 TCP 日志内容
        log_html = self._tcp_log.toHtml() if hasattr(self, "_tcp_log") else ""

        self._protocol = new_proto
        self._config.set("Protocol", "type", new_proto)
        self._save_config()

        self._poll_error_count = 0
        self._disp_timer.setInterval(self._poll_interval_ms())
        self._build_ui()
        self._refresh_ports()
        if log_html:
            self._tcp_log.setHtml(log_html)

        # 已加载的模拟位置文件在新协议下按其采样率配置重新抽点
        if self._analog_pos_raw_x and self._analog_pos_raw_y:
            self._resample_analog_pos()
            self._update_analog_file_lbl()
            if self._analog_play_btn:
                self._analog_play_btn.setEnabled(True)

        self._status_bar.showMessage(f"已切换协议: {PROTOCOL_LABELS[new_proto]}（未连接）")

    # --- 连接栏 ---
    def _build_connection_bar(self) -> QWidget:
        bar = QGroupBox("连接")
        h = QHBoxLayout(bar)
        h.setSpacing(8)

        h.addWidget(QLabel("协议:"))
        self._proto_combo = QComboBox()
        for key, label in PROTOCOL_CHOICES:
            self._proto_combo.addItem(label, key)
        self._proto_combo.setCurrentIndex(self._proto_combo.findData(self._protocol))
        self._proto_combo.setMinimumWidth(110)
        self._proto_combo.setToolTip("切换设备协议（需先断开串口）；选择会写入 config.ini")
        self._proto_combo.setEnabled(not (self._fsc or self._dh))
        self._proto_combo.currentIndexChanged.connect(self._on_protocol_combo_changed)
        h.addWidget(self._proto_combo)

        h.addWidget(QLabel("串口:"))
        self._port_combo = QComboBox()
        self._port_combo.setMinimumWidth(100)
        h.addWidget(self._port_combo)

        refresh_btn = QPushButton("刷新")
        refresh_btn.setFixedWidth(50)
        refresh_btn.clicked.connect(self._refresh_ports)
        h.addWidget(refresh_btn)

        h.addWidget(QLabel("波特率:"))
        self._baud_combo = QComboBox()
        self._baud_combo.addItems(BAUD_RATES)
        # 每个协议独立记忆波特率，切换协议后自动使用对应默认值
        _baud_key     = f"baudrate_{self._protocol}"
        _baud_default = _DEFAULT_BAUD.get(self._protocol, "115200")
        saved_baud    = self._config.get("Serial", _baud_key, fallback=_baud_default)
        if saved_baud in BAUD_RATES:
            self._baud_combo.setCurrentText(saved_baud)
        self._baud_combo.setFixedWidth(90)
        h.addWidget(self._baud_combo)

        # 地址仅芯明天使用
        if self._protocol == PROTOCOL_XINMINGTIAN:
            h.addWidget(QLabel("地址:"))
            self._addr_spin = QSpinBox()
            self._addr_spin.setRange(1, 255)
            self._addr_spin.setValue(self._config.getint("Serial", "address", fallback=1))
            self._addr_spin.setFixedWidth(55)
            h.addWidget(self._addr_spin)

        self._connect_btn = QPushButton("连接")
        self._connect_btn.setFixedWidth(60)
        self._connect_btn.setCheckable(True)
        self._connect_btn.clicked.connect(self._on_connect_toggle)
        h.addWidget(self._connect_btn)

        h.addStretch()
        return bar

    # --- 芯明天: 模式栏 ---
    def _build_mode_bar(self) -> QWidget:
        bar = QWidget()
        h = QHBoxLayout(bar)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(12)
        h.addStretch()

        self._mode_btn = QPushButton("当前模式: 数字  →  切换为模拟")
        self._mode_btn.setFixedHeight(36)
        self._mode_btn.setMinimumWidth(270)
        self._mode_btn.setStyleSheet(self._mode_btn_style(False))
        self._mode_btn.setEnabled(False)
        self._mode_btn.clicked.connect(self._on_mode_toggle)
        h.addWidget(self._mode_btn)

        self._loop_btn = QPushButton("当前状态: 闭环  →  切换为开环")
        self._loop_btn.setFixedHeight(36)
        self._loop_btn.setMinimumWidth(270)
        self._loop_btn.setStyleSheet(self._loop_btn_style(True))
        self._loop_btn.setEnabled(False)
        self._loop_btn.clicked.connect(self._on_loop_toggle)
        h.addWidget(self._loop_btn)

        h.addStretch()
        return bar

    # --- 巅慧: 模拟位置栏 ---
    def _build_analog_pos_bar(self) -> QWidget:
        bar = QGroupBox("模拟位置控制（替代模拟信号输入）")
        vbox = QVBoxLayout(bar)
        vbox.setSpacing(4)
        vbox.setContentsMargins(8, 4, 8, 4)

        # ── 第一行：文件 + 发送/停止 ─────────────────────────────────────
        row1 = QHBoxLayout()
        row1.setSpacing(8)
        row1.addWidget(QLabel("位置文件 (X,Y 两列):"))

        self._analog_file_lbl = QLabel("未选择")
        self._analog_file_lbl.setStyleSheet("color: #888888;")
        row1.addWidget(self._analog_file_lbl, 1)

        load_btn = QPushButton("加载文件...")
        load_btn.clicked.connect(self._on_load_analog_pos)
        row1.addWidget(load_btn)

        self._analog_play_btn = QPushButton("发送模拟信号")
        self._analog_play_btn.setEnabled(False)
        self._analog_play_btn.setStyleSheet(
            "QPushButton{background-color:#1a6b2a;color:white;font-weight:bold;padding:4px 12px;}"
            "QPushButton:hover{background-color:#238c38;}"
            "QPushButton:disabled{background-color:#555;color:#999;}"
        )
        self._analog_play_btn.clicked.connect(self._on_play_analog_pos)
        row1.addWidget(self._analog_play_btn)

        self._analog_stop_btn = QPushButton("停止")
        self._analog_stop_btn.setEnabled(False)
        self._analog_stop_btn.setStyleSheet(
            "QPushButton{background-color:#8b1a1a;color:white;font-weight:bold;padding:4px 12px;}"
            "QPushButton:hover{background-color:#b52424;}"
            "QPushButton:disabled{background-color:#555;color:#999;}"
        )
        self._analog_stop_btn.clicked.connect(self._on_stop_analog_pos)
        row1.addWidget(self._analog_stop_btn)
        vbox.addLayout(row1)

        # ── 第二行：发送选项 ──────────────────────────────────────────────
        row2 = QHBoxLayout()
        row2.setSpacing(10)
        row2.addWidget(QLabel("发送选项:"))

        # 采样率切换按钮
        saved_iv = self._config.getint(
            "Protocol", self._dh_cfg_key("analog_interval_ms"),
            fallback=self._dh_default_interval_ms,
        )
        self._dh_analog_interval_ms = saved_iv if saved_iv in (1, 2) else self._dh_default_interval_ms
        self._analog_rate_btn = QPushButton()
        self._analog_rate_btn.setToolTip(
            "切换每秒发送的位置点数（需重新加载文件才能提升至文件原始精度）"
        )
        self._analog_rate_btn.clicked.connect(self._on_analog_rate_toggle)
        self._update_analog_rate_btn()
        row2.addWidget(self._analog_rate_btn)

        # 连发勾选框
        saved_burst = self._config.getboolean(
            "Protocol", self._dh_cfg_key("analog_burst"), fallback=True
        )
        self._chk_analog_burst = QCheckBox(f"{self._dh_burst_n}帧连发")
        self._chk_analog_burst.setToolTip(
            f"勾选：每个位置点连续发送 {self._dh_burst_n} 帧\n"
            "不勾选：每个位置点单帧发送"
        )
        self._chk_analog_burst.setChecked(saved_burst)
        self._chk_analog_burst.stateChanged.connect(self._on_analog_burst_changed)
        row2.addWidget(self._chk_analog_burst)

        # 插值勾选框
        saved_interp = self._config.getboolean(
            "Protocol", self._dh_cfg_key("analog_interpolate"), fallback=False
        )
        self._chk_analog_interp = QCheckBox("线性插值")
        self._chk_analog_interp.setToolTip(
            "连发时有效：\n"
            f"  勾选 = {self._dh_burst_n}帧在当前点→下一点之间线性插值（每帧值不同）\n"
            f"  不勾 = {self._dh_burst_n}帧重复同一点值"
        )
        self._chk_analog_interp.setChecked(saved_interp)
        self._chk_analog_interp.stateChanged.connect(self._on_analog_interp_changed)
        row2.addWidget(self._chk_analog_interp)

        row2.addStretch()
        vbox.addLayout(row2)

        return bar

    # --- 位移控制区 ---
    def _build_displacement_area(self) -> QWidget:
        unit  = self._dh_unit if self._is_dh else "mrad"
        group = QGroupBox("位移控制")
        h = QHBoxLayout(group)
        h.setSpacing(20)

        for ch in range(2):
            col = QWidget()
            cv  = QVBoxLayout(col)
            cv.setSpacing(6)

            lcd = LcdDisplay(f"通道 {ch}", unit=unit)
            setattr(self, f"_lcd_{ch}", lcd)
            cv.addWidget(lcd)

            ctrl = QHBoxLayout()
            ctrl.addWidget(QLabel("目标:"))

            spin = QDoubleSpinBox()
            if self._is_dh:
                spin.setRange(-self._dh_setpoint_limit, self._dh_setpoint_limit)
                spin.setDecimals(0)
                spin.setSingleStep(1)
            else:
                spin.setRange(-100.0, 100.0)
                spin.setDecimals(3)
                spin.setSingleStep(0.1)
            spin.setValue(0.0)
            spin.setFixedWidth(110)
            setattr(self, f"_disp_spin_{ch}", spin)
            ctrl.addWidget(spin)

            send_btn = QPushButton("发送")
            send_btn.setFixedWidth(55)
            send_btn.clicked.connect(lambda _ch, c=ch: self._send_displacement(c))
            ctrl.addWidget(send_btn)
            ctrl.addStretch()

            cv.addLayout(ctrl)
            h.addWidget(col)

        # 巅慧：轴方向反转勾选框
        if self._is_dh:
            inv_row = QHBoxLayout()
            inv_row.addStretch()
            inv_row.addWidget(QLabel("轴方向反转:"))

            self._chk_invert_x = QCheckBox("X 轴反向")
            self._chk_invert_x.setToolTip(
                "勾选后 X 轴所有输出值取反（翻转 X 方向），可用于匹配其他设备极性"
            )
            saved_ix = self._config.getboolean(
                "Protocol", self._dh_cfg_key("invert_x"), fallback=False
            )
            self._chk_invert_x.setChecked(saved_ix)
            self._chk_invert_x.stateChanged.connect(self._on_axis_invert_changed)
            inv_row.addWidget(self._chk_invert_x)

            self._chk_invert_y = QCheckBox("Y 轴反向")
            self._chk_invert_y.setToolTip(
                "勾选后 Y 轴所有输出值取反（翻转 Y 方向），可用于匹配其他设备极性"
            )
            saved_iy = self._config.getboolean(
                "Protocol", self._dh_cfg_key("invert_y"), fallback=False
            )
            self._chk_invert_y.setChecked(saved_iy)
            self._chk_invert_y.stateChanged.connect(self._on_axis_invert_changed)
            inv_row.addWidget(self._chk_invert_y)

            inv_row.addStretch()
            h.addLayout(inv_row)  # 追加到位移控制区底部

        return group

    # --- 波形控制区 ---
    def _build_wave_area(self) -> QWidget:
        if self._is_dh230:
            title = "波形控制（软件正弦 1 kHz，XY 同时）"
        elif self._is_dh:
            title = "波形控制（硬件正弦，XY 同时）"
        else:
            title = "波形控制（数字模式）"
        group = QGroupBox(title)
        v = QVBoxLayout(group)
        v.setSpacing(6)

        self._wave_widgets: list[WaveChannelWidget] = []
        if self._is_dh230:
            self._dh_sine_widget = DhXySineWidget(
                setpoint_limit=DH230_SETPOINT_LIMIT,
                freq_max=DH230_SINE_FREQ_MAX,
                title="XY 同时正弦扫描（上位机 1 kHz 逐点下发）",
                freq_tip=(
                    "上位机按 1 kHz 逐点下发设定值，范围 0~500 Hz（奈奎斯特上限）。\n"
                    "频率越高每周期点数越少：100 Hz = 10 点/周期，建议不超过 100~200 Hz。"
                ),
                hint=(
                    "软件正弦：1 kHz 逐点下发闭环设定值，以 0 为中心，不支持偏置。"
                    "正弦期间不刷新位置显示（仅定期清空接收缓冲区，保证 1 ms 间隔稳定）；"
                    "停止后回到位移控制区的设定位置。"
                ),
            )
            self._dh_sine_widget.set_callbacks(self._send_dh_sine, self._stop_dh_sine)
            v.addWidget(self._dh_sine_widget)
        elif self._is_dh:
            self._dh_sine_widget = DhXySineWidget()
            self._dh_sine_widget.set_callbacks(self._send_dh_sine, self._stop_dh_sine)
            v.addWidget(self._dh_sine_widget)
        else:
            for ch in range(2):
                w = WaveChannelWidget(ch, protocol_type=self._protocol)
                w.set_callbacks(self._send_wave, self._stop_wave)
                v.addWidget(w)
                self._wave_widgets.append(w)

        if self._is_dh:
            # ── 数据记录行 ─────────────────────────────────────────────
            line2 = QFrame()
            line2.setFrameShape(QFrame.HLine)
            line2.setStyleSheet("color: #444444;")
            v.addWidget(line2)

            rec_row = QHBoxLayout()
            rec_row.setSpacing(10)

            self._rec_btn = QPushButton("● 开始记录")
            self._rec_btn.setMinimumWidth(120)
            self._rec_btn.setStyleSheet(self._rec_btn_style(recording=False))
            self._rec_btn.clicked.connect(self._on_rec_toggle)
            rec_row.addWidget(self._rec_btn)

            self._rec_count_lbl = QLabel("已记录: 0 帧")
            self._rec_count_lbl.setStyleSheet("font-size: 13px; color: #aaaaaa;")
            rec_row.addWidget(self._rec_count_lbl)

            rec_row.addStretch()

            self._rec_save_btn = QPushButton("💾 保存 CSV")
            self._rec_save_btn.setEnabled(False)
            self._rec_save_btn.setStyleSheet(
                "QPushButton{padding: 4px 14px;}"
                "QPushButton:disabled{background-color:#444;color:#777;}"
            )
            self._rec_save_btn.clicked.connect(self._on_rec_save)
            rec_row.addWidget(self._rec_save_btn)

            v.addLayout(rec_row)

        return group

    # --- TCP 日志 ---
    def _build_tcp_log_area(self) -> QWidget:
        group = QGroupBox("TCP 日志")
        v = QVBoxLayout(group)
        v.setContentsMargins(6, 6, 6, 6)

        self._tcp_log = QTextEdit()
        self._tcp_log.setReadOnly(True)
        self._tcp_log.setStyleSheet(
            "QTextEdit{background-color:#1e1e1e;color:#d4d4d4;"
            "font-family:Consolas,monospace;font-size:13px;}"
        )
        v.addWidget(self._tcp_log)
        return group

    # ================================================================
    # 串口连接 / 断开
    # ================================================================

    def _refresh_ports(self):
        saved = self._config.get("Serial", "port", fallback="")
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self._port_combo.clear()
        self._port_combo.addItems(ports)
        if saved and saved in ports:
            self._port_combo.setCurrentText(saved)
        elif ports:
            self._port_combo.setCurrentIndex(0)

    def _on_connect_toggle(self, checked: bool):
        if checked:
            self._do_connect()
        else:
            self._do_disconnect()

    def _do_connect(self):
        port     = self._port_combo.currentText()
        baudrate = int(self._baud_combo.currentText())
        if not port:
            self._show_error("请先选择串口")
            self._connect_btn.setChecked(False)
            return
        try:
            if self._is_dh:
                if self._is_dh230:
                    self._dh = DianghuiFsm230Controller(port=port, baudrate=baudrate)
                    self._dh.open()
                    self._dh_wave_mgr = DianghuiFsm230WaveManager(self._dh)
                else:
                    self._dh = DianghuiController(port=port, baudrate=baudrate)
                    self._dh.open()
                    self._dh_wave_mgr = DianghuiWaveManager(self._dh)
                # 恢复轴反向设置
                if self._chk_invert_x:
                    self._dh_wave_mgr.invert_x = self._chk_invert_x.isChecked()
                if self._chk_invert_y:
                    self._dh_wave_mgr.invert_y = self._chk_invert_y.isChecked()
                # 恢复连发 / 采样率 / 插值设置
                if self._chk_analog_burst:
                    self._dh_wave_mgr.analog_burst = self._chk_analog_burst.isChecked()
                self._dh_wave_mgr.analog_interval_ms = self._dh_analog_interval_ms
                if self._chk_analog_interp:
                    self._dh_wave_mgr.analog_interpolate = self._chk_analog_interp.isChecked()
            else:
                address = self._addr_spin.value() if self._addr_spin else 1
                self._fsc = FSCController(port=port, address=address, baudrate=baudrate)
                self._fsc.open()

            self._connect_btn.setText("断开")
            self._connect_btn.setStyleSheet(
                "QPushButton{background-color:#8b1a1a;color:white;}"
            )
            self._proto_combo.setEnabled(False)   # 连接期间不允许切换协议
            self._poll_error_count = 0
            self._status_bar.showMessage(f"已连接: {port}  波特率: {baudrate}")

            self._config.set("Serial", "port", port)
            self._config.set("Serial", f"baudrate_{self._protocol}", str(baudrate))
            if self._addr_spin:
                self._config.set("Serial", "address", str(self._addr_spin.value()))
            self._save_config()

            self._auto_init()

        except Exception as exc:
            self._fsc = None
            self._dh  = None
            self._connect_btn.setChecked(False)
            self._show_error(f"连接失败: {exc}")

    def _do_disconnect(self):
        # 停止巅慧波形线程；若有未保存的录制数据先缓存到 GUI
        if self._dh_wave_mgr:
            if self._dh_wave_mgr.recording_active:
                self._dh_wave_mgr.stop_recording()
            n = self._dh_wave_mgr.recording_sample_count
            if n > 0:
                self._cached_rec_data = self._dh_wave_mgr.get_recording()
                self._apply_rec_stopped_ui()   # 更新按钮状态
            self._dh_wave_mgr.stop_all()
            self._dh_wave_mgr = None

        # 关闭串口
        for ctrl in (self._fsc, self._dh):
            if ctrl:
                try:
                    ctrl.close()
                except Exception:
                    pass
        self._fsc = None
        self._dh  = None

        self._connect_btn.setText("连接")
        self._connect_btn.setStyleSheet("")
        self._connect_btn.setChecked(False)
        self._proto_combo.setEnabled(True)

        if self._mode_btn:
            self._mode_btn.setEnabled(False)
        if self._loop_btn:
            self._loop_btn.setEnabled(False)
        if self._analog_play_btn:
            has_data = bool(self._analog_pos_x and self._analog_pos_y)
            self._analog_play_btn.setEnabled(has_data)
        if self._analog_stop_btn:
            self._analog_stop_btn.setEnabled(False)
        self._dh_sine_active = False
        if self._dh_status_panel:
            self._dh_status_panel.set_unknown()

        self._status_bar.showMessage("未连接")

    # ================================================================
    # 连接后自动初始化
    # ================================================================

    def _auto_init(self):
        if self._is_dh230:
            self._auto_init_dianghui230()
        elif self._is_dh:
            self._auto_init_dianghui()
        else:
            self._auto_init_fsc()

    def _auto_init_dianghui230(self):
        """
        巅慧 FSM230 初始化：协议无自检指令，直接发闭环指令并归零。
        设备应答（状态帧）即视为通信正常。
        """
        try:
            status = self._dh.send_control(x_phys=0, y_phys=0)
        except Exception as exc:
            self._show_error(f"初始化控制指令失败: {exc}")
            return
        msg = self._status_bar.currentMessage()
        if status:
            self._update_dianghui_feedback(status)
            self._status_bar.showMessage(msg + "  |  初始化完成: 闭环控制，位置归零")
        else:
            self._status_bar.showMessage(
                msg + "  |  警告: 设备无应答（请检查波特率 / 接线 / 电源）"
            )

    def _auto_init_fsc(self):
        """芯明天: 设置数字模式 + 闭环，逐步确认。"""
        errs = []

        for ch in range(2):
            try:
                self._fsc.set_signal_mode(channel=ch, analog=False)
            except _COMM_ERRORS as e:
                errs.append(f"通道 {ch} 设置数字模式失败: {e}")

        for ch in range(2):
            try:
                if self._fsc.get_signal_mode(channel=ch):
                    errs.append(f"通道 {ch} 确认异常: 读回仍为模拟")
            except _COMM_ERRORS as e:
                errs.append(f"通道 {ch} 读取信号模式失败: {e}")

        for ch in range(2):
            try:
                self._fsc.set_loop_mode(channel=ch, closed=True)
            except _COMM_ERRORS as e:
                errs.append(f"通道 {ch} 设置闭环失败: {e}")

        for ch in range(2):
            try:
                if not self._fsc.get_loop_mode(channel=ch):
                    errs.append(f"通道 {ch} 确认异常: 读回仍为开环")
            except _COMM_ERRORS as e:
                errs.append(f"通道 {ch} 读取开闭环失败: {e}")

        self._is_analog      = False
        self._is_closed_loop = True
        self._update_mode_button()
        self._update_loop_button()
        if self._mode_btn:
            self._mode_btn.setEnabled(True)
        if self._loop_btn:
            self._loop_btn.setEnabled(True)

        if errs:
            self._show_error("初始化警告（操作已继续）:\n" + "\n".join(errs))
        else:
            self._status_bar.showMessage(
                self._status_bar.currentMessage() + "  |  初始化完成: 数字模式 + 闭环"
            )

    def _auto_init_dianghui(self):
        """
        巅慧初始化第一步: 发送自检指令。
        为避免阻塞 GUI，1.1 秒后通过 QTimer 执行第二步（闭环 + 归零）。
        """
        try:
            status = self._dh.send_selfcheck()
            if status and self._dh_status_panel:
                self._dh_status_panel.update_status(status)
            msg = self._status_bar.currentMessage()
            self._status_bar.showMessage(msg + "  |  自检指令已发送，1 秒后完成初始化...")
        except Exception as exc:
            self._show_error(f"发送自检指令失败: {exc}")
            return

        QTimer.singleShot(1100, self._auto_init_dianghui_step2)

    def _auto_init_dianghui_step2(self):
        """巅慧初始化第二步: 进入控制模式，位置归零，设置限位 5000。"""
        if not self._dh:
            return
        try:
            status = self._dh.send_control(
                x_phys=0, y_phys=0,
                x_lim=DH_DEFAULT_LIMIT, y_lim=DH_DEFAULT_LIMIT,
            )
            if status:
                if self._dh_status_panel:
                    self._dh_status_panel.update_status(status)
                selfck_ok = not status.selfcheck_error
                self._status_bar.showMessage(
                    f"已连接  |  初始化完成: 自检{'正常' if selfck_ok else '异常'}，"
                    f"位置归零，限位 {DH_DEFAULT_LIMIT}"
                )
                if not selfck_ok:
                    self._show_error("巅慧自检未通过（selfcheck_error），请检查设备连接")
        except Exception as exc:
            self._show_error(f"初始化控制指令失败: {exc}")

    # ================================================================
    # 芯明天: 模式切换
    # ================================================================

    @staticmethod
    def _mode_btn_style(is_analog: bool) -> str:
        bg, hv = ("#6e3a1e", "#a05228") if is_analog else ("#1e3a6e", "#2a52a0")
        return (
            f"QPushButton{{background-color:{bg};color:white;font-size:15px;"
            f"font-weight:bold;border-radius:6px;padding:0 16px;}}"
            f"QPushButton:hover{{background-color:{hv};}}"
            f"QPushButton:disabled{{background-color:#555;color:#999;}}"
        )

    @staticmethod
    def _loop_btn_style(is_closed: bool) -> str:
        bg, hv = ("#1a5c1a", "#247a24") if is_closed else ("#7a6010", "#a07e18")
        return (
            f"QPushButton{{background-color:{bg};color:white;font-size:15px;"
            f"font-weight:bold;border-radius:6px;padding:0 16px;}}"
            f"QPushButton:hover{{background-color:{hv};}}"
            f"QPushButton:disabled{{background-color:#555;color:#999;}}"
        )

    def _on_mode_toggle(self):
        if not self._fsc:
            return
        self._is_analog = not self._is_analog
        try:
            self._fsc.set_signal_mode(channel=0, analog=self._is_analog)
            self._fsc.set_signal_mode(channel=1, analog=self._is_analog)
        except _COMM_ERRORS as exc:
            self._show_error(f"切换模式失败: {exc}")
            self._is_analog = not self._is_analog
            return
        self._update_mode_button()

    def _update_mode_button(self):
        if not self._mode_btn:
            return
        text = ("当前模式: 模拟  →  切换为数字" if self._is_analog
                else "当前模式: 数字  →  切换为模拟")
        self._mode_btn.setText(text)
        self._mode_btn.setStyleSheet(self._mode_btn_style(self._is_analog))

    def _on_loop_toggle(self):
        if not self._fsc:
            return
        self._is_closed_loop = not self._is_closed_loop
        try:
            self._fsc.set_loop_mode(channel=0, closed=self._is_closed_loop)
            self._fsc.set_loop_mode(channel=1, closed=self._is_closed_loop)
        except _COMM_ERRORS as exc:
            self._show_error(f"切换开闭环失败: {exc}")
            self._is_closed_loop = not self._is_closed_loop
            return
        self._update_loop_button()

    def _update_loop_button(self):
        if not self._loop_btn:
            return
        text = ("当前状态: 闭环  →  切换为开环" if self._is_closed_loop
                else "当前状态: 开环  →  切换为闭环")
        self._loop_btn.setText(text)
        self._loop_btn.setStyleSheet(self._loop_btn_style(self._is_closed_loop))

    # ================================================================
    # 巅慧: 模拟位置文件控制
    # ================================================================

    @staticmethod
    def _clamp_int16(value: float) -> int:
        return max(-32768, min(32767, round(value)))

    @staticmethod
    def _parse_comment_interval_ms(comment: str) -> Optional[int]:
        """仅当注释内容以 sample_interval_ms 开头时解析间隔。"""
        body = comment.lstrip("#").strip()
        lowered = body.lower().replace(" ", "").replace("　", "")
        token = "sample_interval_ms"
        if not lowered.startswith(token):
            return None
        rest = lowered[len(token):].lstrip(":=：")
        digits: list[str] = []
        for ch in rest:
            if ch.isdigit():
                digits.append(ch)
            elif digits:
                break
        if not digits:
            return None
        val = int("".join(digits))
        return val if val > 0 else None

    def _on_axis_invert_changed(self):
        """轴反向勾选框变更：实时同步到波形管理器并保存配置。"""
        invert_x = bool(self._chk_invert_x and self._chk_invert_x.isChecked())
        invert_y = bool(self._chk_invert_y and self._chk_invert_y.isChecked())
        if self._dh_wave_mgr:
            self._dh_wave_mgr.invert_x = invert_x
            self._dh_wave_mgr.invert_y = invert_y
        self._config.set("Protocol", self._dh_cfg_key("invert_x"), str(invert_x))
        self._config.set("Protocol", self._dh_cfg_key("invert_y"), str(invert_y))
        self._save_config()
        x_str = "反向" if invert_x else "正向"
        y_str = "反向" if invert_y else "正向"
        self._status_bar.showMessage(f"轴极性更新: X={x_str}  Y={y_str}（实时生效）")

    def _on_analog_burst_changed(self):
        """连发勾选框变更：实时同步到波形管理器并保存配置。"""
        burst = bool(self._chk_analog_burst and self._chk_analog_burst.isChecked())
        if self._dh_wave_mgr:
            self._dh_wave_mgr.analog_burst = burst
        self._config.set("Protocol", self._dh_cfg_key("analog_burst"), str(burst))
        self._save_config()
        mode_str = f"{self._dh_burst_n}帧连发" if burst else "单帧"
        self._status_bar.showMessage(f"模拟发送模式: {mode_str}（实时生效）")

    def _on_analog_interp_changed(self):
        """线性插值勾选框变更：实时同步到波形管理器并保存配置。"""
        interp = bool(self._chk_analog_interp and self._chk_analog_interp.isChecked())
        if self._dh_wave_mgr:
            self._dh_wave_mgr.analog_interpolate = interp
        self._config.set("Protocol", self._dh_cfg_key("analog_interpolate"), str(interp))
        self._save_config()
        mode_str = "线性插值（相邻点间生成不同值）" if interp else "重复同值"
        self._status_bar.showMessage(f"连发模式: {mode_str}（实时生效）")

    def _update_analog_rate_btn(self):
        """刷新采样率切换按钮的显示文字。"""
        if not self._analog_rate_btn:
            return
        iv = self._dh_analog_interval_ms
        hz = 1000 // iv
        other_iv = 1 if iv == 2 else 2
        other_hz = 1000 // other_iv
        self._analog_rate_btn.setText(
            f"当前: {hz}Hz ({iv}ms/点)  ↔  切换到 {other_hz}Hz"
        )

    def _on_analog_rate_toggle(self):
        """切换模拟采样率（500Hz ↔ 1000Hz），并重新对已加载文件抽点。"""
        new_iv = 1 if self._dh_analog_interval_ms == 2 else 2
        self._dh_analog_interval_ms = new_iv
        self._update_analog_rate_btn()
        if self._dh_wave_mgr:
            self._dh_wave_mgr.analog_interval_ms = new_iv
        self._config.set("Protocol", self._dh_cfg_key("analog_interval_ms"), str(new_iv))
        self._save_config()
        # 重新对已加载文件抽点
        if self._analog_pos_raw_x:
            self._resample_analog_pos()
            self._update_analog_file_lbl()
        hz = 1000 // new_iv
        self._status_bar.showMessage(
            f"模拟采样率: {hz}Hz ({new_iv}ms/点)（实时生效；文件已自动重新抽点）"
        )

    def _resample_analog_pos(self) -> bool:
        """
        根据当前 `_dh_analog_interval_ms` 从原始点列重新抽点，更新 `_analog_pos_x/y`。

        - 若发送间隔 >= 文件间隔：正常抽点（step = iv // fi）。
        - 若发送间隔 < 文件间隔：文件精度不足，保留所有原始点（step=1），
          实际等效采样率受文件限制。
        返回 True 表示成功，False 表示原始点列为空。
        """
        if not (self._analog_pos_raw_x and self._analog_pos_raw_y):
            return False
        fi = self._analog_file_interval_ms
        iv = self._dh_analog_interval_ms
        step = max(1, iv // fi)
        pos_x = self._analog_pos_raw_x[::step]
        pos_y = self._analog_pos_raw_y[::step]
        if not pos_x:
            return False
        self._analog_pos_x = pos_x
        self._analog_pos_y = pos_y
        return True

    def _update_analog_file_lbl(self):
        """根据当前点列和采样设置刷新文件标签。"""
        if not self._analog_pos_raw_x or not self._analog_pos_file or not self._analog_file_lbl:
            return
        n_src = len(self._analog_pos_raw_x)
        fi    = self._analog_file_interval_ms
        iv    = self._dh_analog_interval_ms
        n_snd = len(self._analog_pos_x) if self._analog_pos_x else 0
        dur   = n_snd * iv / 1000.0
        lbl   = (
            f"{os.path.basename(self._analog_pos_file)}  "
            f"(原始 {n_src}点/{fi}ms → 下发 {n_snd}点/{iv}ms, {dur:.2f}s)"
        )
        self._analog_file_lbl.setText(lbl)
        self._analog_file_lbl.setStyleSheet("color: #00cc44;")

    def _on_load_analog_pos(self):
        """
        加载模拟位置 CSV（两列 X,Y）。

        解析后以文件原始精度缓存；按当前 `_dh_analog_interval_ms` 抽点后下发。
        切换采样率时调用 `_resample_analog_pos` 重新抽点，无需重新加载文件。
        若注释中写有 ``sample_interval_ms: N``，则按该值识别文件精度。
        """
        poll_was_active = self._pause_serial_poll()
        try:
            path, _ = QFileDialog.getOpenFileName(
                self, "加载模拟位置文件",
                os.path.join(ROOT_PATH, "analog_positions"),
                "CSV 文件 (*.csv);;所有文件 (*)",
            )
        finally:
            self._resume_serial_poll(poll_was_active)
        if not path:
            return
        try:
            pos_x: list[int] = []
            pos_y: list[int] = []
            header_skipped = False
            file_interval_ms = 1
            with open(path, newline="", encoding="utf-8-sig") as f:
                for line_no, row in enumerate(csv.reader(f), start=1):
                    cells = [c.strip() for c in row]
                    if not cells or all(not c for c in cells):
                        continue
                    if cells[0].startswith("#"):
                        parsed = self._parse_comment_interval_ms(",".join(cells))
                        if parsed is not None:
                            file_interval_ms = parsed
                        continue
                    if len(cells) < 2:
                        raise ValueError(
                            f"第 {line_no} 行只有 {len(cells)} 列，需要两列（X,Y）"
                        )
                    try:
                        x_val = self._clamp_int16(float(cells[0]))
                        y_val = self._clamp_int16(float(cells[1]))
                    except ValueError:
                        if not pos_x and not header_skipped:
                            header_skipped = True
                            continue
                        raise ValueError(
                            f"第 {line_no} 行无法解析为数值: {cells[0]!r}, {cells[1]!r}"
                        )
                    pos_x.append(x_val)
                    pos_y.append(y_val)
            if not pos_x:
                raise ValueError("位置列表为空")

            # 存原始点列（文件精度），之后切换采样率无需重新加载
            self._analog_pos_raw_x        = pos_x
            self._analog_pos_raw_y        = pos_y
            self._analog_file_interval_ms = file_interval_ms
            self._analog_pos_file         = path

            if not self._resample_analog_pos():
                raise ValueError("抽点后位置列表为空")

            self._update_analog_file_lbl()
            if self._analog_play_btn:
                self._analog_play_btn.setEnabled(True)
        except Exception as exc:
            self._show_error(f"加载文件失败: {exc}")

    def _on_play_analog_pos(self):
        if not (self._dh and self._dh_wave_mgr):
            self._show_error("串口未连接")
            return
        if not (self._analog_pos_x and self._analog_pos_y):
            self._show_error("请先加载位置文件")
            return
        self._dh_sine_active = False
        self._dh_wave_mgr.start_wave_xy(self._analog_pos_x, self._analog_pos_y)
        n  = len(self._analog_pos_x)
        iv = self._dh_analog_interval_ms
        hz = 1000 // iv
        duration_s = n * iv / 1000.0
        burst = bool(self._chk_analog_burst and self._chk_analog_burst.isChecked())
        interp = bool(self._chk_analog_interp and self._chk_analog_interp.isChecked())
        mode_str = (
            f"{self._dh_burst_n}帧{'插值' if interp else '重复'}连发"
            if burst else "单帧"
        )
        self._status_bar.showMessage(
            f"模拟位置发送中: {n}点/{iv}ms({hz}Hz), {mode_str}, 时长{duration_s:.2f}s 循环"
        )
        if self._analog_stop_btn:
            self._analog_stop_btn.setEnabled(True)

    def _on_stop_analog_pos(self):
        if self._dh_wave_mgr:
            self._dh_wave_mgr.stop_all()
        if self._dh:
            x_val = round(self._disp_spin_0.value())
            y_val = round(self._disp_spin_1.value())
            try:
                self._dh.send_control(x_phys=x_val, y_phys=y_val)
            except _COMM_ERRORS as exc:
                self._show_error(f"停止模拟位置失败: {exc}")
                return
        if self._analog_stop_btn:
            self._analog_stop_btn.setEnabled(False)
        self._status_bar.showMessage("模拟位置已停止，已返回设定位置")

    # ================================================================
    # 远程指令触发的 UI 更新（主线程槽，由信号安全调用）
    # ================================================================

    def _apply_mode_from_remote(self, is_analog: bool):
        if self._protocol != PROTOCOL_XINMINGTIAN:
            return
        self._is_analog = is_analog
        self._update_mode_button()
        self._status_bar.showMessage(f"[远程] 信号模式: {'模拟' if is_analog else '数字'}")

    def _apply_loop_from_remote(self, is_closed: bool):
        if self._protocol != PROTOCOL_XINMINGTIAN:
            return
        self._is_closed_loop = is_closed
        self._update_loop_button()
        self._status_bar.showMessage(f"[远程] 开闭环: {'闭环' if is_closed else '开环'}")

    def _apply_disp_from_remote(self, channel: int, displacement: float):
        if 0 <= channel < len(self._wave_widgets):
            self._wave_widgets[channel].update_last_sent(displacement)
        self._status_bar.showMessage(f"[远程] 通道 {channel} 位移: {displacement:.3f}")

    # ================================================================
    # 位移轮询（1 秒定时）
    # ================================================================

    # 巅慧 100ms × 10 ≈ 1s 后断连；芯明天 1000ms × 3 ≈ 3s 后断连
    _POLL_ERROR_THRESHOLD_DIANHUI     = 10
    _POLL_ERROR_THRESHOLD_XINMINGTIAN = 3

    @property
    def _poll_error_threshold(self) -> int:
        return (self._POLL_ERROR_THRESHOLD_DIANHUI
                if self._is_dh
                else self._POLL_ERROR_THRESHOLD_XINMINGTIAN)

    def _pause_serial_poll(self) -> bool:
        """暂停位移轮询。Windows 原生文件对话框期间串口读会超时，须先停表。"""
        active = self._disp_timer.isActive()
        self._disp_timer.stop()
        return active

    def _resume_serial_poll(self, was_active: bool) -> None:
        self._poll_error_count = 0
        if was_active:
            self._disp_timer.start()

    def _poll_displacement(self):
        if self._is_dh:
            self._poll_dianghui()
        else:
            self._poll_fsc()

    def _poll_fsc(self):
        if not self._fsc:
            return
        any_err = False
        for ch in range(2):
            lcd: LcdDisplay = getattr(self, f"_lcd_{ch}")
            try:
                val = self._fsc.read_displacement(channel=ch)
                self._displacement[ch] = val
                lcd.set_value(val)
                self._wave_widgets[ch].update_displacement(val)
            except (_COMM_ERRORS, Exception):
                lcd.set_error()
                any_err = True
        if any_err:
            self._poll_error_count += 1
            if self._poll_error_count >= self._poll_error_threshold:
                self._on_device_lost()
        else:
            self._poll_error_count = 0

    def _poll_dianghui(self):
        if not self._dh:
            return
        # 刷新录制帧计数（仅在录制中才更新标签）
        if (self._dh_wave_mgr and self._dh_wave_mgr.recording_active
                and self._rec_count_lbl):
            n = self._dh_wave_mgr.recording_sample_count
            self._rec_count_lbl.setText(f"已记录: {n} 帧")
        try:
            # 波形线程运行时从缓存取状态，避免串口争抢
            if self._dh_wave_mgr and self._dh_wave_mgr.is_running:
                # FSM230 软件正弦期间不刷新位置（只清接收缓冲区）：
                # 保持显示不变，且不能把“无新数据”当作通信异常
                if not getattr(self._dh_wave_mgr, "status_available", True):
                    self._poll_error_count = 0
                    return
                status = self._dh_wave_mgr.last_status
            else:
                status = self._dh.send_readback()

            if status:
                self._update_dianghui_feedback(status)
                self._poll_error_count = 0
            else:
                self._poll_error_count += 1
                if self._poll_error_count >= self._poll_error_threshold:
                    self._on_device_lost()
        except _COMM_ERRORS:
            self._poll_error_count += 1
            if self._poll_error_count >= self._poll_error_threshold:
                self._on_device_lost()

    def _update_dianghui_feedback(self, status: Union[DianghuiStatus, DianghuiFsm230Status]):
        lcd0: LcdDisplay = self._lcd_0
        lcd1: LcdDisplay = self._lcd_1
        lcd0.set_int_value(status.x_feedback)
        lcd1.set_int_value(status.y_feedback)
        self._displacement[0] = float(status.x_feedback)
        self._displacement[1] = float(status.y_feedback)
        if len(self._wave_widgets) >= 2:
            self._wave_widgets[0].update_displacement(float(status.x_feedback))
            self._wave_widgets[1].update_displacement(float(status.y_feedback))
        if self._dh_status_panel:
            self._dh_status_panel.update_status(status)

    def _on_device_lost(self):
        self._poll_error_count = 0
        self._do_disconnect()
        self._status_bar.showMessage("设备断连：串口通信失败，已自动断开")

    # ================================================================
    # 位移发送
    # ================================================================

    def _send_displacement(self, channel: int):
        spin: QDoubleSpinBox = getattr(self, f"_disp_spin_{channel}")
        val = spin.value()
        try:
            if self._is_dh:
                if not self._dh:
                    self._show_error("串口未连接")
                    return
                ival = round(val)
                other_ch = 1 - channel
                other_spin: QDoubleSpinBox = getattr(self, f"_disp_spin_{other_ch}")
                other_ival = round(other_spin.value())
                if self._dh_wave_mgr:
                    if self._dh_wave_mgr.sine_active:
                        self._dh_wave_mgr.stop_all()
                        self._dh_sine_active = False
                    if channel == 0:
                        self._dh_wave_mgr.update_static_x(ival)
                    else:
                        self._dh_wave_mgr.update_static_y(ival)
                if channel == 0:
                    self._dh.send_control(x_phys=ival, y_phys=other_ival)
                else:
                    self._dh.send_control(x_phys=other_ival, y_phys=ival)
            else:
                if not self._fsc:
                    self._show_error("串口未连接")
                    return
                self._fsc.send_displacement(channel=channel, displacement=val)

            if 0 <= channel < len(self._wave_widgets):
                self._wave_widgets[channel].update_last_sent(val)
            self._status_bar.showMessage(f"通道 {channel} 位移已发送: {val:.3f}")
        except _COMM_ERRORS as exc:
            self._show_error(f"发送位移失败: {exc}")

    # ================================================================
    # 波形控制
    # ================================================================

    def _send_dh_sine(self):
        """发送 XY 同时硬件正弦（指令 0x06）。界面峰峰值 / 2 后作为协议幅值。"""
        if not (self._dh and self._dh_wave_mgr and self._dh_sine_widget):
            self._show_error("串口未连接")
            return
        try:
            peak_peak = self._dh_sine_widget.get_peak_peak()
            frequency = self._dh_sine_widget.get_frequency()
            amplitude = self._dh_sine_widget.get_amplitude()
            if self._analog_stop_btn:
                self._analog_stop_btn.setEnabled(False)
            self._dh_wave_mgr.start_hw_sine("xy", frequency, amplitude)
            self._dh_sine_active = True
            if self._is_dh230:
                self._status_bar.showMessage(
                    f"XY 同时软件正弦: 峰峰={peak_peak} → 幅值(峰值)={amplitude}  "
                    f"频率={frequency}Hz  1kHz 逐点下发（正弦期间不刷新位置）"
                )
            else:
                self._status_bar.showMessage(
                    f"XY 同时正弦: 峰峰={peak_peak} → 幅值(峰值)={amplitude}  "
                    f"频率={frequency}Hz  1kHz 保活"
                )
        except _COMM_ERRORS as exc:
            self._show_error(f"发送正弦失败: {exc}")
        except ValueError as exc:
            self._show_error(f"参数错误: {exc}")

    def _stop_dh_sine(self):
        """停止硬件正弦，线程退出后再发闭环位置，回到位移控制区设定值。"""
        if not (self._dh and self._dh_wave_mgr):
            self._show_error("串口未连接")
            return
        try:
            x_val = round(self._disp_spin_0.value())
            y_val = round(self._disp_spin_1.value())
            self._dh_wave_mgr.stop_all()
            self._dh.send_control(x_phys=x_val, y_phys=y_val)
            self._dh_sine_active = False
            self._status_bar.showMessage(
                f"正弦已停止，已返回设定位置 X={x_val} Y={y_val} counts"
            )
        except _COMM_ERRORS as exc:
            self._show_error(f"停止正弦失败: {exc}")

    def _send_wave(self, channel: int, wave_type: int,
                   peak_peak: float, frequency: float, offset: float):
        """芯明天通道波形。"""
        try:
            if not self._fsc:
                self._show_error("串口未连接")
                return
            self._fsc.send_wave_displacement(
                channel   = channel,
                wave_type = wave_type,
                peak_peak = peak_peak,
                frequency = frequency,
                offset    = offset,
            )
            self._status_bar.showMessage(
                f"通道 {channel} 波形已发送: 峰峰={peak_peak}  频率={frequency}Hz  偏置={offset}"
            )
        except _COMM_ERRORS as exc:
            self._show_error(f"发送波形失败: {exc}")
        except ValueError as exc:
            self._show_error(f"参数错误: {exc}")

    def _stop_wave(self, channel: int, displacement: float):
        """芯明天停止波形。"""
        try:
            if not self._fsc:
                self._show_error("串口未连接")
                return
            self._fsc.send_displacement(channel=channel, displacement=displacement)
            self._status_bar.showMessage(
                f"通道 {channel} 波形已停止（位移={displacement:.3f}）"
            )
        except _COMM_ERRORS as exc:
            self._show_error(f"停止波形失败: {exc}")

    # ================================================================
    # 数据记录（巅慧专用）
    # ================================================================

    @staticmethod
    def _rec_btn_style(recording: bool) -> str:
        if recording:
            return (
                "QPushButton{background-color:#8b1a1a;color:white;font-weight:bold;"
                "padding:4px 14px;border-radius:4px;}"
                "QPushButton:hover{background-color:#b52424;}"
            )
        return (
            "QPushButton{background-color:#1a3a6e;color:white;font-weight:bold;"
            "padding:4px 14px;border-radius:4px;}"
            "QPushButton:hover{background-color:#2a52a0;}"
        )

    def _on_rec_toggle(self):
        """开始 / 停止位置数据记录。"""
        if not self._dh_wave_mgr:
            self._show_error("串口未连接，无法记录数据")
            return

        if not self._dh_wave_mgr.recording_active:
            # ── 开始记录 ──────────────────────────────────────────────
            self._dh_wave_mgr.start_recording()
            if self._rec_btn:
                self._rec_btn.setText("■ 停止记录")
                self._rec_btn.setStyleSheet(self._rec_btn_style(recording=True))
            if self._rec_count_lbl:
                self._rec_count_lbl.setText("已记录: 0 帧")
                self._rec_count_lbl.setStyleSheet("font-size:13px;color:#ffcc44;")
            if self._rec_save_btn:
                self._rec_save_btn.setEnabled(False)
            self._status_bar.showMessage("位置数据记录已开始")
        else:
            # ── 停止记录 ──────────────────────────────────────────────
            self._dh_wave_mgr.stop_recording()
            self._apply_rec_stopped_ui()

    def _apply_rec_stopped_ui(self):
        """停止记录后统一更新 UI 状态。"""
        n = (self._dh_wave_mgr.recording_sample_count
             if self._dh_wave_mgr else
             (len(self._cached_rec_data[0]) if self._cached_rec_data else 0))
        if self._rec_btn:
            self._rec_btn.setText("● 开始记录")
            self._rec_btn.setStyleSheet(self._rec_btn_style(recording=False))
        if self._rec_count_lbl:
            self._rec_count_lbl.setText(f"已记录: {n} 帧")
            self._rec_count_lbl.setStyleSheet(
                "font-size:13px;color:#44cc44;" if n > 0 else "font-size:13px;color:#aaaaaa;"
            )
        if self._rec_save_btn:
            self._rec_save_btn.setEnabled(n > 0)
        self._status_bar.showMessage(f"记录已停止，共 {n} 帧")

    def _on_rec_save(self):
        """弹出文件对话框，将记录数据保存为 CSV。"""
        import os
        from datetime import datetime

        default_dir = os.path.join(ROOT_PATH, "recordings")
        os.makedirs(default_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_path = os.path.join(default_dir, f"fsc_record_{ts}.csv")

        poll_was_active = self._pause_serial_poll()
        try:
            path, _ = QFileDialog.getSaveFileName(
                self, "保存位置记录数据", default_path,
                "CSV 文件 (*.csv);;所有文件 (*)"
            )
        finally:
            self._resume_serial_poll(poll_was_active)
        if not path:
            return
        try:
            # 优先从活跃的波形管理器保存，其次用缓存
            if self._dh_wave_mgr and self._dh_wave_mgr.recording_sample_count > 0:
                n = self._dh_wave_mgr.save_recording_csv(path)
            elif self._cached_rec_data:
                import csv as _csv
                times_ms, x_data, y_data = self._cached_rec_data
                with open(path, 'w', newline='', encoding='utf-8-sig') as f:
                    w = _csv.writer(f)
                    w.writerow(['时间_ms', 'X轴反馈_counts', 'Y轴反馈_counts'])
                    for t, x, y in zip(times_ms, x_data, y_data):
                        w.writerow([f'{t:.3f}', x, y])
                n = len(times_ms)
            else:
                self._show_error("没有可保存的记录数据")
                return
            self._status_bar.showMessage(f"已保存 {n} 帧位置记录到: {path}")
        except Exception as exc:
            self._show_error(f"保存失败: {exc}")

    # ================================================================
    # TCP 服务端
    # ================================================================

    def _start_tcp(self):
        self._tcp_server._host = self._tcp_ip()
        self._tcp_server._port = self._tcp_port()
        self._tcp_server.start()

    def _stop_tcp(self):
        self._tcp_server.stop()

    def _register_tcp_handlers(self):
        """向 TCP 服务端注册所有命令处理回调。两种协议尽量保持相同 opcode 接口。"""

        def _require():
            if self._is_dh:
                if not self._dh:
                    raise RuntimeError("巅慧串口未连接")
            else:
                if not self._fsc:
                    raise RuntimeError("芯明天串口未连接")

        # --- set_signal_mode ---
        def h_set_signal_mode(p: dict):
            _require()
            if self._is_dh:
                raise RuntimeError("巅慧不支持数字/模拟切换（请使用 analog_positions 文件）")
            ch     = int(p.get("channel", 0))
            analog = bool(p.get("analog", False))
            self._fsc.set_signal_mode(channel=ch, analog=analog)
            self._sig_mode_changed.emit(analog)

        # --- send_displacement ---
        def h_send_displacement(p: dict):
            _require()
            ch   = int(p.get("channel", 0))
            disp = float(p.get("displacement", 0.0))
            if self._is_dh:
                ival = round(disp)
                if ch == 0:
                    self._dh.send_control(x_phys=ival, y_phys=self._dh.y_setpoint)
                else:
                    self._dh.send_control(x_phys=self._dh.x_setpoint, y_phys=ival)
            else:
                self._fsc.send_displacement(channel=ch, displacement=disp)
            self._sig_disp_sent.emit(ch, disp)

        # --- read_displacement ---
        def h_read_displacement(p: dict):
            _require()
            ch = int(p.get("channel", 0))
            if self._is_dh:
                if (self._dh_wave_mgr and self._dh_wave_mgr.is_running
                        and not getattr(self._dh_wave_mgr, "status_available", True)):
                    raise RuntimeError("软件正弦运行期间不刷新位置，请先 stop_wave 再读取")
                st = (self._dh_wave_mgr.last_status
                      if (self._dh_wave_mgr and self._dh_wave_mgr.is_running)
                      else self._dh.send_readback())
                if st is None:
                    raise RuntimeError("回读失败")
                return st.x_feedback if ch == 0 else st.y_feedback
            else:
                return self._fsc.read_displacement(channel=ch)

        # --- send_wave ---
        def h_send_wave(p: dict):
            _require()
            ch        = int(p.get("channel", 0))
            peak_peak = float(p.get("peak_peak", 0.65))
            frequency = float(p.get("frequency", 20.0))
            offset    = float(p.get("offset", 0.0))
            if self._is_dh:
                if not self._dh_wave_mgr:
                    raise RuntimeError("巅慧波形管理器未启动")
                wave_str = str(p.get("wave_type", "sine")).lower()
                if wave_str not in ("sine", "sin"):
                    raise ValueError("巅慧仅支持正弦（wave_type=sine）")
                axis = str(p.get("axis", p.get("mode", "xy"))).lower()
                if axis not in ("x", "y", "xy"):
                    axis = "x" if ch == 0 else "y"
                amplitude = int(round(peak_peak / 2.0))
                if self._is_dh230:
                    self._dh_wave_mgr.start_sine(axis, frequency, amplitude)
                    kind = "软件正弦"
                    freq_txt = f"{frequency:g}"
                else:
                    self._dh_wave_mgr.start_hw_sine(axis, int(round(frequency)), amplitude)
                    kind = "硬件正弦"
                    freq_txt = str(int(round(frequency)))
                self._dh_sine_active = True
                self._sig_status_msg.emit(
                    f"[远程] {kind} {axis}: 峰峰={peak_peak} → 峰值={amplitude} "
                    f"频率={freq_txt}Hz（无偏置）"
                )
            else:
                wave_str = str(p.get("wave_type", "sine")).lower()
                wave_map = {
                    "sine": WaveType.SINE, "square": WaveType.SQUARE,
                    "triangle": WaveType.TRIANGLE, "sawtooth": WaveType.SAWTOOTH,
                }
                if wave_str not in wave_map:
                    raise ValueError(f"不支持的波形类型: {wave_str}")
                self._fsc.send_wave_displacement(
                    channel=ch, wave_type=wave_map[wave_str],
                    peak_peak=peak_peak, frequency=frequency, offset=offset,
                )
                self._sig_status_msg.emit(
                    f"[远程] 通道 {ch} 波形已发送: 峰峰={peak_peak} 频率={frequency}Hz"
                )

        # --- stop_wave ---
        def h_stop_wave(p: dict):
            _require()
            ch   = int(p.get("channel", 0))
            disp = float(p.get("displacement", 0.0))
            if self._is_dh:
                if not self._dh_wave_mgr:
                    raise RuntimeError("巅慧波形管理器未启动")
                ival = round(disp)
                if ch == 0:
                    x_val, y_val = ival, self._dh.y_setpoint
                else:
                    x_val, y_val = self._dh.x_setpoint, ival
                self._dh_wave_mgr.stop_all()
                self._dh.send_control(x_phys=x_val, y_phys=y_val)
                self._dh_sine_active = False
            else:
                self._fsc.send_displacement(channel=ch, displacement=disp)
            self._sig_disp_sent.emit(ch, disp)
            self._sig_status_msg.emit(f"[远程] 通道 {ch} 波形已停止 disp={disp:.3f}")

        # --- 巅慧专属: send_selfcheck ---
        def h_send_selfcheck(p: dict):
            _require()
            if self._protocol != PROTOCOL_DIANHUI:
                raise RuntimeError("仅巅慧 FSM720 协议支持 send_selfcheck")
            st = self._dh.send_selfcheck()
            return "ok" if (st and not st.selfcheck_error) else "selfcheck_error"

        # --- 巅慧专属: read_status ---
        def h_read_status(p: dict):
            _require()
            if not self._is_dh:
                raise RuntimeError("仅巅慧协议支持 read_status")
            if (self._dh_wave_mgr and self._dh_wave_mgr.is_running
                    and not getattr(self._dh_wave_mgr, "status_available", True)):
                raise RuntimeError("软件正弦运行期间不刷新位置，请先 stop_wave 再读取")
            st = (self._dh_wave_mgr.last_status
                  if (self._dh_wave_mgr and self._dh_wave_mgr.is_running)
                  else self._dh.send_readback())
            if st is None:
                raise RuntimeError("回读失败")
            if self._is_dh230:
                # FSM230 状态帧仅含两轴位移
                return {"x_feedback": st.x_feedback, "y_feedback": st.y_feedback}
            return {
                "comm_error":      st.comm_error,
                "selfcheck_error": st.selfcheck_error,
                "x_enabled":       st.x_enabled,
                "y_enabled":       st.y_enabled,
                "x_disp_over":     st.x_disp_over,
                "y_disp_over":     st.y_disp_over,
                "x_cmd_over":      st.x_cmd_over,
                "y_cmd_over":      st.y_cmd_over,
                "x_feedback":      st.x_feedback,
                "y_feedback":      st.y_feedback,
            }

        self._tcp_server.register("set_signal_mode",  h_set_signal_mode)
        self._tcp_server.register("send_displacement", h_send_displacement)
        self._tcp_server.register("read_displacement", h_read_displacement)
        self._tcp_server.register("send_wave",         h_send_wave)
        self._tcp_server.register("stop_wave",         h_stop_wave)
        self._tcp_server.register("send_selfcheck",    h_send_selfcheck)
        self._tcp_server.register("read_status",       h_read_status)

    # ================================================================
    # TCP 日志（线程安全）
    # ================================================================

    def _append_tcp_log(self, msg: str):
        from PyQt5.QtCore import QMetaObject, Q_ARG
        QMetaObject.invokeMethod(
            self._tcp_log, "append",
            Qt.QueuedConnection,
            Q_ARG(str, msg),
        )

    # ================================================================
    # 工具
    # ================================================================

    def _show_error(self, msg: str):
        QMessageBox.critical(self, "错误", msg)

    def closeEvent(self, event):
        self._disp_timer.stop()
        self._stop_tcp()
        self._do_disconnect()
        super().closeEvent(event)
