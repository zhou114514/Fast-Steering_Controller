"""
快反控制盒上位机主窗口

支持两种协议（通过 config.ini [Protocol] type 字段切换）：
  xinmingtian — 芯明天 FSC，波形由硬件生成，支持数字/模拟切换
  dianghui    — 巅慧，由上位机软件生成正弦序列以 1ms 间隔下发；
                具有轴交换透明处理、8 灯状态面板、模拟位置文件控制
"""

import configparser
import csv
import os
from typing import Optional

import serial
import serial.tools.list_ports
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
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

from dianghui_protocol import (
    DEFAULT_LIMIT as DH_DEFAULT_LIMIT,
    SETPOINT_LIMIT as DH_SETPOINT_LIMIT,
    WAVE_INTERVAL_MS as DH_WAVE_INTERVAL_MS,
    DianghuiController,
    DianghuiError,
    DianghuiStatus,
    DianghuiWaveManager,
    generate_sine_setpoints,
    generate_triangle_setpoints,
)
from fsc_protocol import FSCController, FSCError, WaveType
from tcp_server import FSCTcpServer

# ---------------------------------------------------------------------------
# 常量
# ---------------------------------------------------------------------------

CONFIG_PATH = os.path.join(os.path.dirname(__file__), "config.ini")

PROTOCOL_XINMINGTIAN = "xinmingtian"  # 芯明天 FSC
PROTOCOL_DIANHUI     = "dianhui"     # 巅慧

WAVE_TYPES = {
    "正弦波":  WaveType.SINE,
    "方波":    WaveType.SQUARE,
    "三角波":  WaveType.TRIANGLE,
    "锯齿波":  WaveType.SAWTOOTH,
}

# 巅慧软件生成波形类型（字符串标识，与 WaveType 枚举独立）
DH_WAVE_SINE     = "sine"
DH_WAVE_TRIANGLE = "triangle"
DH_WAVE_TYPES = {
    "正弦波":  DH_WAVE_SINE,
    "三角波":  DH_WAVE_TRIANGLE,
}

COMMON_FREQS = [20, 80, 120, 140, 160]
BAUD_RATES   = ["921600", "460800", "115200", "57600", "38400", "19200", "9600"]

# 各协议的默认波特率（首次使用时无 config 记录时自动选中）
_DEFAULT_BAUD = {
    PROTOCOL_DIANHUI:     "921600",
    PROTOCOL_XINMINGTIAN: "115200",
}

# 所有需捕获的通信异常
_COMM_ERRORS = (
    FSCError, DianghuiError,
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
        if is_dh:
            # 巅慧：软件生成，支持正弦和三角波
            self._wave_combo.addItems(list(DH_WAVE_TYPES.keys()))
            self._wave_combo.setToolTip(
                "正弦波：变速往复，端点平滑；三角波：恒速线性扫描"
            )
        else:
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
        巅慧协议返回字符串标识（DH_WAVE_SINE / DH_WAVE_TRIANGLE）；
        芯明天协议返回 WaveType 整数常量。
        """
        if self._protocol == PROTOCOL_DIANHUI:
            return DH_WAVE_TYPES.get(self._wave_combo.currentText(), DH_WAVE_SINE)
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

        # --- 芯明天状态 ---
        self._fsc: Optional[FSCController] = None
        self._is_analog      = False
        self._is_closed_loop = True

        # --- 巅慧状态 ---
        self._dh: Optional[DianghuiController]         = None
        self._dh_wave_mgr: Optional[DianghuiWaveManager] = None
        self._analog_pos_file:    Optional[str]        = None
        self._analog_pos_data:    Optional[list[int]]  = None
        self._analog_pos_channel: int                  = 0

        # --- 公共状态 ---
        self._displacement      = [0.0, 0.0]
        self._poll_error_count  = 0

        # --- 协议特定 UI 控件（_build_ui 之前先声明为 None） ---
        self._mode_btn:        Optional[QPushButton]          = None
        self._loop_btn:        Optional[QPushButton]          = None
        self._addr_spin:       Optional[QSpinBox]             = None
        self._dh_status_panel: Optional[DianghuiStatusPanel]  = None
        self._analog_file_lbl: Optional[QLabel]               = None
        self._analog_play_btn: Optional[QPushButton]          = None
        self._analog_stop_btn: Optional[QPushButton]          = None
        self._sync_wave_btn:   Optional[QPushButton]          = None
        self._sync_wave_active: bool                          = False
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
        self._disp_timer.setInterval(
            100 if self._protocol == PROTOCOL_DIANHUI else 1000
        )
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
    # UI 构建
    # ================================================================

    def _build_ui(self):
        proto_label = "芯明天 FSC" if self._protocol == PROTOCOL_XINMINGTIAN else "巅慧"
        self.setWindowTitle(f"快反控制盒上位机  [{proto_label}]")
        self.setMinimumWidth(840)

        central = QWidget()
        self.setCentralWidget(central)
        ml = QVBoxLayout(central)
        ml.setSpacing(8)
        ml.setContentsMargins(10, 10, 10, 10)

        ml.addWidget(self._build_connection_bar())

        if self._protocol == PROTOCOL_DIANHUI:
            ml.addWidget(self._build_analog_pos_bar())
        else:
            ml.addWidget(self._build_mode_bar())

        ml.addWidget(self._build_displacement_area())

        if self._protocol == PROTOCOL_DIANHUI:
            self._dh_status_panel = DianghuiStatusPanel()
            ml.addWidget(self._dh_status_panel)

        ml.addWidget(self._build_wave_area())
        ml.addWidget(self._build_tcp_log_area(), stretch=1)

        self._status_bar = QStatusBar()
        self.setStatusBar(self._status_bar)
        self._status_bar.showMessage("未连接")

    # --- 连接栏 ---
    def _build_connection_bar(self) -> QWidget:
        bar = QGroupBox("连接")
        h = QHBoxLayout(bar)
        h.setSpacing(8)

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
        h = QHBoxLayout(bar)
        h.setSpacing(8)

        h.addWidget(QLabel("位置文件:"))
        self._analog_file_lbl = QLabel("未选择")
        self._analog_file_lbl.setStyleSheet("color: #888888;")
        h.addWidget(self._analog_file_lbl, 1)

        load_btn = QPushButton("加载文件...")
        load_btn.clicked.connect(self._on_load_analog_pos)
        h.addWidget(load_btn)

        self._analog_play_btn = QPushButton("发送模拟信号")
        self._analog_play_btn.setEnabled(False)
        self._analog_play_btn.setStyleSheet(
            "QPushButton{background-color:#1a6b2a;color:white;font-weight:bold;padding:4px 12px;}"
            "QPushButton:hover{background-color:#238c38;}"
            "QPushButton:disabled{background-color:#555;color:#999;}"
        )
        self._analog_play_btn.clicked.connect(self._on_play_analog_pos)
        h.addWidget(self._analog_play_btn)

        self._analog_stop_btn = QPushButton("停止")
        self._analog_stop_btn.setEnabled(False)
        self._analog_stop_btn.setStyleSheet(
            "QPushButton{background-color:#8b1a1a;color:white;font-weight:bold;padding:4px 12px;}"
            "QPushButton:hover{background-color:#b52424;}"
            "QPushButton:disabled{background-color:#555;color:#999;}"
        )
        self._analog_stop_btn.clicked.connect(self._on_stop_analog_pos)
        h.addWidget(self._analog_stop_btn)

        return bar

    # --- 位移控制区 ---
    def _build_displacement_area(self) -> QWidget:
        unit  = "counts" if self._protocol == PROTOCOL_DIANHUI else "mrad"
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
            if self._protocol == PROTOCOL_DIANHUI:
                spin.setRange(-DH_SETPOINT_LIMIT, DH_SETPOINT_LIMIT)
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

        return group

    # --- 波形控制区 ---
    def _build_wave_area(self) -> QWidget:
        title = "波形控制（软件正弦，1ms 间隔）" if self._protocol == PROTOCOL_DIANHUI \
                else "波形控制（数字模式）"
        group = QGroupBox(title)
        v = QVBoxLayout(group)
        v.setSpacing(6)

        self._wave_widgets: list[WaveChannelWidget] = []
        for ch in range(2):
            w = WaveChannelWidget(ch, protocol_type=self._protocol)
            w.set_callbacks(self._send_wave, self._stop_wave)
            v.addWidget(w)
            self._wave_widgets.append(w)

        # 巅慧专属：双轴同步启停按钮
        if self._protocol == PROTOCOL_DIANHUI:
            line = QFrame()
            line.setFrameShape(QFrame.HLine)
            line.setStyleSheet("color: #444444;")
            v.addWidget(line)

            self._sync_wave_btn = QPushButton("▶   双轴同步发送")
            self._sync_wave_btn.setMinimumHeight(44)
            self._sync_wave_btn.setStyleSheet(self._sync_btn_style(active=False))
            self._sync_wave_btn.clicked.connect(self._on_sync_wave_toggle)
            v.addWidget(self._sync_wave_btn)

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
            if self._protocol == PROTOCOL_DIANHUI:
                self._dh = DianghuiController(port=port, baudrate=baudrate)
                self._dh.open()
                self._dh_wave_mgr = DianghuiWaveManager(self._dh)
            else:
                address = self._addr_spin.value() if self._addr_spin else 1
                self._fsc = FSCController(port=port, address=address, baudrate=baudrate)
                self._fsc.open()

            self._connect_btn.setText("断开")
            self._connect_btn.setStyleSheet(
                "QPushButton{background-color:#8b1a1a;color:white;}"
            )
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

        if self._mode_btn:
            self._mode_btn.setEnabled(False)
        if self._loop_btn:
            self._loop_btn.setEnabled(False)
        if self._analog_play_btn:
            self._analog_play_btn.setEnabled(False if not self._analog_pos_data else True)
        if self._analog_stop_btn:
            self._analog_stop_btn.setEnabled(False)
        # 重置同步按钮状态
        self._sync_wave_active = False
        if self._sync_wave_btn:
            self._sync_wave_btn.setText("▶   双轴同步发送")
            self._sync_wave_btn.setStyleSheet(self._sync_btn_style(active=False))
        if self._dh_status_panel:
            self._dh_status_panel.set_unknown()

        self._status_bar.showMessage("未连接")

    # ================================================================
    # 连接后自动初始化
    # ================================================================

    def _auto_init(self):
        if self._protocol == PROTOCOL_DIANHUI:
            self._auto_init_dianghui()
        else:
            self._auto_init_fsc()

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

    def _on_load_analog_pos(self):
        """加载模拟位置 CSV 文件（每行一个数值）并解析。"""
        path, _ = QFileDialog.getOpenFileName(
            self, "加载模拟位置文件",
            os.path.join(os.path.dirname(__file__), "analog_positions"),
            "CSV 文件 (*.csv);;所有文件 (*)",
        )
        if not path:
            return
        try:
            positions = []
            channel   = 0
            with open(path, newline="", encoding="utf-8") as f:
                for row in csv.reader(f):
                    for cell in row:
                        cell = cell.strip()
                        if cell and not cell.startswith("#"):
                            positions.append(
                                max(-32768, min(32767, round(float(cell))))
                            )
            if not positions:
                raise ValueError("位置列表为空")
            self._analog_pos_data    = positions
            self._analog_pos_channel = channel
            self._analog_pos_file    = path
            lbl = f"通道{channel}: {os.path.basename(path)} ({len(positions)} 点)"
            if self._analog_file_lbl:
                self._analog_file_lbl.setText(lbl)
                self._analog_file_lbl.setStyleSheet("color: #00cc44;")
            if self._analog_play_btn:
                self._analog_play_btn.setEnabled(True)
        except Exception as exc:
            self._show_error(f"加载文件失败: {exc}")

    def _on_play_analog_pos(self):
        if not (self._dh and self._dh_wave_mgr):
            self._show_error("串口未连接")
            return
        if not self._analog_pos_data:
            self._show_error("请先加载位置文件")
            return
        ch = self._analog_pos_channel
        if ch == 0:
            self._dh_wave_mgr.start_wave_x(self._analog_pos_data)
        else:
            self._dh_wave_mgr.start_wave_y(self._analog_pos_data)
        if self._analog_stop_btn:
            self._analog_stop_btn.setEnabled(True)
        self._status_bar.showMessage(
            f"模拟位置发送中: 通道 {ch}，{len(self._analog_pos_data)} 点循环"
        )

    def _on_stop_analog_pos(self):
        if self._dh_wave_mgr:
            ch = self._analog_pos_channel
            if ch == 0:
                self._dh_wave_mgr.stop_wave_x(static_value=0)
            else:
                self._dh_wave_mgr.stop_wave_y(static_value=0)
        if self._analog_stop_btn:
            self._analog_stop_btn.setEnabled(False)
        self._status_bar.showMessage("模拟位置已停止")

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
                if self._protocol == PROTOCOL_DIANHUI
                else self._POLL_ERROR_THRESHOLD_XINMINGTIAN)

    def _poll_displacement(self):
        if self._protocol == PROTOCOL_DIANHUI:
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

    def _update_dianghui_feedback(self, status: DianghuiStatus):
        lcd0: LcdDisplay = self._lcd_0
        lcd1: LcdDisplay = self._lcd_1
        lcd0.set_int_value(status.x_feedback)
        lcd1.set_int_value(status.y_feedback)
        self._displacement[0] = float(status.x_feedback)
        self._displacement[1] = float(status.y_feedback)
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
            if self._protocol == PROTOCOL_DIANHUI:
                if not self._dh:
                    self._show_error("串口未连接")
                    return
                ival = round(val)
                # 另一轴始终从各自的位移输入框读目标值，避免用到正弦瞬时值
                other_ch = 1 - channel
                other_spin: QDoubleSpinBox = getattr(self, f"_disp_spin_{other_ch}")
                other_ival = round(other_spin.value())
                # 同步波形管理器的静态值（不中断另一轴正在运行的波形）
                if self._dh_wave_mgr:
                    if channel == 0:
                        self._dh_wave_mgr.update_static_x(ival)
                    else:
                        self._dh_wave_mgr.update_static_y(ival)
                # 直接发一帧立即到达目标（若波形管理器在运行会在1ms内接管）
                if channel == 0:
                    self._dh.send_control(x_phys=ival, y_phys=other_ival)
                else:
                    self._dh.send_control(x_phys=other_ival, y_phys=ival)
            else:
                if not self._fsc:
                    self._show_error("串口未连接")
                    return
                self._fsc.send_displacement(channel=channel, displacement=val)

            self._wave_widgets[channel].update_last_sent(val)
            self._status_bar.showMessage(f"通道 {channel} 位移已发送: {val:.3f}")
        except _COMM_ERRORS as exc:
            self._show_error(f"发送位移失败: {exc}")

    # ================================================================
    # 波形控制（由 WaveChannelWidget 回调）
    # ================================================================

    def _send_wave(self, channel: int, wave_type: int,
                   peak_peak: float, frequency: float, offset: float):
        try:
            if self._protocol == PROTOCOL_DIANHUI:
                if not (self._dh and self._dh_wave_mgr):
                    self._show_error("串口未连接")
                    return
                # 根据所选波形类型生成设定值序列
                if wave_type == DH_WAVE_TRIANGLE:
                    pts      = generate_triangle_setpoints(peak_peak, frequency, offset)
                    type_str = "三角波"
                else:
                    pts      = generate_sine_setpoints(peak_peak, frequency, offset)
                    type_str = "正弦波"
                if channel == 0:
                    self._dh_wave_mgr.start_wave_x(pts)
                else:
                    self._dh_wave_mgr.start_wave_y(pts)
                self._status_bar.showMessage(
                    f"通道 {channel} {type_str}: 峰峰={peak_peak}  频率={frequency}Hz"
                    f"  偏置={offset}  周期点数={len(pts)}"
                    f"  采样间隔={DH_WAVE_INTERVAL_MS}ms"
                )
            else:
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
        try:
            if self._protocol == PROTOCOL_DIANHUI:
                if not (self._dh and self._dh_wave_mgr):
                    self._show_error("串口未连接")
                    return
                # 从位移控制面板读当前目标值（而非历史 last_sent）
                target_spin: QDoubleSpinBox = getattr(self, f"_disp_spin_{channel}")
                ival = round(target_spin.value())
                other_ch = 1 - channel
                other_spin: QDoubleSpinBox = getattr(self, f"_disp_spin_{other_ch}")
                other_ival = round(other_spin.value())
                # 停止该轴波形，设回目标静态值
                if channel == 0:
                    self._dh_wave_mgr.stop_wave_x(static_value=ival)
                else:
                    self._dh_wave_mgr.stop_wave_y(static_value=ival)
                # 立即发一帧使设备快速到达目标位置（闭环）
                if channel == 0:
                    self._dh.send_control(x_phys=ival, y_phys=other_ival)
                else:
                    self._dh.send_control(x_phys=other_ival, y_phys=ival)
                self._wave_widgets[channel].update_last_sent(float(ival))
                self._status_bar.showMessage(
                    f"通道 {channel} 波形已停止，已返回目标位置 {ival} counts（闭环）"
                )
            else:
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

        default_dir = os.path.join(os.path.dirname(__file__), "recordings")
        os.makedirs(default_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_path = os.path.join(default_dir, f"fsc_record_{ts}.csv")

        path, _ = QFileDialog.getSaveFileName(
            self, "保存位置记录数据", default_path,
            "CSV 文件 (*.csv);;所有文件 (*)"
        )
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
    # 双轴同步启停（巅慧专用）
    # ================================================================

    @staticmethod
    def _sync_btn_style(active: bool) -> str:
        if active:
            return (
                "QPushButton{background-color:#8b1a1a;color:white;font-size:16px;"
                "font-weight:bold;border-radius:6px;padding:6px 0;}"
                "QPushButton:hover{background-color:#b52424;}"
                "QPushButton:disabled{background-color:#555;color:#999;}"
            )
        else:
            return (
                "QPushButton{background-color:#1a5c2e;color:white;font-size:16px;"
                "font-weight:bold;border-radius:6px;padding:6px 0;}"
                "QPushButton:hover{background-color:#247a3e;}"
                "QPushButton:disabled{background-color:#555;color:#999;}"
            )

    def _on_sync_wave_toggle(self):
        """双轴同步启停：一键同时对两个通道执行发送或停止。"""
        if not self._sync_wave_active:
            self._sync_start_all()
        else:
            self._sync_stop_all()

    def _sync_start_all(self):
        """同时启动两个通道的正弦波形。"""
        if not (self._dh and self._dh_wave_mgr):
            self._show_error("串口未连接")
            return
        success = True
        for ch in range(2):
            w = self._wave_widgets[ch]
            try:
                self._send_wave(
                    channel   = ch,
                    wave_type = w.get_wave_type(),
                    peak_peak = w.get_peak_peak(),
                    frequency = w.get_frequency(),
                    offset    = w.get_offset(),
                )
            except Exception as exc:
                self._show_error(f"通道 {ch} 启动波形失败: {exc}")
                success = False
        if success:
            self._sync_wave_active = True
            if self._sync_wave_btn:
                self._sync_wave_btn.setText("■   双轴同步停止")
                self._sync_wave_btn.setStyleSheet(self._sync_btn_style(active=True))
            self._status_bar.showMessage("双轴波形已同步启动")

    def _sync_stop_all(self):
        """同时停止两个通道的波形，各自返回位移面板的目标位置。"""
        for ch in range(2):
            self._stop_wave(channel=ch, displacement=0)  # displacement 在巅慧下从spinbox读
        self._sync_wave_active = False
        if self._sync_wave_btn:
            self._sync_wave_btn.setText("▶   双轴同步发送")
            self._sync_wave_btn.setStyleSheet(self._sync_btn_style(active=False))
        self._status_bar.showMessage("双轴波形已同步停止，已返回目标位置")

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
            if self._protocol == PROTOCOL_DIANHUI:
                if not self._dh:
                    raise RuntimeError("巅慧串口未连接")
            else:
                if not self._fsc:
                    raise RuntimeError("芯明天串口未连接")

        # --- set_signal_mode ---
        def h_set_signal_mode(p: dict):
            _require()
            if self._protocol == PROTOCOL_DIANHUI:
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
            if self._protocol == PROTOCOL_DIANHUI:
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
            if self._protocol == PROTOCOL_DIANHUI:
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
            if self._protocol == PROTOCOL_DIANHUI:
                wave_str = str(p.get("wave_type", "sine")).lower()
                if wave_str == "triangle":
                    pts      = generate_triangle_setpoints(peak_peak, frequency, offset)
                    type_str = "三角波"
                else:
                    pts      = generate_sine_setpoints(peak_peak, frequency, offset)
                    type_str = "正弦波"
                if ch == 0:
                    self._dh_wave_mgr.start_wave_x(pts)
                else:
                    self._dh_wave_mgr.start_wave_y(pts)
                self._sig_status_msg.emit(
                    f"[远程] 通道 {ch} {type_str}: 峰峰={peak_peak} 频率={frequency}Hz"
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
            if self._protocol == PROTOCOL_DIANHUI:
                ival = round(disp)
                if ch == 0:
                    self._dh_wave_mgr.stop_wave_x(ival)
                else:
                    self._dh_wave_mgr.stop_wave_y(ival)
            else:
                self._fsc.send_displacement(channel=ch, displacement=disp)
            self._sig_disp_sent.emit(ch, disp)
            self._sig_status_msg.emit(f"[远程] 通道 {ch} 波形已停止 disp={disp:.3f}")

        # --- 巅慧专属: send_selfcheck ---
        def h_send_selfcheck(p: dict):
            _require()
            if self._protocol != PROTOCOL_DIANHUI:
                raise RuntimeError("仅巅慧协议支持 send_selfcheck")
            st = self._dh.send_selfcheck()
            return "ok" if (st and not st.selfcheck_error) else "selfcheck_error"

        # --- 巅慧专属: read_status ---
        def h_read_status(p: dict):
            _require()
            if self._protocol != PROTOCOL_DIANHUI:
                raise RuntimeError("仅巅慧协议支持 read_status")
            st = (self._dh_wave_mgr.last_status
                  if (self._dh_wave_mgr and self._dh_wave_mgr.is_running)
                  else self._dh.send_readback())
            if st is None:
                raise RuntimeError("回读失败")
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
