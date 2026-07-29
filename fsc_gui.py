"""
快反控制盒上位机主窗口
"""

import configparser
import os
from typing import Optional

import serial
import serial.tools.list_ports
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
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

from fsc_protocol import FSCController, FSCError, WaveType
from tcp_server import FSCTcpServer

CONFIG_PATH = os.path.join(os.path.dirname(__file__), "config.ini")

WAVE_TYPES = {
    "正弦波": WaveType.SINE,
    "方波": WaveType.SQUARE,
    "三角波": WaveType.TRIANGLE,
    "锯齿波": WaveType.SAWTOOTH,
}

COMMON_FREQS = [20, 80, 120, 140, 160]

# 所有需要捕获的设备通信异常（串口断连、帧校验失败、断言等）
_COMM_ERRORS = (FSCError, AssertionError, serial.SerialException, OSError)


# ---------------------------------------------------------------------------
# 七段码显示控件（QLCDNumber 封装）
# ---------------------------------------------------------------------------

class LcdDisplay(QWidget):
    """黑底白字七段码显示，支持正负号和小数。"""

    def __init__(self, label_text: str, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        title = QLabel(label_text)
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont("Microsoft YaHei", 12, QFont.Bold))
        layout.addWidget(title)

        self._lcd = QLCDNumber(self)
        self._lcd.setDigitCount(9)
        self._lcd.setSegmentStyle(QLCDNumber.Flat)
        self._lcd.setSmallDecimalPoint(False)
        self._lcd.setMinimumHeight(70)
        self._lcd.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self._lcd.setStyleSheet(
            "QLCDNumber {"
            "  background-color: #000000;"
            "  color: #ffffff;"
            "  border: 2px solid #444444;"
            "  border-radius: 4px;"
            "}"
        )
        self._lcd.display("-.--")
        layout.addWidget(self._lcd)

        unit = QLabel("mrad")
        unit.setAlignment(Qt.AlignCenter)
        unit.setStyleSheet("color: #888888; font-size: 13px;")
        layout.addWidget(unit)

    def set_value(self, value: float) -> None:
        self._lcd.display(f"{value:.3f}")

    def set_error(self) -> None:
        self._lcd.display("Err")


# ---------------------------------------------------------------------------
# 单通道波形控制区
# ---------------------------------------------------------------------------

class WaveChannelWidget(QGroupBox):
    """单通道波形参数设置与发送控件。"""

    def __init__(self, channel: int, parent=None):
        super().__init__(f"通道 {channel} 波形控制", parent)
        self._channel = channel
        self._current_displacement: float = 0.0  # 设备轮询读回值（仅供参考）
        self._last_sent: float = 0.0             # 最后一次 send_displacement 发送的设定值
        self._send_wave_cb = None
        self._stop_wave_cb = None
        self._build_ui()

    def _build_ui(self):
        outer = QVBoxLayout(self)
        outer.setSpacing(8)
        outer.setContentsMargins(10, 14, 10, 10)

        # ---- 参数区（统一竖列对齐） ----
        grid = QGridLayout()
        grid.setSpacing(6)
        grid.setColumnMinimumWidth(1, 90)

        # 行 0 — 波形类型
        grid.addWidget(QLabel("波形:"), 0, 0)
        self._wave_combo = QComboBox()
        self._wave_combo.addItems(list(WAVE_TYPES.keys()))
        self._wave_combo.setMinimumWidth(90)
        grid.addWidget(self._wave_combo, 0, 1)

        # 行 1 — 峰峰值
        grid.addWidget(QLabel("峰峰值:"), 1, 0)
        self._pp_spin = QDoubleSpinBox()
        self._pp_spin.setRange(0.01, 200.0)
        self._pp_spin.setDecimals(3)
        self._pp_spin.setSingleStep(0.01)
        self._pp_spin.setValue(0.65)
        self._pp_spin.setMinimumWidth(90)
        grid.addWidget(self._pp_spin, 1, 1)

        # 行 2 — 频率 + 快捷按钮
        grid.addWidget(QLabel("频率(Hz):"), 2, 0)
        self._freq_spin = QDoubleSpinBox()
        self._freq_spin.setRange(0.1, 10000.0)
        self._freq_spin.setDecimals(1)
        self._freq_spin.setSingleStep(1.0)
        self._freq_spin.setValue(20.0)
        self._freq_spin.setMinimumWidth(90)
        grid.addWidget(self._freq_spin, 2, 1)

        freq_btn_layout = QHBoxLayout()
        freq_btn_layout.setSpacing(5)
        for f in COMMON_FREQS:
            btn = QPushButton(str(f))
            btn.setFixedWidth(42)
            btn.clicked.connect(lambda checked, val=f: self._freq_spin.setValue(val))
            freq_btn_layout.addWidget(btn)
        freq_btn_layout.addStretch()
        grid.addLayout(freq_btn_layout, 2, 2)

        # 行 3 — 偏置 + 填充按钮
        grid.addWidget(QLabel("偏置:"), 3, 0)
        self._offset_spin = QDoubleSpinBox()
        self._offset_spin.setRange(-200.0, 200.0)
        self._offset_spin.setDecimals(3)
        self._offset_spin.setSingleStep(0.1)
        self._offset_spin.setValue(0.0)
        self._offset_spin.setMinimumWidth(90)
        grid.addWidget(self._offset_spin, 3, 1)

        fill_btn = QPushButton("填充当前位移")
        fill_btn.setToolTip("将当前通道的位移读数填入偏置")
        fill_btn.clicked.connect(self._fill_offset)
        grid.addWidget(fill_btn, 3, 2)

        outer.addLayout(grid)
        outer.addStretch()

        # ---- 按钮行（沉底） ----
        btn_row = QHBoxLayout()
        btn_row.addStretch()

        send_btn = QPushButton("发送波形")
        send_btn.setMinimumWidth(100)
        send_btn.setStyleSheet(
            "QPushButton { background-color: #1a6b2a; color: white; font-weight: bold; padding: 5px 14px; }"
            "QPushButton:hover { background-color: #238c38; }"
            "QPushButton:disabled { background-color: #555; color: #999; }"
        )
        send_btn.clicked.connect(self._on_send_wave)
        btn_row.addWidget(send_btn)

        stop_btn = QPushButton("停止波形")
        stop_btn.setMinimumWidth(100)
        stop_btn.setStyleSheet(
            "QPushButton { background-color: #8b1a1a; color: white; font-weight: bold; padding: 5px 14px; }"
            "QPushButton:hover { background-color: #b52424; }"
            "QPushButton:disabled { background-color: #555; color: #999; }"
        )
        stop_btn.clicked.connect(self._on_stop_wave)
        btn_row.addWidget(stop_btn)

        outer.addLayout(btn_row)

    def set_callbacks(self, send_cb, stop_cb):
        """注册发送波形和停止波形回调。"""
        self._send_wave_cb = send_cb
        self._stop_wave_cb = stop_cb

    def update_displacement(self, value: float):
        """更新轮询读回的位移（内部记录，不用于填充/停止）。"""
        self._current_displacement = value

    def update_last_sent(self, value: float):
        """更新最后一次 send_displacement 发送的设定值。"""
        self._last_sent = value

    def _fill_offset(self):
        """将最后一次发送的位移设定值填入偏置输入框。"""
        self._offset_spin.setValue(self._last_sent)

    def _on_send_wave(self):
        if self._send_wave_cb:
            wave_type = WAVE_TYPES[self._wave_combo.currentText()]
            self._send_wave_cb(
                channel=self._channel,
                wave_type=wave_type,
                peak_peak=self._pp_spin.value(),
                frequency=self._freq_spin.value(),
                offset=self._offset_spin.value(),
            )

    def _on_stop_wave(self):
        """停止波形，回到最后一次发送的位移设定值。"""
        if self._stop_wave_cb:
            self._stop_wave_cb(
                channel=self._channel,
                displacement=self._last_sent,
            )


# ---------------------------------------------------------------------------
# 主窗口
# ---------------------------------------------------------------------------

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self._fsc: Optional[FSCController] = None
        self._config = configparser.ConfigParser()
        self._load_config()

        self._is_analog = False   # 当前信号模式：False=数字，True=模拟
        self._is_closed_loop = True   # 当前开闭环状态：True=闭环，False=开环
        self._displacement = [0.0, 0.0]
        self._poll_error_count = 0   # 连续读取失败次数，超过阈值时自动断连

        self._tcp_server = FSCTcpServer(port=self._tcp_port())
        self._tcp_server.set_log_callback(self._append_tcp_log)
        self._register_tcp_handlers()

        self._build_ui()
        self._refresh_ports()

        # 定时读取位移（1 秒 1 次）
        self._disp_timer = QTimer(self)
        self._disp_timer.setInterval(1000)
        self._disp_timer.timeout.connect(self._poll_displacement)
        self._disp_timer.start()

        if self._config.getboolean("TCP", "auto_start", fallback=True):
            self._start_tcp()

    # ------------------------------------------------------------------
    # 配置文件
    # ------------------------------------------------------------------

    def _load_config(self):
        if os.path.exists(CONFIG_PATH):
            self._config.read(CONFIG_PATH, encoding="utf-8")
        if not self._config.has_section("Serial"):
            self._config.add_section("Serial")
        if not self._config.has_section("TCP"):
            self._config.add_section("TCP")

    def _save_config(self):
        with open(CONFIG_PATH, "w", encoding="utf-8") as f:
            self._config.write(f)

    def _tcp_port(self) -> int:
        return self._config.getint("TCP", "port", fallback=10014)

    def _tcp_ip(self) -> str:
        return self._config.get("TCP", "ip", fallback="0.0.0.0")

    # ------------------------------------------------------------------
    # UI 构建
    # ------------------------------------------------------------------

    def _build_ui(self):
        self.setWindowTitle("快反控制盒上位机")
        self.setMinimumWidth(780)

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setSpacing(8)
        main_layout.setContentsMargins(10, 10, 10, 10)

        main_layout.addWidget(self._build_connection_bar())
        main_layout.addWidget(self._build_mode_bar())
        main_layout.addWidget(self._build_displacement_area())
        main_layout.addWidget(self._build_wave_area())
        main_layout.addWidget(self._build_tcp_log_area(), stretch=1)

        self._status_bar = QStatusBar()
        self.setStatusBar(self._status_bar)
        self._status_bar.showMessage("未连接")

    # --- 串口连接栏 ---
    def _build_connection_bar(self) -> QWidget:
        bar = QGroupBox("连接")
        layout = QHBoxLayout(bar)
        layout.setSpacing(8)

        layout.addWidget(QLabel("串口:"))
        self._port_combo = QComboBox()
        self._port_combo.setMinimumWidth(100)
        layout.addWidget(self._port_combo)

        refresh_btn = QPushButton("刷新")
        refresh_btn.setFixedWidth(50)
        refresh_btn.clicked.connect(self._refresh_ports)
        layout.addWidget(refresh_btn)

        layout.addWidget(QLabel("地址:"))
        self._addr_spin = QSpinBox()
        self._addr_spin.setRange(1, 255)
        self._addr_spin.setValue(self._config.getint("Serial", "address", fallback=1))
        self._addr_spin.setFixedWidth(55)
        layout.addWidget(self._addr_spin)

        self._connect_btn = QPushButton("连接")
        self._connect_btn.setFixedWidth(60)
        self._connect_btn.setCheckable(True)
        self._connect_btn.clicked.connect(self._on_connect_toggle)
        layout.addWidget(self._connect_btn)

        layout.addStretch()
        return bar

    # --- 模式切换栏 ---
    def _build_mode_bar(self) -> QWidget:
        bar = QWidget()
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)
        layout.addStretch()

        # 信号模式切换（数字 / 模拟）
        self._mode_btn = QPushButton("当前模式: 数字  →  切换为模拟")
        self._mode_btn.setFixedHeight(36)
        self._mode_btn.setMinimumWidth(270)
        self._mode_btn.setStyleSheet(
            "QPushButton {"
            "  background-color: #1e3a6e;"
            "  color: white;"
            "  font-size: 15px;"
            "  font-weight: bold;"
            "  border-radius: 6px;"
            "  padding: 0 16px;"
            "}"
            "QPushButton:hover { background-color: #2a52a0; }"
            "QPushButton:disabled { background-color: #555; color: #999; }"
        )
        self._mode_btn.setEnabled(False)
        self._mode_btn.clicked.connect(self._on_mode_toggle)
        layout.addWidget(self._mode_btn)

        # 开闭环切换
        self._loop_btn = QPushButton("当前状态: 闭环  →  切换为开环")
        self._loop_btn.setFixedHeight(36)
        self._loop_btn.setMinimumWidth(270)
        self._loop_btn.setStyleSheet(
            "QPushButton {"
            "  background-color: #1a5c1a;"
            "  color: white;"
            "  font-size: 15px;"
            "  font-weight: bold;"
            "  border-radius: 6px;"
            "  padding: 0 16px;"
            "}"
            "QPushButton:hover { background-color: #247a24; }"
            "QPushButton:disabled { background-color: #555; color: #999; }"
        )
        self._loop_btn.setEnabled(False)
        self._loop_btn.clicked.connect(self._on_loop_toggle)
        layout.addWidget(self._loop_btn)

        layout.addStretch()
        return bar

    # --- 位移显示 + 输入区 ---
    def _build_displacement_area(self) -> QWidget:
        group = QGroupBox("位移控制")
        layout = QHBoxLayout(group)
        layout.setSpacing(20)

        for ch in range(2):
            ch_widget = QWidget()
            ch_layout = QVBoxLayout(ch_widget)
            ch_layout.setSpacing(6)

            lcd = LcdDisplay(f"通道 {ch}")
            setattr(self, f"_lcd_{ch}", lcd)
            ch_layout.addWidget(lcd)

            ctrl_layout = QHBoxLayout()
            ctrl_layout.addWidget(QLabel("目标位移:"))

            disp_spin = QDoubleSpinBox()
            disp_spin.setRange(-100.0, 100.0)
            disp_spin.setDecimals(3)
            disp_spin.setSingleStep(0.1)
            disp_spin.setValue(0.0)
            disp_spin.setFixedWidth(100)
            setattr(self, f"_disp_spin_{ch}", disp_spin)
            ctrl_layout.addWidget(disp_spin)

            send_btn = QPushButton("发送")
            send_btn.setFixedWidth(55)
            send_btn.clicked.connect(lambda checked, c=ch: self._send_displacement(c))
            ctrl_layout.addWidget(send_btn)
            ctrl_layout.addStretch()

            ch_layout.addLayout(ctrl_layout)
            layout.addWidget(ch_widget)

        return group

    # --- 波形控制区 ---
    def _build_wave_area(self) -> QWidget:
        group = QGroupBox("波形控制（数字模式）")
        layout = QVBoxLayout(group)
        layout.setSpacing(6)

        self._wave_widgets = []
        for ch in range(2):
            w = WaveChannelWidget(ch)
            w.set_callbacks(self._send_wave, self._stop_wave)
            layout.addWidget(w)
            self._wave_widgets.append(w)

        return group

    # --- TCP 日志区 ---
    def _build_tcp_log_area(self) -> QWidget:
        group = QGroupBox("TCP 日志")
        layout = QVBoxLayout(group)
        layout.setContentsMargins(6, 6, 6, 6)

        self._tcp_log = QTextEdit()
        self._tcp_log.setReadOnly(True)
        self._tcp_log.setStyleSheet(
            "QTextEdit { background-color: #1e1e1e; color: #d4d4d4;"
            " font-family: Consolas, monospace; font-size: 13px; }"
        )
        layout.addWidget(self._tcp_log)
        return group

    # ------------------------------------------------------------------
    # 串口连接
    # ------------------------------------------------------------------

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
        port = self._port_combo.currentText()
        address = self._addr_spin.value()
        if not port:
            self._show_error("请先选择串口")
            self._connect_btn.setChecked(False)
            return
        try:
            self._fsc = FSCController(port=port, address=address)
            self._fsc.open()
            self._connect_btn.setText("断开")
            self._connect_btn.setStyleSheet("QPushButton { background-color: #8b1a1a; color: white; }")
            self._mode_btn.setEnabled(True)
            self._loop_btn.setEnabled(True)
            self._poll_error_count = 0
            self._status_bar.showMessage(f"已连接: {port}  地址: {address}")
            self._config.set("Serial", "port", port)
            self._config.set("Serial", "address", str(address))
            self._save_config()
            self._auto_init()
        except Exception as exc:
            self._fsc = None
            self._connect_btn.setChecked(False)
            self._show_error(f"连接失败: {exc}")

    def _do_disconnect(self):
        if self._fsc:
            try:
                self._fsc.close()
            except Exception:
                pass
            self._fsc = None
        self._connect_btn.setText("连接")
        self._connect_btn.setStyleSheet("")
        self._connect_btn.setChecked(False)
        self._mode_btn.setEnabled(False)
        self._loop_btn.setEnabled(False)
        self._status_bar.showMessage("未连接")

    # ------------------------------------------------------------------
    # 连接后自动初始化
    # ------------------------------------------------------------------

    def _auto_init(self):
        """串口连接成功后自动初始化：设置数字模式并闭环，逐步确认。"""
        steps = []

        # Step 1 — 设置数字模式
        for ch in range(2):
            try:
                self._fsc.set_signal_mode(channel=ch, analog=False)
            except _COMM_ERRORS as exc:
                steps.append(f"通道 {ch} 设置数字模式失败: {exc}")

        # Step 2 — 确认数字模式
        for ch in range(2):
            try:
                is_analog = self._fsc.get_signal_mode(channel=ch)
                if is_analog:
                    steps.append(f"通道 {ch} 确认模式异常: 读回仍为模拟模式")
            except _COMM_ERRORS as exc:
                steps.append(f"通道 {ch} 读取信号模式失败: {exc}")

        # Step 3 — 设置闭环
        for ch in range(2):
            try:
                self._fsc.set_loop_mode(channel=ch, closed=True)
            except _COMM_ERRORS as exc:
                steps.append(f"通道 {ch} 设置闭环失败: {exc}")

        # Step 4 — 确认闭环
        for ch in range(2):
            try:
                is_closed = self._fsc.get_loop_mode(channel=ch)
                if not is_closed:
                    steps.append(f"通道 {ch} 确认闭环异常: 读回仍为开环")
            except _COMM_ERRORS as exc:
                steps.append(f"通道 {ch} 读取开闭环状态失败: {exc}")

        # 更新内部状态与按钮
        self._is_analog = False
        self._is_closed_loop = True
        self._update_mode_button()
        self._update_loop_button()

        if steps:
            self._show_error("初始化警告（操作已继续）:\n" + "\n".join(steps))
        else:
            self._status_bar.showMessage(
                self._status_bar.currentMessage() + "  |  初始化完成: 数字模式 + 闭环"
            )

    # ------------------------------------------------------------------
    # 模式切换
    # ------------------------------------------------------------------

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
        if self._is_analog:
            self._mode_btn.setText("当前模式: 模拟  →  切换为数字")
            self._mode_btn.setStyleSheet(
                "QPushButton {"
                "  background-color: #6e3a1e;"
                "  color: white; font-size: 15px; font-weight: bold;"
                "  border-radius: 6px; padding: 0 16px;"
                "}"
                "QPushButton:hover { background-color: #a05228; }"
                "QPushButton:disabled { background-color: #555; color: #999; }"
            )
        else:
            self._mode_btn.setText("当前模式: 数字  →  切换为模拟")
            self._mode_btn.setStyleSheet(
                "QPushButton {"
                "  background-color: #1e3a6e;"
                "  color: white; font-size: 15px; font-weight: bold;"
                "  border-radius: 6px; padding: 0 16px;"
                "}"
                "QPushButton:hover { background-color: #2a52a0; }"
                "QPushButton:disabled { background-color: #555; color: #999; }"
            )

    # ------------------------------------------------------------------
    # 开闭环切换
    # ------------------------------------------------------------------

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
        if self._is_closed_loop:
            self._loop_btn.setText("当前状态: 闭环  →  切换为开环")
            self._loop_btn.setStyleSheet(
                "QPushButton {"
                "  background-color: #1a5c1a;"
                "  color: white; font-size: 15px; font-weight: bold;"
                "  border-radius: 6px; padding: 0 16px;"
                "}"
                "QPushButton:hover { background-color: #247a24; }"
                "QPushButton:disabled { background-color: #555; color: #999; }"
            )
        else:
            self._loop_btn.setText("当前状态: 开环  →  切换为闭环")
            self._loop_btn.setStyleSheet(
                "QPushButton {"
                "  background-color: #7a6010;"
                "  color: white; font-size: 15px; font-weight: bold;"
                "  border-radius: 6px; padding: 0 16px;"
                "}"
                "QPushButton:hover { background-color: #a07e18; }"
                "QPushButton:disabled { background-color: #555; color: #999; }"
            )

    # ------------------------------------------------------------------
    # 位移读取（定时）
    # ------------------------------------------------------------------

    _POLL_ERROR_THRESHOLD = 3  # 连续失败超过此次数触发自动断连

    def _poll_displacement(self):
        if not self._fsc:
            return
        any_error = False
        for ch in range(2):
            lcd: LcdDisplay = getattr(self, f"_lcd_{ch}")
            try:
                val = self._fsc.read_displacement(channel=ch)
                self._displacement[ch] = val
                lcd.set_value(val)
                self._wave_widgets[ch].update_displacement(val)
            except _COMM_ERRORS:
                lcd.set_error()
                any_error = True
            except Exception:
                lcd.set_error()
                any_error = True

        if any_error:
            self._poll_error_count += 1
            if self._poll_error_count >= self._POLL_ERROR_THRESHOLD:
                self._on_device_lost()
        else:
            self._poll_error_count = 0

    def _on_device_lost(self):
        """连续读取失败超过阈值，判定设备断连，自动清理状态。"""
        self._poll_error_count = 0
        self._do_disconnect()
        self._status_bar.showMessage("设备断连：串口通信失败，已自动断开")

    # ------------------------------------------------------------------
    # 位移发送
    # ------------------------------------------------------------------

    def _send_displacement(self, channel: int):
        if not self._fsc:
            self._show_error("串口未连接")
            return
        spin: QDoubleSpinBox = getattr(self, f"_disp_spin_{channel}")
        val = spin.value()
        try:
            self._fsc.send_displacement(channel=channel, displacement=val)
            self._wave_widgets[channel].update_last_sent(val)
            self._status_bar.showMessage(f"通道 {channel} 位移已发送: {val:.3f} mrad")
        except _COMM_ERRORS as exc:
            self._show_error(f"发送位移失败: {exc}")

    # ------------------------------------------------------------------
    # 波形控制（由 WaveChannelWidget 回调）
    # ------------------------------------------------------------------

    def _send_wave(self, channel: int, wave_type: int, peak_peak: float,
                   frequency: float, offset: float):
        if not self._fsc:
            self._show_error("串口未连接")
            return
        try:
            self._fsc.send_wave_displacement(
                channel=channel,
                wave_type=wave_type,
                peak_peak=peak_peak,
                frequency=frequency,
                offset=offset,
            )
            self._status_bar.showMessage(
                f"通道 {channel} 波形已发送: 峰峰值={peak_peak} 频率={frequency}Hz 偏置={offset}"
            )
        except _COMM_ERRORS as exc:
            self._show_error(f"发送波形失败: {exc}")

    def _stop_wave(self, channel: int, displacement: float):
        """停止波形：使用 send_displacement 发送当前位移，不调用 stop_wave 以保留偏置。"""
        if not self._fsc:
            self._show_error("串口未连接")
            return
        try:
            self._fsc.send_displacement(channel=channel, displacement=displacement)
            self._status_bar.showMessage(f"通道 {channel} 波形已停止（位移={displacement:.3f}）")
        except _COMM_ERRORS as exc:
            self._show_error(f"停止波形失败: {exc}")

    # ------------------------------------------------------------------
    # TCP 服务端
    # ------------------------------------------------------------------

    def _start_tcp(self):
        ip = self._tcp_ip()
        port = self._tcp_port()
        self._tcp_server._host = ip
        self._tcp_server._port = port
        self._tcp_server.start()

    def _stop_tcp(self):
        self._tcp_server.stop()

    def _register_tcp_handlers(self):
        """向 TCP 服务端注册所有命令处理回调。"""

        def require_fsc():
            if not self._fsc:
                raise RuntimeError("串口未连接")

        def handle_set_signal_mode(p: dict):
            require_fsc()
            channel = int(p.get("channel", 0))
            analog = bool(p.get("analog", False))
            self._fsc.set_signal_mode(channel=channel, analog=analog)

        def handle_send_displacement(p: dict):
            require_fsc()
            channel = int(p.get("channel", 0))
            displacement = float(p.get("displacement", 0.0))
            self._fsc.send_displacement(channel=channel, displacement=displacement)

        def handle_read_displacement(p: dict):
            require_fsc()
            channel = int(p.get("channel", 0))
            return self._fsc.read_displacement(channel=channel)

        def handle_send_wave(p: dict):
            require_fsc()
            channel = int(p.get("channel", 0))
            wave_str = str(p.get("wave_type", "sine")).lower()
            wave_map = {
                "sine": WaveType.SINE,
                "square": WaveType.SQUARE,
                "triangle": WaveType.TRIANGLE,
                "sawtooth": WaveType.SAWTOOTH,
            }
            if wave_str not in wave_map:
                raise ValueError(f"不支持的波形类型: {wave_str}，可选: sine/square/triangle/sawtooth")
            self._fsc.send_wave_displacement(
                channel=channel,
                wave_type=wave_map[wave_str],
                peak_peak=float(p.get("peak_peak", 0.65)),
                frequency=float(p.get("frequency", 20.0)),
                offset=float(p.get("offset", 0.0)),
            )

        def handle_stop_wave(p: dict):
            require_fsc()
            channel = int(p.get("channel", 0))
            displacement = float(p.get("displacement", 0.0))
            self._fsc.send_displacement(channel=channel, displacement=displacement)

        self._tcp_server.register("set_signal_mode", handle_set_signal_mode)
        self._tcp_server.register("send_displacement", handle_send_displacement)
        self._tcp_server.register("read_displacement", handle_read_displacement)
        self._tcp_server.register("send_wave", handle_send_wave)
        self._tcp_server.register("stop_wave", handle_stop_wave)

    # ------------------------------------------------------------------
    # TCP 日志（线程安全通过 Qt 信号机制）
    # ------------------------------------------------------------------

    def _append_tcp_log(self, msg: str):
        from PyQt5.QtCore import QMetaObject, Q_ARG
        QMetaObject.invokeMethod(
            self._tcp_log,
            "append",
            Qt.QueuedConnection,
            Q_ARG(str, msg),
        )

    # ------------------------------------------------------------------
    # 工具
    # ------------------------------------------------------------------

    def _show_error(self, msg: str):
        QMessageBox.critical(self, "错误", msg)

    def closeEvent(self, event):
        self._disp_timer.stop()
        self._stop_tcp()
        self._do_disconnect()
        super().closeEvent(event)
