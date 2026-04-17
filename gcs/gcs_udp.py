from __future__ import annotations

import csv
import datetime as dt
import json
import queue
import socket
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

from PySide6.QtCore import QPointF, QSettings, Qt, QTimer
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QPlainTextEdit,
    QProgressBar,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

APP_STYLESHEET = """
QMainWindow, QWidget {
    background: #f4f7fb;
    color: #1f2937;
    font-size: 12px;
}
QGroupBox {
    background: #ffffff;
    border: 1px solid #d5dce8;
    border-radius: 8px;
    margin-top: 8px;
    padding: 8px;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 4px;
    color: #344054;
    font-weight: 600;
}
QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox, QPlainTextEdit {
    background: #ffffff;
    color: #1f2937;
    border: 1px solid #c7d0df;
    border-radius: 4px;
    padding: 4px;
}
QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus, QPlainTextEdit:focus {
    border: 1px solid #2563eb;
}
QPushButton {
    background: #ffffff;
    color: #1f2937;
    border: 1px solid #aeb8c8;
    border-radius: 6px;
    padding: 5px 9px;
}
QPushButton:hover {
    background: #edf3ff;
    border-color: #7aa7ff;
}
QPushButton:pressed {
    background: #dbeafe;
}
QPushButton:disabled {
    color: #8a94a6;
    background: #eef1f5;
    border-color: #d1d8e3;
}
QPushButton#primaryButton {
    color: #ffffff;
    background: #2563eb;
    border-color: #1d4ed8;
    font-weight: 600;
}
QPushButton#primaryButton:hover {
    background: #1d4ed8;
}
QPushButton#dangerButton {
    color: #b42318;
    border-color: #f5a5a0;
    background: #fff5f5;
}
QPushButton#dangerButton:hover {
    background: #fee4e2;
}
QTabWidget::pane {
    border: 1px solid #d5dce8;
    background: #ffffff;
}
QTabBar::tab {
    background: #e9eef6;
    color: #344054;
    border: 1px solid #d5dce8;
    border-bottom: none;
    border-top-left-radius: 6px;
    border-top-right-radius: 6px;
    padding: 6px 13px;
}
QTabBar::tab:selected {
    background: #ffffff;
    color: #111827;
    font-weight: 600;
}
QLabel#sectionNote {
    color: #667085;
}
QLabel#statusBadge, QLabel#statusChip, QLabel#valueBadge {
    border: 1px solid #c7d0df;
    border-radius: 6px;
    padding: 5px 8px;
    background: #ffffff;
    font-weight: 600;
}
QLabel#statusBadge[state="ok"], QLabel#statusChip[state="ok"], QLabel#valueBadge[state="ok"] {
    color: #067647;
    border-color: #75e0a7;
    background: #ecfdf3;
}
QLabel#statusBadge[state="warn"], QLabel#statusChip[state="warn"], QLabel#valueBadge[state="warn"] {
    color: #b54708;
    border-color: #fec84b;
    background: #fffaeb;
}
QLabel#statusBadge[state="error"], QLabel#statusChip[state="error"], QLabel#valueBadge[state="error"] {
    color: #b42318;
    border-color: #fda29b;
    background: #fff1f3;
}
QLabel#metricValue {
    color: #111827;
    font-family: Consolas, "Courier New", monospace;
    font-weight: 600;
}
QLabel#panelTitle {
    color: #667085;
    font-size: 11px;
    font-weight: 600;
    background: transparent;
}
QProgressBar {
    background: #ffffff;
    color: #1f2937;
    border: 1px solid #c7d0df;
    border-radius: 5px;
    min-height: 12px;
    text-align: center;
}
QProgressBar::chunk {
    background: #2563eb;
    border-radius: 4px;
}
QProgressBar[level="warn"]::chunk {
    background: #f79009;
}
QProgressBar[level="error"]::chunk {
    background: #d92d20;
}
"""


CSV_FIELDS = [
    "type",
    "time_ms",
    "radio_ok",
    "failsafe",
    "arm",
    "mode",
    "roll",
    "pitch",
    "yaw",
    "gx",
    "gy",
    "gz",
    "speed_z",
    "ch_roll",
    "ch_pitch",
    "ch_throttle",
    "ch_yaw",
    "motor1",
    "motor2",
    "motor3",
    "motor4",
    "control1",
    "control2",
    "control3",
    "control4",
    "k_roll",
    "k_pitch",
    "k_yaw",
    "k_roll_rate",
    "k_pitch_rate",
    "k_yaw_rate",
    "k_i_roll",
    "k_i_pitch",
    "k_i_yaw",
    "safety_reason",
    "motors_started",
    "imu_ready",
    "imu_cal_sys",
    "imu_cal_gyro",
    "imu_cal_accel",
    "imu_cal_mag",
    "imu_samples",
    "radio_frames",
    "radio_desync",
    "radio_invalid",
    "radio_interval_ms",
    "control_dt_ms",
    "roll_cmd",
    "pitch_cmd",
    "yaw_cmd",
    "roll_out",
    "pitch_out",
    "yaw_out",
]

PID_FIELDS = [
    "k_roll",
    "k_pitch",
    "k_yaw",
    "k_roll_rate",
    "k_pitch_rate",
    "k_yaw_rate",
    "k_i_roll",
    "k_i_pitch",
    "k_i_yaw",
]

PID_AXIS_FIELDS = {
    "Roll": ["k_roll", "k_roll_rate", "k_i_roll"],
    "Pitch": ["k_pitch", "k_pitch_rate", "k_i_pitch"],
    "Yaw": ["k_yaw", "k_yaw_rate", "k_i_yaw"],
}

DEFAULT_PID_VALUES = {
    "k_roll": 3.5,
    "k_pitch": 3.5,
    "k_yaw": 0.6,
    "k_roll_rate": 1.0,
    "k_pitch_rate": 1.0,
    "k_yaw_rate": 0.5,
    "k_i_roll": 0.0,
    "k_i_pitch": 0.0,
    "k_i_yaw": 0.0,
}

BUILTIN_PID_PRESETS = {
    "Firmware defaults": DEFAULT_PID_VALUES,
    "Zero I and D": {
        "k_roll": DEFAULT_PID_VALUES["k_roll"],
        "k_pitch": DEFAULT_PID_VALUES["k_pitch"],
        "k_yaw": DEFAULT_PID_VALUES["k_yaw"],
        "k_roll_rate": 0.0,
        "k_pitch_rate": 0.0,
        "k_yaw_rate": 0.0,
        "k_i_roll": 0.0,
        "k_i_pitch": 0.0,
        "k_i_yaw": 0.0,
    },
}

COMMAND_RESPONSE_PREFIXES = ("ACK", "ERR", "HELP")
PRESET_FILE = Path(__file__).with_name("pid_presets.json")
LOG_DIR = Path(__file__).with_name("logs")
APP_ORG = "AdvancedRoboticsProject"
APP_NAME = "UdpGcsDashboard"

DISPLAY_LABELS = {
    "radio_ok": "Radio",
    "failsafe": "Failsafe",
    "arm": "Arm",
    "mode": "Mode",
    "safety_reason": "Safety",
    "motors_started": "Motors started",
    "imu_ready": "IMU ready",
    "imu_cal_sys": "Cal sys",
    "imu_cal_gyro": "Cal gyro",
    "imu_cal_accel": "Cal accel",
    "imu_cal_mag": "Cal mag",
    "imu_samples": "IMU samples",
    "radio_frames": "Radio frames",
    "radio_desync": "Radio desync",
    "radio_invalid": "Radio invalid",
    "radio_interval_ms": "Radio interval",
    "control_dt_ms": "Control dt",
    "roll_cmd": "Roll cmd",
    "pitch_cmd": "Pitch cmd",
    "yaw_cmd": "Yaw cmd",
    "roll_out": "Roll out",
    "pitch_out": "Pitch out",
    "yaw_out": "Yaw out",
    "ch_roll": "Roll",
    "ch_pitch": "Pitch",
    "ch_throttle": "Throttle",
    "ch_yaw": "Yaw",
    "speed_z": "Speed Z",
}

PID_LABELS = {
    "k_roll": "k_roll (P)",
    "k_roll_rate": "k_roll_rate (D)",
    "k_i_roll": "k_i_roll (I)",
    "k_pitch": "k_pitch (P)",
    "k_pitch_rate": "k_pitch_rate (D)",
    "k_i_pitch": "k_i_pitch (I)",
    "k_yaw": "k_yaw (P)",
    "k_yaw_rate": "k_yaw_rate (D)",
    "k_i_yaw": "k_i_yaw (I)",
}


def parse_value(value: str) -> int | float | str:
    try:
        if "." not in value:
            return int(value)
        return float(value)
    except ValueError:
        return value


def parse_csv_line(line: str) -> dict[str, int | float | str] | None:
    parts = line.strip().split(",")
    if not parts or parts[0] != "TEL":
        return None
    if len(parts) > len(CSV_FIELDS):
        parts = parts[: len(CSV_FIELDS)]
    data = {name: parse_value(value) for name, value in zip(CSV_FIELDS, parts)}
    for field in CSV_FIELDS[len(parts) :]:
        data[field] = ""
    return data


@dataclass
class UdpEvent:
    kind: str
    message: str


class AttitudeChart(QWidget):
    def __init__(self, max_points: int = 180) -> None:
        super().__init__()
        self.max_points = max_points
        self.roll_values: list[float] = []
        self.pitch_values: list[float] = []
        self.setMinimumHeight(165)
        self.setMaximumHeight(210)

    def add_sample(self, roll: int | float | str | None, pitch: int | float | str | None) -> None:
        try:
            roll_value = float(roll)
            pitch_value = float(pitch)
        except (TypeError, ValueError):
            return

        self.roll_values.append(roll_value)
        self.pitch_values.append(pitch_value)

        if len(self.roll_values) > self.max_points:
            self.roll_values = self.roll_values[-self.max_points:]
            self.pitch_values = self.pitch_values[-self.max_points:]

        self.update()

    def clear(self) -> None:
        self.roll_values.clear()
        self.pitch_values.clear()
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        width = self.width()
        height = self.height()
        margin_left = 42
        margin_right = 14
        margin_top = 24
        margin_bottom = 28
        plot_width = max(1, width - margin_left - margin_right)
        plot_height = max(1, height - margin_top - margin_bottom)
        y_min = -45.0
        y_max = 45.0

        painter.fillRect(self.rect(), QColor("#ffffff"))
        painter.setPen(QPen(QColor("#d8d8d8"), 1))
        painter.drawRect(margin_left, margin_top, plot_width, plot_height)

        def map_y(value: float) -> float:
            value = max(y_min, min(y_max, value))
            normalized = (value - y_min) / (y_max - y_min)
            return margin_top + plot_height - (normalized * plot_height)

        def map_x(index: int, count: int) -> float:
            if count <= 1:
                return margin_left
            return margin_left + (index / (count - 1)) * plot_width

        painter.setPen(QPen(QColor("#eeeeee"), 1))
        for value in (-30, -15, 15, 30):
            y = map_y(float(value))
            painter.drawLine(QPointF(margin_left, y), QPointF(margin_left + plot_width, y))

        limit_pen = QPen(QColor("#f28b82"), 1)
        limit_pen.setStyle(Qt.DashLine)
        painter.setPen(limit_pen)
        for value in (-30, 30):
            y = map_y(float(value))
            painter.drawLine(QPointF(margin_left, y), QPointF(margin_left + plot_width, y))

        painter.setPen(QPen(QColor("#9aa0a6"), 1))
        zero_y = map_y(0.0)
        painter.drawLine(QPointF(margin_left, zero_y), QPointF(margin_left + plot_width, zero_y))

        painter.setPen(QColor("#5f6368"))
        for value in (-45, 0, 45):
            y = map_y(float(value))
            painter.drawText(4, int(y) + 4, f"{value}")
        painter.drawText(margin_left, height - 8, "time")
        painter.drawText(4, 16, "deg")

        if not self.roll_values:
            painter.setPen(QColor("#5f6368"))
            painter.drawText(self.rect(), Qt.AlignCenter, "Waiting telemetry")
            return

        def draw_series(values: list[float], color: str) -> None:
            painter.setPen(QPen(QColor(color), 2))
            for index in range(1, len(values)):
                painter.drawLine(
                    QPointF(map_x(index - 1, len(values)), map_y(values[index - 1])),
                    QPointF(map_x(index, len(values)), map_y(values[index])),
                )

        draw_series(self.roll_values, "#d93025")
        draw_series(self.pitch_values, "#1a73e8")

        painter.setPen(QColor("#202124"))
        latest_roll = self.roll_values[-1]
        latest_pitch = self.pitch_values[-1]
        roll_color = "#b3261e" if abs(latest_roll) >= 30.0 else "#d93025"
        pitch_color = "#b3261e" if abs(latest_pitch) >= 30.0 else "#1a73e8"
        painter.setPen(QColor(roll_color))
        painter.drawText(margin_left, 16, f"Roll {latest_roll:.1f}")
        painter.setPen(QColor(pitch_color))
        painter.drawText(margin_left + 92, 16, f"Pitch {latest_pitch:.1f}")
        painter.setPen(QColor("#5f6368"))
        painter.drawText(margin_left + 210, 16, "limit +/-30 deg")


class UdpLink:
    def __init__(self) -> None:
        self.sock: socket.socket | None = None
        self.remote: tuple[str, int] | None = None
        self.reader_thread: threading.Thread | None = None
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.events: queue.Queue[UdpEvent] = queue.Queue()

    @property
    def is_connected(self) -> bool:
        return self.sock is not None

    def connect(self, host: str, remote_port: int, local_port: int) -> None:
        self.disconnect()
        self.stop_event.clear()

        resolved_ip = socket.gethostbyname(host)
        self.remote = (resolved_ip, remote_port)

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", local_port))
        sock.settimeout(0.2)

        self.sock = sock
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()
        self.events.put(UdpEvent("status", f"UDP ready: local {local_port}, remote {host} ({resolved_ip}):{remote_port}"))
        self.write("HELLO")
        self.write("HEADER")

    def disconnect(self) -> None:
        self.stop_event.set()
        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=0.5)
        self.reader_thread = None

        with self.lock:
            if self.sock:
                self.sock.close()
            self.sock = None
            self.remote = None

    def write(self, command: str) -> None:
        with self.lock:
            if not self.sock or not self.remote:
                self.events.put(UdpEvent("error", "UDP is not connected"))
                return
            payload = command.strip().encode("utf-8")
            self.sock.sendto(payload, self.remote)

    def _reader_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                with self.lock:
                    sock = self.sock
                if not sock:
                    break
                data, addr = sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError as exc:
                if not self.stop_event.is_set():
                    self.events.put(UdpEvent("error", str(exc)))
                break

            line = data.decode("utf-8", errors="replace").strip()
            if line:
                self.events.put(UdpEvent("line", line))


class Dashboard(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Advanced Robotics Project - UDP GCS")
        self.resize(1280, 820)

        self.settings = QSettings(APP_ORG, APP_NAME)
        self.udp_link = UdpLink()
        self.latest_data: dict[str, int | float | str] = {}
        self.last_telemetry_time = 0.0
        self.pid_loaded_once = False
        self.telemetry_count = 0
        self.packet_rate = 0.0
        self.packet_rate_last_time = time.monotonic()
        self.packet_rate_last_count = 0
        self.csv_file = None
        self.csv_writer: csv.DictWriter | None = None
        self.csv_log_path: Path | None = None
        self.event_log_file = None
        self.event_log_writer: csv.DictWriter | None = None
        self.event_log_path: Path | None = None
        self.session_log_dir: Path | None = None
        self.last_logged_state: dict[str, int | float | str] = {}
        self.user_pid_presets = self._load_user_pid_presets()

        self.value_labels: dict[str, QLabel] = {}
        self.pid_inputs: dict[str, QDoubleSpinBox] = {}
        self.pid_send_buttons: list[QPushButton] = []
        self.pid_action_buttons: list[QPushButton] = []
        self.motor_bars: dict[str, QProgressBar] = {}
        self.status_chips: dict[str, QLabel] = {}
        self.log_paused = False
        self.paused_log_lines: list[str] = []
        self.attitude_chart = AttitudeChart()

        self._build_ui()
        self.restore_ui_settings()

        self.event_timer = QTimer(self)
        self.event_timer.timeout.connect(self.poll_events)
        self.event_timer.start(50)

        self.hello_timer = QTimer(self)
        self.hello_timer.timeout.connect(self.send_hello_if_needed)
        self.hello_timer.start(1000)

        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self.update_connection_status)
        self.status_timer.start(250)

    def closeEvent(self, event) -> None:  # noqa: N802
        self.save_ui_settings()
        self.stop_session_logging()
        self.udp_link.disconnect()
        super().closeEvent(event)

    def restore_ui_settings(self) -> None:
        geometry = self.settings.value("window_geometry")
        if geometry:
            try:
                self.restoreGeometry(geometry)
            except TypeError:
                pass
        self.host_input.setText(str(self.settings.value("host", self.host_input.text())))
        try:
            self.remote_port_input.setValue(int(self.settings.value("remote_port", self.remote_port_input.value())))
            self.local_port_input.setValue(int(self.settings.value("local_port", self.local_port_input.value())))
            self.tabs.setCurrentIndex(int(self.settings.value("tab_index", self.tabs.currentIndex())))
        except (TypeError, ValueError):
            pass

    def save_ui_settings(self) -> None:
        self.settings.setValue("window_geometry", self.saveGeometry())
        self.settings.setValue("host", self.host_input.text().strip())
        self.settings.setValue("remote_port", self.remote_port_input.value())
        self.settings.setValue("local_port", self.local_port_input.value())
        self.settings.setValue("tab_index", self.tabs.currentIndex())

    def _build_ui(self) -> None:
        root = QWidget()
        layout = QVBoxLayout(root)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(8)

        layout.addWidget(self._build_connection_panel())

        self.tabs = QTabWidget()
        self.tabs.addTab(self._build_main_tab(), "Dashboard")
        self.tabs.addTab(self._build_log_tab(), "Log")
        layout.addWidget(self.tabs)

        self.setCentralWidget(root)

    def _build_connection_panel(self) -> QGroupBox:
        panel = QGroupBox("UDP link")
        layout = QGridLayout(panel)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setHorizontalSpacing(10)
        layout.setVerticalSpacing(8)
        layout.setColumnStretch(9, 1)

        self.host_input = QLineEdit("10.250.131.125")
        self.host_input.setFixedWidth(180)

        self.remote_port_input = QSpinBox()
        self.remote_port_input.setRange(1, 65535)
        self.remote_port_input.setValue(4210)
        self.remote_port_input.setFixedWidth(96)

        self.local_port_input = QSpinBox()
        self.local_port_input.setRange(1, 65535)
        self.local_port_input.setValue(4211)
        self.local_port_input.setFixedWidth(96)

        self.status_label = QLabel("Disconnected")
        self.status_label.setObjectName("statusBadge")
        self.status_label.setMinimumWidth(150)
        self.status_label.setAlignment(Qt.AlignCenter)

        self.telemetry_age_label = QLabel("Telemetry: -")
        self.telemetry_age_label.setObjectName("metricValue")
        self.telemetry_age_label.setMinimumWidth(118)

        self.packet_count_label = QLabel("Packets: 0")
        self.packet_count_label.setObjectName("metricValue")
        self.packet_count_label.setMinimumWidth(88)

        self.packet_rate_label = QLabel("Rate: 0/s")
        self.packet_rate_label.setObjectName("metricValue")
        self.packet_rate_label.setMinimumWidth(78)

        self.auto_log_label = QLabel("Auto log: idle")
        self.auto_log_label.setObjectName("sectionNote")
        self.auto_log_label.setMinimumWidth(180)

        self.response_label = QLabel("Last response: -")
        self.response_label.setObjectName("sectionNote")
        self.response_label.setMinimumWidth(220)

        self.connect_button = QPushButton("Start UDP")
        self.connect_button.setObjectName("primaryButton")
        self.connect_button.clicked.connect(self.toggle_connection)

        header_button = QPushButton("Request Header")
        header_button.setFixedWidth(120)
        header_button.clicked.connect(lambda: self.send_command("HEADER"))

        json_button = QPushButton("Request JSON")
        json_button.setFixedWidth(120)
        json_button.clicked.connect(lambda: self.send_command("GET JSON"))

        layout.addWidget(QLabel("ESP32 host"), 0, 0)
        layout.addWidget(self.host_input, 0, 1)
        layout.addWidget(QLabel("Remote port"), 0, 2)
        layout.addWidget(self.remote_port_input, 0, 3)
        layout.addWidget(QLabel("Local port"), 0, 4)
        layout.addWidget(self.local_port_input, 0, 5)
        layout.addWidget(self.connect_button, 0, 6)
        layout.addWidget(self.status_label, 0, 10)

        layout.addWidget(header_button, 1, 0)
        layout.addWidget(json_button, 1, 1)
        layout.addWidget(self.auto_log_label, 1, 2, 1, 3)
        layout.addWidget(self.telemetry_age_label, 1, 5)
        layout.addWidget(self.packet_count_label, 1, 6)
        layout.addWidget(self.packet_rate_label, 1, 7)
        layout.addWidget(self.response_label, 1, 8, 1, 3)
        self._set_status("Disconnected", "error")
        return panel

    def _build_main_tab(self) -> QWidget:
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.NoFrame)

        tab = QWidget()
        scroll.setWidget(tab)

        layout = QVBoxLayout(tab)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        self.alert_label = QLabel("")
        self.alert_label.setObjectName("statusChip")
        self.alert_label.setProperty("state", "warn")
        self.alert_label.hide()
        layout.addWidget(self.alert_label)

        top = QHBoxLayout()
        top.setSpacing(8)
        top.addWidget(self._build_status_group(), 1)
        top.addWidget(self._make_group("Receiver", ["ch_roll", "ch_pitch", "ch_throttle", "ch_yaw", "radio_interval_ms"]), 1)
        top.addWidget(self._build_compact_motor_group(), 1)

        middle = QHBoxLayout()
        middle.setSpacing(8)
        middle.addWidget(self._build_attitude_chart_group(), 3)
        middle.addWidget(self._make_group("IMU", ["roll", "pitch", "yaw", "gx", "gy", "gz", "imu_ready", "imu_cal_sys", "imu_cal_gyro"]), 2)

        layout.addLayout(top)
        layout.addLayout(middle)
        layout.addWidget(self._build_pid_group())
        layout.addLayout(self._build_pid_buttons())
        self.pid_lock_label = QLabel("PID updates unlocked while disarmed.")
        self.pid_lock_label.setObjectName("statusChip")
        self.pid_lock_label.setProperty("state", "ok")
        layout.addWidget(self.pid_lock_label)
        layout.addStretch()
        return scroll

    def _build_status_group(self) -> QGroupBox:
        group = QGroupBox("Flight status")
        grid = QGridLayout(group)
        grid.setContentsMargins(8, 8, 8, 8)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(6)

        for index, field in enumerate(["radio_ok", "failsafe", "arm", "mode", "safety_reason", "motors_started"]):
            chip = QLabel(f"{DISPLAY_LABELS.get(field, field)}\n-")
            chip.setObjectName("statusChip")
            chip.setAlignment(Qt.AlignCenter)
            chip.setMinimumHeight(44)
            chip.setProperty("state", "warn")
            self.status_chips[field] = chip
            grid.addWidget(chip, index // 2, index % 2)

        return group

    def _build_pid_group(self) -> QGroupBox:
        pid_group = QGroupBox("PID tuning")
        axis_layout = QHBoxLayout(pid_group)
        axis_layout.setContentsMargins(8, 8, 8, 8)
        axis_layout.setSpacing(8)
        pid_group.setMaximumHeight(178)

        for axis, fields in PID_AXIS_FIELDS.items():
            axis_group = QGroupBox(axis)
            axis_grid = QGridLayout(axis_group)
            axis_grid.setContentsMargins(8, 8, 8, 8)
            axis_grid.setHorizontalSpacing(8)
            axis_grid.setVerticalSpacing(5)
            axis_grid.setColumnStretch(1, 1)
            axis_group.setMaximumHeight(145)

            for row, name in enumerate(fields):
                spin = QDoubleSpinBox()
                spin.setDecimals(4)
                spin.setRange(0.0, 20.0 if "_rate" not in name and "_i_" not in name else 10.0)
                if "_i_" in name:
                    spin.setRange(0.0, 2.0)
                spin.setSingleStep(0.05)
                spin.setValue(DEFAULT_PID_VALUES[name])
                spin.setMinimumWidth(120)
                spin.setMaximumWidth(170)
                spin.setFixedHeight(26)
                self.pid_inputs[name] = spin

                send_button = QPushButton("Send")
                send_button.setFixedWidth(78)
                send_button.setFixedHeight(28)
                send_button.setToolTip(f"Send {name} only. Disabled while the drone is armed.")
                send_button.clicked.connect(lambda checked=False, field=name: self.send_pid(field))
                self.pid_send_buttons.append(send_button)

                label = QLabel(PID_LABELS.get(name, name))
                label.setToolTip(name)
                label.setMinimumWidth(115)
                label.setMaximumWidth(140)
                label.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
                axis_grid.addWidget(label, row, 0)
                axis_grid.addWidget(spin, row, 1)
                axis_grid.addWidget(send_button, row, 2)

            axis_layout.addWidget(axis_group, 1)

        return pid_group

    def _build_pid_buttons(self) -> QHBoxLayout:
        button_bar = QHBoxLayout()
        button_bar.setSpacing(8)
        send_all_button = QPushButton("Send All PID")
        send_all_button.setObjectName("dangerButton")
        send_all_button.setFixedWidth(120)
        send_all_button.setToolTip("Send every PID value. Disabled while the drone is armed.")
        send_all_button.clicked.connect(self.send_all_pid)
        self.pid_action_buttons.append(send_all_button)

        load_button = QPushButton("Load From Telemetry")
        load_button.setFixedWidth(150)
        load_button.setToolTip("Copy the latest PID values received from the ESP32 into the dashboard inputs.")
        load_button.clicked.connect(self.load_pid_from_telemetry)

        request_button = QPushButton("Request PID")
        request_button.setObjectName("primaryButton")
        request_button.setFixedWidth(110)
        request_button.setToolTip("Ask the ESP32 to send the current PID values.")
        request_button.clicked.connect(lambda: self.send_command("PID"))

        self.preset_combo = QComboBox()
        self.preset_combo.setMinimumWidth(165)
        self.refresh_pid_presets()

        load_preset_button = QPushButton("Load Preset")
        load_preset_button.setFixedWidth(110)
        load_preset_button.setToolTip("Load the selected preset into the dashboard inputs only.")
        load_preset_button.clicked.connect(self.load_selected_pid_preset)

        save_preset_button = QPushButton("Save Preset")
        save_preset_button.setFixedWidth(110)
        save_preset_button.setToolTip("Save the current dashboard PID values as a local preset.")
        save_preset_button.clicked.connect(self.save_current_pid_preset)

        delete_preset_button = QPushButton("Delete Preset")
        delete_preset_button.setFixedWidth(110)
        delete_preset_button.setToolTip("Delete the selected local preset.")
        delete_preset_button.clicked.connect(self.delete_selected_pid_preset)

        button_bar.addWidget(send_all_button)
        button_bar.addWidget(load_button)
        button_bar.addWidget(request_button)
        button_bar.addSpacing(12)
        button_bar.addWidget(QLabel("PID preset"))
        button_bar.addWidget(self.preset_combo)
        button_bar.addWidget(load_preset_button)
        button_bar.addWidget(save_preset_button)
        button_bar.addWidget(delete_preset_button)
        button_bar.addStretch()
        return button_bar

    def _build_attitude_chart_group(self) -> QGroupBox:
        group = QGroupBox("Attitude: Roll and Pitch")
        layout = QVBoxLayout(group)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        badge_row = QHBoxLayout()
        self.roll_badge = QLabel("Roll: - deg")
        self.roll_badge.setObjectName("valueBadge")
        self.roll_badge.setProperty("state", "ok")
        self.pitch_badge = QLabel("Pitch: - deg")
        self.pitch_badge.setObjectName("valueBadge")
        self.pitch_badge.setProperty("state", "ok")

        badge_row.addWidget(self.roll_badge)
        badge_row.addWidget(self.pitch_badge)
        badge_row.addStretch()

        layout.addLayout(badge_row)
        layout.addWidget(self.attitude_chart)
        return group

    def _build_compact_motor_group(self) -> QGroupBox:
        group = QGroupBox("Motors")
        grid = QGridLayout(group)
        grid.setContentsMargins(8, 8, 8, 8)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(4)
        grid.setColumnStretch(2, 1)

        for row, field in enumerate(["motor1", "motor2", "motor3", "motor4"]):
            label = QLabel("-")
            label.setObjectName("metricValue")
            label.setMinimumWidth(58)
            label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self.value_labels[field] = label

            bar = QProgressBar()
            bar.setRange(1000, 2000)
            bar.setValue(1000)
            bar.setFormat("-")
            bar.setTextVisible(True)
            bar.setFixedHeight(16)
            bar.setProperty("level", "ok")
            self.motor_bars[field] = bar

            grid.addWidget(QLabel(field), row, 0)
            grid.addWidget(label, row, 1)
            grid.addWidget(bar, row, 2)
        return group

    def _build_log_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)

        command_bar = QHBoxLayout()
        self.command_input = QLineEdit()
        self.command_input.setPlaceholderText("Command: SET <NAME> <VALUE>")
        self.command_input.returnPressed.connect(self.send_manual_command)

        send_button = QPushButton("Send Command")
        send_button.setObjectName("primaryButton")
        send_button.clicked.connect(self.send_manual_command)

        command_bar.addWidget(self.command_input)
        command_bar.addWidget(send_button)

        self.response_panel = QLabel("Last response: -")
        self.response_panel.setObjectName("sectionNote")

        self.log_session_label = QLabel("Auto session log starts when UDP starts.")
        self.log_session_label.setObjectName("sectionNote")

        log_tools = QHBoxLayout()
        log_title = QLabel("Event log")
        log_title.setObjectName("panelTitle")
        self.pause_log_button = QPushButton("Pause Log")
        self.pause_log_button.setFixedWidth(96)
        self.pause_log_button.clicked.connect(self.toggle_log_pause)
        clear_log_button = QPushButton("Clear Log")
        clear_log_button.setFixedWidth(90)
        clear_log_button.clicked.connect(self.clear_log)
        log_tools.addWidget(log_title)
        log_tools.addStretch()
        log_tools.addWidget(self.pause_log_button)
        log_tools.addWidget(clear_log_button)

        self.log_output = QPlainTextEdit()
        self.log_output.setReadOnly(True)
        self.log_output.setPlaceholderText("No log yet. Start UDP or send a command to see events here.")

        layout.addLayout(command_bar)
        layout.addWidget(self.response_panel)
        layout.addWidget(self.log_session_label)
        layout.addLayout(log_tools)
        layout.addWidget(self.log_output)
        return tab

    def _make_group(self, title: str, fields: list[str]) -> QGroupBox:
        group = QGroupBox(title)
        grid = QGridLayout(group)
        grid.setColumnStretch(1, 1)

        for row, field in enumerate(fields):
            label = QLabel("-")
            label.setObjectName("metricValue")
            label.setMinimumWidth(90)
            label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            self.value_labels[field] = label
            grid.addWidget(QLabel(DISPLAY_LABELS.get(field, field)), row, 0)
            grid.addWidget(label, row, 1)

        return group

    def _apply_state(self, widget: QWidget, state: str) -> None:
        widget.setProperty("state", state)
        widget.style().unpolish(widget)
        widget.style().polish(widget)

    def _apply_level(self, widget: QWidget, level: str) -> None:
        widget.setProperty("level", level)
        widget.style().unpolish(widget)
        widget.style().polish(widget)

    def _set_status(self, text: str, state: str) -> None:
        self.status_label.setText(text)
        self._apply_state(self.status_label, state)

    def _is_armed(self) -> bool:
        try:
            return int(self.latest_data.get("arm", 0)) == 1
        except (TypeError, ValueError):
            return False

    def _pid_updates_locked(self) -> bool:
        return self._is_armed()

    def _is_pid_update_command(self, command: str) -> bool:
        upper = command.strip().upper()
        return upper.startswith("SET ")

    def _current_pid_values(self) -> dict[str, float]:
        return {field: self.pid_inputs[field].value() for field in PID_FIELDS}

    def _load_user_pid_presets(self) -> dict[str, dict[str, float]]:
        if not PRESET_FILE.exists():
            return {}
        try:
            raw = json.loads(PRESET_FILE.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return {}

        presets: dict[str, dict[str, float]] = {}
        if not isinstance(raw, dict):
            return presets

        for name, values in raw.items():
            if not isinstance(name, str) or not isinstance(values, dict):
                continue
            preset: dict[str, float] = {}
            for field in PID_FIELDS:
                try:
                    preset[field] = float(values[field])
                except (KeyError, TypeError, ValueError):
                    preset[field] = DEFAULT_PID_VALUES[field]
            presets[name] = preset
        return presets

    def _save_user_pid_presets(self) -> None:
        PRESET_FILE.write_text(json.dumps(self.user_pid_presets, indent=2), encoding="utf-8")

    def refresh_pid_presets(self) -> None:
        self.preset_combo.clear()
        for name in BUILTIN_PID_PRESETS:
            self.preset_combo.addItem(f"Built-in: {name}", ("built-in", name))
        for name in sorted(self.user_pid_presets):
            self.preset_combo.addItem(f"User: {name}", ("user", name))

    def load_selected_pid_preset(self) -> None:
        source, name = self.preset_combo.currentData()
        values = BUILTIN_PID_PRESETS[name] if source == "built-in" else self.user_pid_presets[name]
        for field, value in values.items():
            if field in self.pid_inputs:
                self.pid_inputs[field].setValue(float(value))
        self.log(f"Loaded PID preset: {name}")

    def save_current_pid_preset(self) -> None:
        name, accepted = QInputDialog.getText(self, "Save PID Preset", "Preset name:")
        name = name.strip()
        if not accepted or not name:
            return
        self.user_pid_presets[name] = self._current_pid_values()
        self._save_user_pid_presets()
        self.refresh_pid_presets()
        self.log(f"Saved PID preset: {name}")

    def delete_selected_pid_preset(self) -> None:
        source, name = self.preset_combo.currentData()
        if source != "user":
            self.log("Built-in presets cannot be deleted")
            return
        answer = QMessageBox.question(
            self,
            "Delete PID Preset",
            f"Delete preset '{name}'?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if answer != QMessageBox.StandardButton.Yes:
            return
        self.user_pid_presets.pop(name, None)
        self._save_user_pid_presets()
        self.refresh_pid_presets()
        self.log(f"Deleted PID preset: {name}")

    def toggle_connection(self) -> None:
        if self.udp_link.is_connected:
            self.log("Stopping UDP link")
            self.udp_link.disconnect()
            self.stop_session_logging()
            self._set_status("Disconnected", "error")
            self.connect_button.setText("Start UDP")
            self.last_telemetry_time = 0.0
            self.latest_data = {}
            self.telemetry_age_label.setText("Telemetry: -")
            self.packet_count_label.setText("Packets: 0")
            self.packet_rate_label.setText("Rate: 0/s")
            self.auto_log_label.setText("Auto log: idle")
            self.reset_live_indicators()
            self.update_pid_safety()
            return

        try:
            self.telemetry_count = 0
            self.packet_rate = 0.0
            self.packet_rate_last_time = time.monotonic()
            self.packet_rate_last_count = 0
            self.pid_loaded_once = False
            self.latest_data = {}
            self.reset_live_indicators()
            self.update_pid_safety()
            self.udp_link.connect(
                self.host_input.text().strip(),
                self.remote_port_input.value(),
                self.local_port_input.value(),
            )
            self.start_session_logging()
            self.connect_button.setText("Stop UDP")
            self._set_status("Waiting telemetry", "warn")
            self.packet_count_label.setText("Packets: 0")
            QTimer.singleShot(300, lambda: self.send_command("PID", log_command=False))
        except OSError as exc:
            self.stop_session_logging()
            self._set_status("Start failed", "error")
            self.log(f"UDP start error: {exc}")

    def poll_events(self) -> None:
        while True:
            try:
                event = self.udp_link.events.get_nowait()
            except queue.Empty:
                break

            if event.kind == "line":
                self.handle_line(event.message)
            else:
                self.log(event.message, kind=event.kind)

    def handle_line(self, line: str) -> None:
        if line.startswith("TEL,"):
            data = parse_csv_line(line)
            if data:
                self.last_telemetry_time = time.monotonic()
                self.telemetry_count += 1
                self._set_status("Telemetry OK", "ok")
                self.update_dashboard(data)
                self.write_csv_sample(data)
            return

        if line.startswith("{"):
            try:
                data = json.loads(line)
            except json.JSONDecodeError:
                self.log(line)
                return
            self.last_telemetry_time = time.monotonic()
            self.telemetry_count += 1
            self._set_status("Telemetry OK", "ok")
            self.update_dashboard(data)
            self.write_csv_sample(data)
            return

        if line.startswith("PID,"):
            self.handle_pid_line(line)
            self.update_response(line)
            self.log(line, kind="response")
            return

        if line.startswith(COMMAND_RESPONSE_PREFIXES):
            self.update_response(line)
            self.log(line, kind="response")
            return

        self.log(line, kind="info")

    def handle_pid_line(self, line: str) -> None:
        parts = line.split(",")
        values = parts[1:]
        if len(values) != len(PID_FIELDS):
            return

        for field, value in zip(PID_FIELDS, values):
            try:
                self.pid_inputs[field].setValue(float(value))
            except ValueError:
                pass
        self.pid_loaded_once = True

    def update_dashboard(self, data: dict[str, int | float | str]) -> None:
        self.latest_data = data

        for field, label in self.value_labels.items():
            value = data.get(field, "-")
            label.setText(str(value))

        self.attitude_chart.add_sample(data.get("roll"), data.get("pitch"))
        self.update_status_chips(data)
        self.update_attitude_badges(data)
        self.update_motor_bars(data)
        self.update_pid_safety()
        self.log_state_changes(data)

        if not self.pid_loaded_once:
            self.load_pid_from_telemetry()
            self.pid_loaded_once = True

    def log_state_changes(self, data: dict[str, int | float | str]) -> None:
        if not self.last_logged_state:
            self.write_event_record(
                "state",
                "Initial state: "
                f"arm={data.get('arm', '-')}, mode={data.get('mode', '-')}, "
                f"failsafe={data.get('failsafe', '-')}, radio_ok={data.get('radio_ok', '-')}, "
                f"safety={data.get('safety_reason', '-')}",
            )
        tracked_fields = [
            "radio_ok",
            "failsafe",
            "arm",
            "mode",
            "safety_reason",
            "motors_started",
            "imu_ready",
            "imu_cal_sys",
            "imu_cal_gyro",
            "imu_cal_accel",
            "imu_cal_mag",
            "ch_roll",
            "ch_pitch",
            "ch_throttle",
            "ch_yaw",
            "k_roll",
            "k_pitch",
            "k_yaw",
            "k_roll_rate",
            "k_pitch_rate",
            "k_yaw_rate",
            "k_i_roll",
            "k_i_pitch",
            "k_i_yaw",
        ]
        for field in tracked_fields:
            new_value = data.get(field)
            old_value = self.last_logged_state.get(field)
            if old_value is None:
                self.last_logged_state[field] = new_value
                continue
            if new_value != old_value:
                self.last_logged_state[field] = new_value
                self.write_event_record("change", f"{field} changed from {old_value} to {new_value}")

    def update_status_chips(self, data: dict[str, int | float | str]) -> None:
        def as_int(field: str) -> int:
            try:
                return int(float(data.get(field, 0) or 0))
            except (TypeError, ValueError):
                return 0

        radio_ok = as_int("radio_ok")
        failsafe = as_int("failsafe")
        arm = as_int("arm")
        mode = data.get("mode", "-")
        safety_reason = str(data.get("safety_reason", "-"))
        motors_started = as_int("motors_started")
        safety_state = "ok" if safety_reason in ("", "ok", "disarmed", "idle") else "warn"
        if safety_reason in ("radio", "imu", "tilt", "throttle", "actuator"):
            safety_state = "error"

        status_values = {
            "radio_ok": ("Radio\nOK" if radio_ok == 1 else "Radio\nLost", "ok" if radio_ok == 1 else "error"),
            "failsafe": ("Failsafe\nON" if failsafe == 1 else "Failsafe\nOFF", "error" if failsafe == 1 else "ok"),
            "arm": ("ARMED" if arm == 1 else "DISARMED", "warn" if arm == 1 else "ok"),
            "mode": (f"Mode\n{mode}", "ok"),
            "safety_reason": (f"Safety\n{safety_reason}", safety_state),
            "motors_started": ("Motors\nON" if motors_started == 1 else "Motors\nOFF", "warn" if motors_started == 1 else "ok"),
        }

        for field, (text, state) in status_values.items():
            chip = self.status_chips.get(field)
            if chip:
                chip.setText(text)
                self._apply_state(chip, state)

        alerts = []
        alert_state = "warn"
        if radio_ok != 1:
            alerts.append("Radio signal is not OK")
            alert_state = "error"
        if failsafe == 1:
            alerts.append("Failsafe is active")
            alert_state = "error"
        if arm == 1:
            alerts.append("Armed: PID updates are locked")
        if safety_reason not in ("", "-", "ok", "disarmed", "idle"):
            alerts.append(f"Safety: {safety_reason}")
            if safety_state == "error":
                alert_state = "error"
        if alerts:
            self.alert_label.setText(" | ".join(alerts))
            self._apply_state(self.alert_label, alert_state)
            self.alert_label.show()
        else:
            self.alert_label.hide()

    def update_attitude_badges(self, data: dict[str, int | float | str]) -> None:
        for field, badge in (("roll", self.roll_badge), ("pitch", self.pitch_badge)):
            try:
                value = float(data.get(field, 0.0))
            except (TypeError, ValueError):
                badge.setText(f"{field.title()}: - deg")
                self._apply_state(badge, "warn")
                continue
            state = "error" if abs(value) >= 30.0 else "warn" if abs(value) >= 15.0 else "ok"
            badge.setText(f"{field.title()}: {value:.1f} deg")
            self._apply_state(badge, state)

    def reset_live_indicators(self) -> None:
        for label in self.value_labels.values():
            label.setText("-")
        for field, chip in self.status_chips.items():
            chip.setText(f"{DISPLAY_LABELS.get(field, field)}\n-")
            self._apply_state(chip, "warn")
        for field, badge in (("roll", self.roll_badge), ("pitch", self.pitch_badge)):
            badge.setText(f"{field.title()}: - deg")
            self._apply_state(badge, "warn")
        for field, bar in self.motor_bars.items():
            bar.setValue(1000)
            bar.setFormat("-")
            self._apply_level(bar, "ok")
        self.attitude_chart.clear()
        if hasattr(self, "alert_label"):
            self.alert_label.hide()

    def update_motor_bars(self, data: dict[str, int | float | str]) -> None:
        for field, bar in self.motor_bars.items():
            try:
                value = int(float(data.get(field, 1000)))
            except (TypeError, ValueError):
                bar.setValue(1000)
                bar.setFormat("-")
                self._apply_level(bar, "ok")
                continue
            value = max(1000, min(2000, value))
            bar.setValue(value)
            bar.setFormat(f"{value} us")
            level = "error" if value >= 1900 else "warn" if value >= 1750 else "ok"
            self._apply_level(bar, level)

    def update_pid_safety(self) -> None:
        locked = self._pid_updates_locked()
        for button in self.pid_send_buttons + self.pid_action_buttons:
            button.setEnabled(not locked)

        if locked:
            self.pid_lock_label.setText("PID locked: arm=1. Disarm before sending PID updates.")
            self._apply_state(self.pid_lock_label, "warn")
        else:
            self.pid_lock_label.setText("PID updates unlocked while disarmed.")
            self._apply_state(self.pid_lock_label, "ok")

    def send_hello_if_needed(self) -> None:
        if not self.udp_link.is_connected:
            return
        if time.monotonic() - self.last_telemetry_time > 2.0:
            self.send_command("HELLO", log_command=False)

    def update_connection_status(self) -> None:
        if not self.udp_link.is_connected:
            self._set_status("Disconnected", "error")
            return

        now = time.monotonic()
        elapsed = now - self.packet_rate_last_time
        if elapsed >= 1.0:
            packet_delta = self.telemetry_count - self.packet_rate_last_count
            self.packet_rate = packet_delta / elapsed
            self.packet_rate_last_time = now
            self.packet_rate_last_count = self.telemetry_count
            self.packet_rate_label.setText(f"Rate: {self.packet_rate:.1f}/s")

        if self.last_telemetry_time <= 0.0:
            self._set_status("Waiting telemetry", "warn")
            self.telemetry_age_label.setText("Telemetry: waiting")
            self.packet_count_label.setText(f"Packets: {self.telemetry_count}")
            return

        age = now - self.last_telemetry_time
        self.telemetry_age_label.setText(f"Telemetry: {age:.1f}s ago")
        self.packet_count_label.setText(f"Packets: {self.telemetry_count}")

        if age > 2.5:
            self._set_status("Telemetry timeout", "error")
        elif age > 1.0:
            self._set_status("Telemetry slow", "warn")
        else:
            self._set_status("Telemetry OK", "ok")

    def update_response(self, line: str) -> None:
        text = f"Last response: {line}"
        self.response_label.setText(text)
        self.response_label.setToolTip(text)
        self.response_panel.setText(text)

    def start_session_logging(self) -> None:
        if self.csv_writer or self.event_log_writer:
            return
        LOG_DIR.mkdir(exist_ok=True)
        self.session_log_dir = LOG_DIR / dt.datetime.now().strftime("session_%Y%m%d_%H%M%S")
        self.session_log_dir.mkdir(parents=True, exist_ok=True)

        self.csv_log_path = self.session_log_dir / "telemetry.csv"
        self.csv_file = self.csv_log_path.open("w", newline="", encoding="utf-8")
        telemetry_fields = ["pc_time_iso", *CSV_FIELDS]
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=telemetry_fields)
        self.csv_writer.writeheader()

        self.event_log_path = self.session_log_dir / "events.csv"
        self.event_log_file = self.event_log_path.open("w", newline="", encoding="utf-8")
        event_fields = [
            "pc_time_iso",
            "kind",
            "message",
            "arm",
            "mode",
            "failsafe",
            "radio_ok",
            "safety_reason",
            "motors_started",
            "roll",
            "pitch",
            "yaw",
            "gx",
            "gy",
            "gz",
            "ch_throttle",
            "motor1",
            "motor2",
            "motor3",
            "motor4",
        ]
        self.event_log_writer = csv.DictWriter(self.event_log_file, fieldnames=event_fields)
        self.event_log_writer.writeheader()

        self.last_logged_state = {}
        session_text = f"Auto session log: {self.session_log_dir}"
        self.auto_log_label.setText("Auto log: active")
        self.log_session_label.setText(session_text)
        self.log(session_text, kind="session")

    def stop_session_logging(self) -> None:
        last_dir = self.session_log_dir
        if self.event_log_writer:
            self.write_event_record("session", "Session logging stopped")
        if self.csv_file:
            self.csv_file.close()
        if self.event_log_file:
            self.event_log_file.close()
        self.csv_file = None
        self.csv_writer = None
        self.event_log_file = None
        self.event_log_writer = None
        self.csv_log_path = None
        self.event_log_path = None
        self.session_log_dir = None
        self.last_logged_state = {}
        if hasattr(self, "auto_log_label"):
            self.auto_log_label.setText("Auto log: idle")
        if hasattr(self, "log_session_label"):
            if last_dir:
                self.log_session_label.setText(f"Last session log: {last_dir}")
            else:
                self.log_session_label.setText("Auto session log starts when UDP starts.")

    def write_csv_sample(self, data: dict[str, int | float | str]) -> None:
        if not self.csv_writer or not self.csv_file:
            return
        row = {field: data.get(field, "") for field in CSV_FIELDS}
        row["pc_time_iso"] = dt.datetime.now().isoformat(timespec="milliseconds")
        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def write_event_record(self, kind: str, message: str) -> None:
        if not self.event_log_writer or not self.event_log_file:
            return
        row = {
            "pc_time_iso": dt.datetime.now().isoformat(timespec="milliseconds"),
            "kind": kind,
            "message": message,
            "arm": self.latest_data.get("arm", ""),
            "mode": self.latest_data.get("mode", ""),
            "failsafe": self.latest_data.get("failsafe", ""),
            "radio_ok": self.latest_data.get("radio_ok", ""),
            "safety_reason": self.latest_data.get("safety_reason", ""),
            "motors_started": self.latest_data.get("motors_started", ""),
            "roll": self.latest_data.get("roll", ""),
            "pitch": self.latest_data.get("pitch", ""),
            "yaw": self.latest_data.get("yaw", ""),
            "gx": self.latest_data.get("gx", ""),
            "gy": self.latest_data.get("gy", ""),
            "gz": self.latest_data.get("gz", ""),
            "ch_throttle": self.latest_data.get("ch_throttle", ""),
            "motor1": self.latest_data.get("motor1", ""),
            "motor2": self.latest_data.get("motor2", ""),
            "motor3": self.latest_data.get("motor3", ""),
            "motor4": self.latest_data.get("motor4", ""),
        }
        self.event_log_writer.writerow(row)
        self.event_log_file.flush()

    def load_pid_from_telemetry(self) -> None:
        for field in PID_FIELDS:
            if field in self.latest_data:
                try:
                    self.pid_inputs[field].setValue(float(self.latest_data[field]))
                except (TypeError, ValueError):
                    pass
        self.pid_loaded_once = True

    def send_pid(self, field: str) -> None:
        if self._pid_updates_locked():
            self.log("PID update blocked by dashboard: arm=1", kind="warning")
            return
        value = self.pid_inputs[field].value()
        self.send_command(f"SET {field.upper()} {value:.4f}")

    def send_all_pid(self) -> None:
        if self._pid_updates_locked():
            self.log("PID update blocked by dashboard: arm=1", kind="warning")
            return
        for field in PID_FIELDS:
            self.send_pid(field)

    def send_manual_command(self) -> None:
        command = self.command_input.text().strip()
        if command:
            self.send_command(command)
            self.command_input.clear()

    def send_command(self, command: str, log_command: bool = True) -> None:
        if self._pid_updates_locked() and self._is_pid_update_command(command):
            self.log("PID update blocked by dashboard: arm=1", kind="warning")
            self.update_response("ERR,DASHBOARD_PID_LOCKED_WHILE_ARMED")
            return
        if log_command:
            self.log(f"> {command}", kind="command")
        self.udp_link.write(command)

    def toggle_log_pause(self) -> None:
        self.log_paused = not self.log_paused
        if self.log_paused:
            self.pause_log_button.setText("Resume Log")
            self.log_output.appendPlainText("[log paused]")
            return

        self.pause_log_button.setText("Pause Log")
        if self.paused_log_lines:
            self.log_output.appendPlainText(f"[log resumed: {len(self.paused_log_lines)} buffered lines]")
            self.log_output.appendPlainText("\n".join(self.paused_log_lines))
            self.paused_log_lines.clear()

    def clear_log(self) -> None:
        self.paused_log_lines.clear()
        self.log_output.clear()

    def log(self, message: str, kind: str = "info") -> None:
        timestamp = time.strftime("%H:%M:%S")
        line = f"[{timestamp}] {message}"
        self.write_event_record(kind, message)
        if self.log_paused:
            self.paused_log_lines.append(line)
            self.pause_log_button.setText(f"Resume ({len(self.paused_log_lines)})")
            return
        self.log_output.appendPlainText(line)


def main() -> int:
    app = QApplication(sys.argv)
    app.setStyleSheet(APP_STYLESHEET)
    window = Dashboard()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
