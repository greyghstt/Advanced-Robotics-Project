from __future__ import annotations

import json
import queue
import socket
import sys
import threading
import time
from dataclasses import dataclass

from PySide6.QtCore import QPointF, Qt, QTimer
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QPlainTextEdit,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

APP_STYLESHEET = """
QMainWindow, QWidget {
    background: #f7f7f7;
    color: #202124;
    font-size: 12px;
}
QGroupBox {
    background: #ffffff;
    border: 1px solid #d8d8d8;
    border-radius: 6px;
    margin-top: 10px;
    padding: 10px;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 4px;
}
QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox, QPlainTextEdit {
    background: #ffffff;
    color: #202124;
    border: 1px solid #c9c9c9;
    border-radius: 4px;
    padding: 4px;
}
QPushButton {
    background: #ffffff;
    color: #202124;
    border: 1px solid #b8b8b8;
    border-radius: 6px;
    padding: 6px 10px;
}
QPushButton:hover {
    background: #eeeeee;
}
QPushButton:pressed {
    background: #e1e1e1;
}
QTabWidget::pane {
    border: 1px solid #d8d8d8;
    background: #ffffff;
}
QTabBar::tab {
    background: #eeeeee;
    color: #202124;
    border: 1px solid #d8d8d8;
    border-bottom: none;
    border-top-left-radius: 6px;
    border-top-right-radius: 6px;
    padding: 7px 14px;
}
QTabBar::tab:selected {
    background: #ffffff;
}
QLabel#sectionNote {
    color: #5f6368;
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
    "k_roll": 3.99,
    "k_pitch": 3.99,
    "k_yaw": 3.99,
    "k_roll_rate": 1.1581,
    "k_pitch_rate": 1.1581,
    "k_yaw_rate": 1.1581,
    "k_i_roll": 0.0,
    "k_i_pitch": 0.0,
    "k_i_yaw": 0.0,
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
    if not parts or parts[0] != "TEL" or len(parts) != len(CSV_FIELDS):
        return None
    return {name: parse_value(value) for name, value in zip(CSV_FIELDS, parts)}


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
        self.setMinimumHeight(210)

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
        painter.drawText(margin_left, 16, f"Roll {latest_roll:.1f}")
        painter.setPen(QColor("#1a73e8"))
        painter.drawText(margin_left + 92, 16, f"Pitch {latest_pitch:.1f}")


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
        self.setWindowTitle("Proyek Robotika Lanjut - UDP GCS")
        self.resize(1280, 820)

        self.udp_link = UdpLink()
        self.latest_data: dict[str, int | float | str] = {}
        self.last_telemetry_time = 0.0
        self.pid_loaded_once = False

        self.value_labels: dict[str, QLabel] = {}
        self.pid_inputs: dict[str, QDoubleSpinBox] = {}
        self.attitude_chart = AttitudeChart()

        self._build_ui()

        self.event_timer = QTimer(self)
        self.event_timer.timeout.connect(self.poll_events)
        self.event_timer.start(50)

        self.hello_timer = QTimer(self)
        self.hello_timer.timeout.connect(self.send_hello_if_needed)
        self.hello_timer.start(1000)

    def closeEvent(self, event) -> None:  # noqa: N802
        self.udp_link.disconnect()
        super().closeEvent(event)

    def _build_ui(self) -> None:
        root = QWidget()
        layout = QVBoxLayout(root)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(10)

        layout.addLayout(self._build_connection_bar())

        tabs = QTabWidget()
        tabs.addTab(self._build_main_tab(), "Dashboard")
        tabs.addTab(self._build_log_tab(), "Log")
        layout.addWidget(tabs)

        self.setCentralWidget(root)

    def _build_connection_bar(self) -> QHBoxLayout:
        layout = QHBoxLayout()

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
        self.status_label.setMinimumWidth(130)
        self.status_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        self.connect_button = QPushButton("Start UDP")
        self.connect_button.clicked.connect(self.toggle_connection)

        header_button = QPushButton("Header")
        header_button.clicked.connect(lambda: self.send_command("HEADER"))

        json_button = QPushButton("JSON")
        json_button.clicked.connect(lambda: self.send_command("GET JSON"))

        layout.addWidget(QLabel("ESP32 host"))
        layout.addWidget(self.host_input)
        layout.addWidget(QLabel("Remote port"))
        layout.addWidget(self.remote_port_input)
        layout.addWidget(QLabel("Local port"))
        layout.addWidget(self.local_port_input)
        layout.addWidget(self.connect_button)
        layout.addWidget(header_button)
        layout.addWidget(json_button)
        layout.addStretch()
        layout.addWidget(self.status_label)
        return layout

    def _build_main_tab(self) -> QWidget:
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.NoFrame)

        tab = QWidget()
        scroll.setWidget(tab)

        layout = QVBoxLayout(tab)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        top = QHBoxLayout()
        top.setSpacing(10)
        top.addWidget(self._make_group("Status", ["radio_ok", "failsafe", "arm", "mode"]), 1)
        top.addWidget(self._make_group("Receiver", ["ch_roll", "ch_pitch", "ch_throttle", "ch_yaw"]), 1)
        top.addWidget(self._make_group("Motors", ["motor1", "motor2", "motor3", "motor4"]), 1)

        middle = QHBoxLayout()
        middle.setSpacing(10)
        middle.addWidget(self._build_attitude_chart_group(), 3)
        middle.addWidget(self._make_group("IMU", ["roll", "pitch", "yaw", "gx", "gy", "gz", "speed_z"]), 2)

        layout.addLayout(top)
        layout.addLayout(middle)
        layout.addWidget(self._build_pid_group())
        layout.addLayout(self._build_pid_buttons())
        note = QLabel("Firmware default blocks PID updates while arm=1. Disarm before sending PID.")
        note.setObjectName("sectionNote")
        layout.addWidget(note)
        layout.addStretch()
        return scroll

    def _build_pid_group(self) -> QGroupBox:
        pid_group = QGroupBox("PID input")
        axis_layout = QHBoxLayout(pid_group)
        axis_layout.setSpacing(10)

        for axis, fields in PID_AXIS_FIELDS.items():
            axis_group = QGroupBox(axis)
            axis_grid = QGridLayout(axis_group)
            axis_grid.setColumnStretch(1, 1)

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
                self.pid_inputs[name] = spin

                send_button = QPushButton("Send")
                send_button.setFixedWidth(78)
                send_button.clicked.connect(lambda checked=False, field=name: self.send_pid(field))

                axis_grid.addWidget(QLabel(name), row, 0)
                axis_grid.addWidget(spin, row, 1)
                axis_grid.addWidget(send_button, row, 2)

            axis_layout.addWidget(axis_group, 1)

        return pid_group

    def _build_pid_buttons(self) -> QHBoxLayout:
        button_bar = QHBoxLayout()
        send_all_button = QPushButton("Send All PID")
        send_all_button.setFixedWidth(120)
        send_all_button.clicked.connect(self.send_all_pid)

        load_button = QPushButton("Load From Telemetry")
        load_button.setFixedWidth(150)
        load_button.clicked.connect(self.load_pid_from_telemetry)

        request_button = QPushButton("Request PID")
        request_button.setFixedWidth(110)
        request_button.clicked.connect(lambda: self.send_command("PID"))

        button_bar.addWidget(send_all_button)
        button_bar.addWidget(load_button)
        button_bar.addWidget(request_button)
        button_bar.addStretch()
        return button_bar

    def _build_attitude_chart_group(self) -> QGroupBox:
        group = QGroupBox("Attitude: Roll dan Pitch")
        layout = QVBoxLayout(group)
        layout.addWidget(self.attitude_chart)
        return group

    def _build_log_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)

        command_bar = QHBoxLayout()
        self.command_input = QLineEdit()
        self.command_input.setPlaceholderText("Example: SET K_PITCH 4.8")
        self.command_input.returnPressed.connect(self.send_manual_command)

        send_button = QPushButton("Send Command")
        send_button.clicked.connect(self.send_manual_command)

        command_bar.addWidget(self.command_input)
        command_bar.addWidget(send_button)

        self.log_output = QPlainTextEdit()
        self.log_output.setReadOnly(True)

        layout.addLayout(command_bar)
        layout.addWidget(self.log_output)
        return tab

    def _make_group(self, title: str, fields: list[str]) -> QGroupBox:
        group = QGroupBox(title)
        grid = QGridLayout(group)
        grid.setColumnStretch(1, 1)

        for row, field in enumerate(fields):
            label = QLabel("-")
            label.setMinimumWidth(90)
            label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            self.value_labels[field] = label
            grid.addWidget(QLabel(field), row, 0)
            grid.addWidget(label, row, 1)

        return group

    def toggle_connection(self) -> None:
        if self.udp_link.is_connected:
            self.udp_link.disconnect()
            self.status_label.setText("Disconnected")
            self.connect_button.setText("Start UDP")
            return

        try:
            self.udp_link.connect(
                self.host_input.text().strip(),
                self.remote_port_input.value(),
                self.local_port_input.value(),
            )
            self.connect_button.setText("Stop UDP")
            self.status_label.setText("Waiting telemetry")
        except OSError as exc:
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
                self.log(event.message)

    def handle_line(self, line: str) -> None:
        if line.startswith("TEL,"):
            data = parse_csv_line(line)
            if data:
                self.last_telemetry_time = time.monotonic()
                self.status_label.setText("Telemetry OK")
                self.update_dashboard(data)
            return

        if line.startswith("{"):
            try:
                data = json.loads(line)
            except json.JSONDecodeError:
                self.log(line)
                return
            self.last_telemetry_time = time.monotonic()
            self.status_label.setText("Telemetry OK")
            self.update_dashboard(data)
            return

        if line.startswith("PID,"):
            self.handle_pid_line(line)
            self.log(line)
            return

        self.log(line)

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

        if not self.pid_loaded_once:
            self.load_pid_from_telemetry()
            self.pid_loaded_once = True

    def send_hello_if_needed(self) -> None:
        if not self.udp_link.is_connected:
            return
        if time.monotonic() - self.last_telemetry_time > 2.0:
            self.send_command("HELLO", log_command=False)

    def load_pid_from_telemetry(self) -> None:
        for field in PID_FIELDS:
            if field in self.latest_data:
                try:
                    self.pid_inputs[field].setValue(float(self.latest_data[field]))
                except (TypeError, ValueError):
                    pass
        self.pid_loaded_once = True

    def send_pid(self, field: str) -> None:
        value = self.pid_inputs[field].value()
        self.send_command(f"SET {field.upper()} {value:.4f}")

    def send_all_pid(self) -> None:
        for field in PID_FIELDS:
            self.send_pid(field)

    def send_manual_command(self) -> None:
        command = self.command_input.text().strip()
        if command:
            self.send_command(command)
            self.command_input.clear()

    def send_command(self, command: str, log_command: bool = True) -> None:
        if log_command:
            self.log(f"> {command}")
        self.udp_link.write(command)

    def log(self, message: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        self.log_output.appendPlainText(f"[{timestamp}] {message}")


def main() -> int:
    app = QApplication(sys.argv)
    app.setStyleSheet(APP_STYLESHEET)
    window = Dashboard()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
