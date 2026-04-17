from __future__ import annotations

import json
import queue
import sys
import threading
import time
from dataclasses import dataclass

import serial
from serial.tools import list_ports
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QPlainTextEdit,
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
class SerialEvent:
    kind: str
    message: str


class SerialLink:
    def __init__(self) -> None:
        self.serial_port: serial.Serial | None = None
        self.reader_thread: threading.Thread | None = None
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.events: queue.Queue[SerialEvent] = queue.Queue()

    @property
    def is_connected(self) -> bool:
        return self.serial_port is not None and self.serial_port.is_open

    def connect(self, port: str, baudrate: int) -> None:
        self.disconnect()
        self.stop_event.clear()
        self.serial_port = serial.Serial(port, baudrate=baudrate, timeout=0.1)
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()
        self.events.put(SerialEvent("status", f"Connected to {port}"))
        self.write("HEADER")

    def disconnect(self) -> None:
        self.stop_event.set()
        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=0.5)
        self.reader_thread = None

        with self.lock:
            if self.serial_port:
                try:
                    self.serial_port.close()
                except serial.SerialException:
                    pass
            self.serial_port = None

    def write(self, command: str) -> None:
        with self.lock:
            if not self.serial_port or not self.serial_port.is_open:
                self.events.put(SerialEvent("error", "Serial is not connected"))
                return
            payload = (command.strip() + "\n").encode("utf-8")
            self.serial_port.write(payload)

    def _reader_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                with self.lock:
                    port = self.serial_port
                if not port or not port.is_open:
                    break
                raw_line = port.readline()
            except serial.SerialException as exc:
                self.events.put(SerialEvent("error", str(exc)))
                break

            if not raw_line:
                continue

            line = raw_line.decode("utf-8", errors="replace").strip()
            if line:
                self.events.put(SerialEvent("line", line))


class Dashboard(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Robjut Bluetooth GCS")
        self.resize(1180, 760)

        self.serial_link = SerialLink()
        self.latest_data: dict[str, int | float | str] = {}
        self.pid_loaded_once = False

        self.value_labels: dict[str, QLabel] = {}
        self.pid_inputs: dict[str, QDoubleSpinBox] = {}

        self._build_ui()
        self.refresh_ports()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll_events)
        self.timer.start(50)

    def closeEvent(self, event) -> None:  # noqa: N802
        self.serial_link.disconnect()
        super().closeEvent(event)

    def _build_ui(self) -> None:
        root = QWidget()
        layout = QVBoxLayout(root)

        layout.addLayout(self._build_connection_bar())

        tabs = QTabWidget()
        tabs.addTab(self._build_main_tab(), "Dashboard")
        tabs.addTab(self._build_log_tab(), "Log")
        layout.addWidget(tabs)

        self.setCentralWidget(root)

    def _build_connection_bar(self) -> QHBoxLayout:
        layout = QHBoxLayout()

        self.port_combo = QComboBox()
        self.baud_input = QLineEdit("115200")
        self.baud_input.setFixedWidth(90)
        self.status_label = QLabel("Disconnected")

        refresh_button = QPushButton("Refresh")
        refresh_button.clicked.connect(self.refresh_ports)

        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.toggle_connection)

        request_header_button = QPushButton("Header")
        request_header_button.clicked.connect(lambda: self.send_command("HEADER"))

        request_json_button = QPushButton("JSON")
        request_json_button.clicked.connect(lambda: self.send_command("GET JSON"))

        layout.addWidget(QLabel("Port"))
        layout.addWidget(self.port_combo)
        layout.addWidget(QLabel("Baud"))
        layout.addWidget(self.baud_input)
        layout.addWidget(refresh_button)
        layout.addWidget(self.connect_button)
        layout.addWidget(request_header_button)
        layout.addWidget(request_json_button)
        layout.addStretch()
        layout.addWidget(self.status_label)
        return layout

    def _build_main_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)

        top = QHBoxLayout()
        top.addWidget(self._make_group("Status", ["radio_ok", "failsafe", "arm", "mode"]))
        top.addWidget(self._make_group("Receiver", ["ch_roll", "ch_pitch", "ch_throttle", "ch_yaw"]))
        top.addWidget(self._make_group("Motors", ["motor1", "motor2", "motor3", "motor4"]))

        middle = QHBoxLayout()
        middle.addWidget(self._make_group("Control", ["control1", "control2", "control3", "control4"]))
        middle.addWidget(self._make_group("IMU", ["roll", "pitch", "yaw", "gx", "gy", "gz", "speed_z"]))

        layout.addLayout(top)
        layout.addLayout(middle)
        layout.addWidget(self._build_pid_group())
        layout.addLayout(self._build_pid_buttons())
        layout.addWidget(QLabel("Firmware default blocks PID updates while arm=1. Disarm before sending PID."))
        layout.addStretch()
        return tab

    def _build_pid_group(self) -> QGroupBox:
        pid_group = QGroupBox("PID input")
        grid = QGridLayout(pid_group)

        for row, name in enumerate(PID_FIELDS):
            spin = QDoubleSpinBox()
            spin.setDecimals(4)
            spin.setRange(0.0, 20.0 if "_rate" not in name and "_i_" not in name else 10.0)
            if "_i_" in name:
                spin.setRange(0.0, 2.0)
            spin.setSingleStep(0.05)
            spin.setValue(DEFAULT_PID_VALUES[name])
            self.pid_inputs[name] = spin

            send_button = QPushButton("Send")
            send_button.clicked.connect(lambda checked=False, field=name: self.send_pid(field))

            grid.addWidget(QLabel(name), row, 0)
            grid.addWidget(spin, row, 1)
            grid.addWidget(send_button, row, 2)

        return pid_group

    def _build_pid_buttons(self) -> QHBoxLayout:
        button_bar = QHBoxLayout()
        send_all_button = QPushButton("Send All PID")
        send_all_button.clicked.connect(self.send_all_pid)

        load_button = QPushButton("Load From Telemetry")
        load_button.clicked.connect(self.load_pid_from_telemetry)

        request_button = QPushButton("Request PID")
        request_button.clicked.connect(lambda: self.send_command("PID"))

        button_bar.addWidget(send_all_button)
        button_bar.addWidget(load_button)
        button_bar.addWidget(request_button)
        button_bar.addStretch()
        return button_bar

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

        for row, field in enumerate(fields):
            label = QLabel("-")
            label.setMinimumWidth(90)
            self.value_labels[field] = label
            grid.addWidget(QLabel(field), row, 0)
            grid.addWidget(label, row, 1)

        return group

    def refresh_ports(self) -> None:
        current = self.port_combo.currentText()
        self.port_combo.clear()
        ports = list(list_ports.comports())
        for port in ports:
            label = f"{port.device} - {port.description}"
            self.port_combo.addItem(label, port.device)

        if current:
            index = self.port_combo.findText(current)
            if index >= 0:
                self.port_combo.setCurrentIndex(index)

    def toggle_connection(self) -> None:
        if self.serial_link.is_connected:
            self.serial_link.disconnect()
            self.status_label.setText("Disconnected")
            self.connect_button.setText("Connect")
            return

        port = self.port_combo.currentData()
        if not port:
            self.log("No serial port selected")
            return

        try:
            baudrate = int(self.baud_input.text())
            self.serial_link.connect(port, baudrate)
            self.connect_button.setText("Disconnect")
            self.status_label.setText(f"Connected: {port}")
        except (ValueError, serial.SerialException) as exc:
            self.log(f"Connect error: {exc}")

    def poll_events(self) -> None:
        while True:
            try:
                event = self.serial_link.events.get_nowait()
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
                self.update_dashboard(data)
            return

        if line.startswith("{"):
            try:
                data = json.loads(line)
            except json.JSONDecodeError:
                self.log(line)
                return
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

        if not self.pid_loaded_once:
            self.load_pid_from_telemetry()
            self.pid_loaded_once = True

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

    def send_command(self, command: str) -> None:
        self.log(f"> {command}")
        self.serial_link.write(command)

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
