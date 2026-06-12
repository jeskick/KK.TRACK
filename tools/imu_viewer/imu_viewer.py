#!/usr/bin/env python3
"""KK-track TX IMU serial viewer — real-time X/Y/Z waveform plots."""

from __future__ import annotations

import re
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple

import numpy as np
import pyqtgraph as pg
import serial
import serial.tools.list_ports
from PyQt5 import QtCore, QtGui, QtWidgets

BAUD_DEFAULT = 115200
WINDOW_SEC_DEFAULT = 10.0
REFRESH_HZ = 50
MAX_BUFFER_SEC = 30.0

CSV_RE = re.compile(
    r"^\s*(\d+(?:\.\d+)?)\s*,\s*([-+]?\d+(?:\.\d+)?)\s*,\s*([-+]?\d+(?:\.\d+)?)\s*,\s*([-+]?\d+(?:\.\d+)?)\s*$"
)
CSV_HEADER_ALIASES = ("ms,x,y,z", "ms,yaw,pitch,roll")


@dataclass
class ImuSample:
    t_ms: float
    x: float
    y: float
    z: float


class SerialWorker(QtCore.QObject):
    sample = QtCore.pyqtSignal(object)
    status = QtCore.pyqtSignal(str)
    error = QtCore.pyqtSignal(str)
    connected = QtCore.pyqtSignal()
    stopped = QtCore.pyqtSignal()

    def __init__(self) -> None:
        super().__init__()
        self._ser: Optional[serial.Serial] = None
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self, port: str, baud: int) -> None:
        self.stop()
        self._stop.clear()
        try:
            self._ser = serial.Serial(port, baud, timeout=0.05)
        except serial.SerialException as exc:
            self.error.emit(str(exc))
            return

        self.status.emit(f"Connected {port} @ {baud}")
        self.connected.emit()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._thread = None
        if self._ser and self._ser.is_open:
            try:
                self._ser.close()
            except serial.SerialException:
                pass
        self._ser = None
        self.stopped.emit()

    def _read_loop(self) -> None:
        assert self._ser is not None
        ser = self._ser
        t0: Optional[float] = None

        while not self._stop.is_set():
            try:
                raw = ser.readline()
            except serial.SerialException as exc:
                self.error.emit(str(exc))
                break

            if not raw:
                continue

            try:
                line = raw.decode("utf-8", errors="ignore").strip()
            except Exception:
                continue

            if line.startswith("# re-zero"):
                self.status.emit("re-zero (next sample)")
                continue
            if not line or line.startswith("#"):
                continue
            if line.lower() in CSV_HEADER_ALIASES or line.startswith("ms,") or line.startswith("W (") or line.startswith("I ("):
                continue

            m = CSV_RE.match(line)
            if not m:
                continue

            t_ms = float(m.group(1))
            x = float(m.group(2))
            y = float(m.group(3))
            z = float(m.group(4))

            if t0 is None:
                t0 = t_ms
            t_rel = (t_ms - t0) / 1000.0

            self.sample.emit(ImuSample(t_rel, x, y, z))

        self.stop()


class RingBuffer:
    def __init__(self, max_seconds: float) -> None:
        self.max_seconds = max_seconds
        self.t: Deque[float] = deque()
        self.x: Deque[float] = deque()
        self.y: Deque[float] = deque()
        self.z: Deque[float] = deque()
        self._last: Optional[ImuSample] = None
        self._rate_count = 0
        self._rate_t0 = time.monotonic()
        self.hz = 0.0

    def clear(self) -> None:
        self.t.clear()
        self.x.clear()
        self.y.clear()
        self.z.clear()
        self._last = None
        self._rate_count = 0
        self._rate_t0 = time.monotonic()
        self.hz = 0.0

    def push(self, s: ImuSample) -> None:
        self.t.append(s.t_ms)
        self.x.append(s.x)
        self.y.append(s.y)
        self.z.append(s.z)
        self._last = s

        self._rate_count += 1
        now = time.monotonic()
        dt = now - self._rate_t0
        if dt >= 1.0:
            self.hz = self._rate_count / dt
            self._rate_count = 0
            self._rate_t0 = now

        if not self.t:
            return
        t_max = self.t[-1]
        t_min = t_max - self.max_seconds
        while self.t and self.t[0] < t_min:
            self.t.popleft()
            self.x.popleft()
            self.y.popleft()
            self.z.popleft()

    def window_arrays(self, window_sec: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        if not self.t:
            empty = np.array([], dtype=float)
            return empty, empty, empty, empty

        t_arr = np.asarray(self.t, dtype=float)
        x_arr = np.asarray(self.x, dtype=float)
        y_arr = np.asarray(self.y, dtype=float)
        z_arr = np.asarray(self.z, dtype=float)
        t_min = t_arr[-1] - window_sec
        mask = t_arr >= t_min
        return t_arr[mask], x_arr[mask], y_arr[mask], z_arr[mask]


class ImuViewerWindow(QtWidgets.QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("KK-track IMU Viewer")
        self.resize(1100, 720)

        self._buf = RingBuffer(MAX_BUFFER_SEC)
        self._worker = SerialWorker()
        self._worker.sample.connect(self._on_sample)
        self._worker.status.connect(self._on_status)
        self._worker.error.connect(self._on_error)
        self._worker.connected.connect(self._on_connected)
        self._worker.stopped.connect(self._on_stopped)
        self._connected = False

        pg.setConfigOptions(antialias=True, background="#1e1e1e", foreground="#d4d4d4")

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)

        root.addLayout(self._build_toolbar())

        self._value_label = QtWidgets.QLabel("Yaw: —   Pitch: —   Roll: —   |   0.0 Hz   |   idle")
        self._value_label.setFont(QtGui.QFont("Consolas", 11))
        root.addWidget(self._value_label)

        self._plot_widget = pg.GraphicsLayoutWidget()
        root.addWidget(self._plot_widget, stretch=1)

        self._plots = []
        self._curves = []
        labels = [
            ("Yaw", "#4fc3f7"),
            ("Pitch", "#81c784"),
            ("Roll", "#ffb74d"),
        ]
        for i, (title, color) in enumerate(labels):
            p = self._plot_widget.addPlot(row=i, col=0, title=title)
            p.showGrid(x=True, y=True, alpha=0.25)
            p.setLabel("left", "deg")
            if i < len(labels) - 1:
                p.getAxis("bottom").setStyle(showValues=False)
            else:
                p.setLabel("bottom", "time", units="s")
            p.setYRange(-45, 45)
            p.enableAutoRange(axis="y", enable=False)
            curve = p.plot(pen=pg.mkPen(color, width=2))
            self._plots.append(p)
            self._curves.append(curve)

        self._overlay_plot = self._plot_widget.addPlot(row=3, col=0, title="Overlay (X/Y/Z)")
        self._overlay_plot.showGrid(x=True, y=True, alpha=0.25)
        self._overlay_plot.setLabel("left", "deg")
        self._overlay_plot.setLabel("bottom", "time", units="s")
        self._overlay_plot.setYRange(-45, 45)
        self._overlay_plot.addLegend(offset=(10, 10))
        self._overlay_curves = [
            self._overlay_plot.plot(pen=pg.mkPen("#4fc3f7", width=2), name="Yaw"),
            self._overlay_plot.plot(pen=pg.mkPen("#81c784", width=2), name="Pitch"),
            self._overlay_plot.plot(pen=pg.mkPen("#ffb74d", width=2), name="Roll"),
        ]

        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._refresh_plots)
        self._timer.start(int(1000 / REFRESH_HZ))

        self._refresh_ports()
        self._on_status("Select COM port and Connect")

    def _build_toolbar(self) -> QtWidgets.QHBoxLayout:
        bar = QtWidgets.QHBoxLayout()

        bar.addWidget(QtWidgets.QLabel("Port:"))
        self._port_combo = QtWidgets.QComboBox()
        self._port_combo.setMinimumWidth(140)
        bar.addWidget(self._port_combo)

        self._refresh_btn = QtWidgets.QPushButton("Refresh")
        self._refresh_btn.clicked.connect(self._refresh_ports)
        bar.addWidget(self._refresh_btn)

        bar.addWidget(QtWidgets.QLabel("Baud:"))
        self._baud_combo = QtWidgets.QComboBox()
        for b in ("115200", "921600", "460800", "230400", "57600"):
            self._baud_combo.addItem(b)
        bar.addWidget(self._baud_combo)

        bar.addWidget(QtWidgets.QLabel("Window:"))
        self._window_spin = QtWidgets.QDoubleSpinBox()
        self._window_spin.setRange(1.0, 60.0)
        self._window_spin.setValue(WINDOW_SEC_DEFAULT)
        self._window_spin.setSuffix(" s")
        bar.addWidget(self._window_spin)

        bar.addWidget(QtWidgets.QLabel("Y range:"))
        self._yrange_spin = QtWidgets.QDoubleSpinBox()
        self._yrange_spin.setRange(5.0, 180.0)
        self._yrange_spin.setValue(45.0)
        self._yrange_spin.setSuffix(" deg")
        bar.addWidget(self._yrange_spin)

        bar.addStretch(1)

        self._connect_btn = QtWidgets.QPushButton("Connect")
        self._connect_btn.clicked.connect(self._toggle_connect)
        bar.addWidget(self._connect_btn)

        self._clear_btn = QtWidgets.QPushButton("Clear")
        self._clear_btn.clicked.connect(self._clear_data)
        bar.addWidget(self._clear_btn)

        return bar

    def _refresh_ports(self) -> None:
        current = self._port_combo.currentText()
        self._port_combo.clear()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if not ports:
            self._port_combo.addItem("(no ports)")
        else:
            for p in ports:
                self._port_combo.addItem(p)
            if current in ports:
                self._port_combo.setCurrentText(current)

    def _toggle_connect(self) -> None:
        if self._connected:
            self._worker.stop()
            return

        port = self._port_combo.currentText()
        if not port or port.startswith("("):
            self._on_error("No COM port selected")
            return

        baud = int(self._baud_combo.currentText())
        self._connect_btn.setEnabled(False)
        self._worker.start(port, baud)

    def _clear_data(self) -> None:
        self._buf.clear()
        for c in self._curves + self._overlay_curves:
            c.setData([], [])

    def _on_sample(self, s: ImuSample) -> None:
        self._buf.push(s)

    def _on_status(self, msg: str) -> None:
        parts = self._value_label.text().split("|")
        if len(parts) >= 3:
            self._value_label.setText(f"{parts[0].strip()} | {parts[1].strip()} | {msg}")
        else:
            self._value_label.setText(f"X: —   Y: —   Z: —   |   0.0 Hz   |   {msg}")

    def _on_error(self, msg: str) -> None:
        self._connected = False
        self._connect_btn.setEnabled(True)
        self._connect_btn.setText("Connect")
        self._on_status(f"ERROR: {msg}")
        QtWidgets.QMessageBox.warning(self, "Serial", msg)

    def _on_connected(self) -> None:
        self._connected = True
        self._connect_btn.setEnabled(True)
        self._connect_btn.setText("Disconnect")

    def _on_stopped(self) -> None:
        self._connected = False
        self._connect_btn.setEnabled(True)
        self._connect_btn.setText("Connect")
        self._on_status("Disconnected")

    def _refresh_plots(self) -> None:
        window = self._window_spin.value()
        yrange = self._yrange_spin.value()
        t, x, y, z = self._buf.window_arrays(window)

        for plot in self._plots + [self._overlay_plot]:
            plot.setYRange(-yrange, yrange)

        if t.size == 0:
            return

        self._curves[0].setData(t, x)
        self._curves[1].setData(t, y)
        self._curves[2].setData(t, z)
        self._overlay_curves[0].setData(t, x)
        self._overlay_curves[1].setData(t, y)
        self._overlay_curves[2].setData(t, z)

        for plot in self._plots + [self._overlay_plot]:
            plot.setXRange(max(0.0, t[-1] - window), max(window, t[-1]), padding=0.02)

        last = self._buf._last
        if last:
            status = "streaming" if self._connected else "idle"
            self._value_label.setText(
                f"Yaw: {last.x:+.2f}°   Pitch: {last.y:+.2f}°   Roll: {last.z:+.2f}°   |   "
                f"{self._buf.hz:.1f} Hz   |   {status}"
            )

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self._worker.stop()
        super().closeEvent(event)


def main() -> int:
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    win = ImuViewerWindow()
    win.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
