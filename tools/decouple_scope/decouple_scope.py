#!/usr/bin/env python3
"""KK-track 轴解耦示波器 — 实时解析 TX 串口 [DEC] 流并绘制波形。

数据来源：固件 link_config.h 中 KK_DBG_DECOUPLE=1 时，TX 串口每 ~50ms 输出一行：
  W (176931) KK.TX: [DEC] gY=31 gP=162 | rawY=-6.8 rawP=9.3 | geoY=-7.4 geoP=9.3 \
      | outY=-8.3 outP=9.3 | supY=1.00 supP=0.00

用法：
  1. 先关掉占用该 COM 口的 `pio device monitor`（串口不能两个程序同时开）。
  2. 选 COM 口 -> 点 Start：开始读取+实时绘制。
  3. 做完动作点 Stop：自动把整段数据写入 logs/dec_<时间戳>.txt（制表符分隔，可直接发回分析）。

四块波形：
  Yaw   : rawY/geoY/outY  —— 看纯转头时 outY 是否被冻结（贴 ref，不跟 geoY 漂）
  Pitch : rawP/geoP/outP  —— 看纯点头时 outP 残留
  Gyro  : gY/gP           —— 主观驱动轴（谁在转）
  Sup   : supY/supP       —— xdec 抑制量 0..1（对侧主导时升高）
"""

from __future__ import annotations

import os
import re
import sys
import threading
import time
from collections import deque
from datetime import datetime
from typing import Deque, Dict, List, Optional

import numpy as np
import pyqtgraph as pg
import serial
import serial.tools.list_ports
from PyQt5 import QtCore, QtGui, QtWidgets

BAUD_DEFAULT = 115200
WINDOW_SEC_DEFAULT = 12.0
REFRESH_HZ = 30
MAX_PLOT_SEC = 60.0  # 绘图滚动窗最多保留秒数（完整记录另存，不受此限）

# 解析顺序固定，决定存盘列顺序
CHANNELS: List[str] = [
    "gY", "gP",
    "rawY", "rawP",
    "geoY", "geoP",
    "outY", "outP",
    "supY", "supP",
]

KV_RE = re.compile(r"([a-zA-Z]+)=(-?\d+(?:\.\d+)?)")
TS_RE = re.compile(r"\((\d+)\)")

# 每通道合理上限：串口偶发半行/乱码会拼出离谱大数（曾把上千灌进 pitch，撑爆自动量程
# 使真实 ±10° 被压成底部平线）。解析后逐通道剔除越界/非有限值，单通道坏值不污染整帧。
CHAN_LIMITS: Dict[str, float] = {
    "gY": 4000.0, "gP": 4000.0,
    "rawY": 400.0, "rawP": 400.0,
    "geoY": 400.0, "geoP": 400.0,
    "outY": 400.0, "outP": 400.0,
    "supY": 2.0, "supP": 2.0,
}

# 每块图：标题、(通道, 颜色, 线宽) 列表、Y 轴标签、固定 Y 范围(None=自动)
PANELS = [
    ("Yaw (deg)", [("rawY", "#555555", 1), ("geoY", "#ffb74d", 1), ("outY", "#4fc3f7", 2)], "deg", None),
    ("Pitch (deg)", [("rawP", "#555555", 1), ("geoP", "#ffb74d", 1), ("outP", "#81c784", 2)], "deg", None),
    ("Gyro (dps)", [("gY", "#4fc3f7", 1), ("gP", "#81c784", 1)], "dps", None),
    ("Suppress 0..1", [("supY", "#4fc3f7", 2), ("supP", "#81c784", 2)], "", (-0.05, 1.05)),
]


class SerialWorker(QtCore.QObject):
    sample = QtCore.pyqtSignal(object)  # (t_rel: float, values: dict)
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
            if "[DEC]" not in line:
                continue

            pairs = dict(KV_RE.findall(line.split("[DEC]", 1)[1]))
            if not pairs:
                continue
            values: Dict[str, float] = {}
            for k in CHANNELS:
                if k not in pairs:
                    continue
                try:
                    fv = float(pairs[k])
                except ValueError:
                    continue
                # 非有限或超出合理量程：丢弃该通道，避免污染自动量程/曲线
                if fv != fv or abs(fv) > CHAN_LIMITS[k]:
                    continue
                values[k] = fv
            if not values:
                continue

            ts = TS_RE.search(line)
            t_ms = float(ts.group(1)) if ts else time.monotonic() * 1000.0
            if t0 is None:
                t0 = t_ms
            t_rel = (t_ms - t0) / 1000.0
            # 设备毫秒可能回绕/重置，回退到本地时基
            if t_rel < 0:
                t0 = t_ms
                t_rel = 0.0

            self.sample.emit((t_rel, values))

        self.stop()


class ChannelStore:
    """绘图滚动窗 + 完整记录（供存盘）。"""

    def __init__(self, max_seconds: float) -> None:
        self.max_seconds = max_seconds
        self.t: Deque[float] = deque()
        self.data: Dict[str, Deque[float]] = {c: deque() for c in CHANNELS}
        # 完整记录
        self.rec_t: List[float] = []
        self.rec: Dict[str, List[float]] = {c: [] for c in CHANNELS}
        self.last: Dict[str, float] = {}
        self._rate_count = 0
        self._rate_t0 = time.monotonic()
        self.hz = 0.0

    def clear(self) -> None:
        self.t.clear()
        for d in self.data.values():
            d.clear()
        self.rec_t.clear()
        for d in self.rec.values():
            d.clear()
        self.last = {}
        self._rate_count = 0
        self._rate_t0 = time.monotonic()
        self.hz = 0.0

    def push(self, t_rel: float, values: Dict[str, float]) -> None:
        self.t.append(t_rel)
        self.rec_t.append(t_rel)
        for c in CHANNELS:
            v = values.get(c, float("nan"))
            self.data[c].append(v)
            self.rec[c].append(v)
        self.last.update(values)

        self._rate_count += 1
        now = time.monotonic()
        dt = now - self._rate_t0
        if dt >= 1.0:
            self.hz = self._rate_count / dt
            self._rate_count = 0
            self._rate_t0 = now

        t_min = self.t[-1] - self.max_seconds
        while self.t and self.t[0] < t_min:
            self.t.popleft()
            for d in self.data.values():
                d.popleft()

    def window(self, channel: str, window_sec: float):
        if not self.t:
            empty = np.array([], dtype=float)
            return empty, empty
        t_arr = np.asarray(self.t, dtype=float)
        v_arr = np.asarray(self.data[channel], dtype=float)
        mask = t_arr >= (t_arr[-1] - window_sec)
        return t_arr[mask], v_arr[mask]

    def dump(self, path: str) -> int:
        cols = ["t_s"] + CHANNELS
        with open(path, "w", encoding="utf-8") as f:
            f.write("\t".join(cols) + "\n")
            for i, t in enumerate(self.rec_t):
                row = [f"{t:.3f}"]
                for c in CHANNELS:
                    v = self.rec[c][i]
                    row.append("" if v != v else f"{v:.3f}")  # NaN -> 空
                f.write("\t".join(row) + "\n")
        return len(self.rec_t)


class ScopeWindow(QtWidgets.QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("KK-track 解耦示波器 (decouple scope)")
        self.resize(1180, 820)

        self._store = ChannelStore(MAX_PLOT_SEC)
        self._worker = SerialWorker()
        self._worker.sample.connect(self._on_sample)
        self._worker.status.connect(self._on_status)
        self._worker.error.connect(self._on_error)
        self._worker.connected.connect(self._on_connected)
        self._worker.stopped.connect(self._on_stopped)
        self._running = False

        pg.setConfigOptions(antialias=True, background="#1e1e1e", foreground="#d4d4d4")

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.addLayout(self._build_toolbar())

        self._value_label = QtWidgets.QLabel("idle")
        self._value_label.setFont(QtGui.QFont("Consolas", 10))
        root.addWidget(self._value_label)

        self._plot_widget = pg.GraphicsLayoutWidget()
        root.addWidget(self._plot_widget, stretch=1)

        self._curves: Dict[str, pg.PlotDataItem] = {}
        self._plots: List[pg.PlotItem] = []
        for i, (title, chans, ylabel, yrange) in enumerate(PANELS):
            p = self._plot_widget.addPlot(row=i, col=0, title=title)
            p.showGrid(x=True, y=True, alpha=0.25)
            p.setLabel("left", ylabel)
            p.addLegend(offset=(10, 5))
            if i < len(PANELS) - 1:
                p.getAxis("bottom").setStyle(showValues=False)
            else:
                p.setLabel("bottom", "time", units="s")
            if yrange is not None:
                p.setYRange(*yrange)
                p.enableAutoRange(axis="y", enable=False)
            if i > 0:
                p.setXLink(self._plots[0])
            for name, color, width in chans:
                self._curves[name] = p.plot(pen=pg.mkPen(color, width=width), name=name)
            self._plots.append(p)

        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._refresh_plots)
        self._timer.start(int(1000 / REFRESH_HZ))

        self._refresh_ports()
        self._on_status("选 COM 口 -> Start")

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
        self._window_spin.setRange(2.0, MAX_PLOT_SEC)
        self._window_spin.setValue(WINDOW_SEC_DEFAULT)
        self._window_spin.setSuffix(" s")
        bar.addWidget(self._window_spin)

        bar.addStretch(1)

        self._start_btn = QtWidgets.QPushButton("Start")
        self._start_btn.clicked.connect(self._toggle_run)
        bar.addWidget(self._start_btn)

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

    def _toggle_run(self) -> None:
        if self._running:
            # Stop -> 存盘
            self._worker.stop()
            self._save_dump()
            return
        port = self._port_combo.currentText()
        if not port or port.startswith("("):
            self._on_error("未选择 COM 口")
            return
        self._store.clear()
        for c in self._curves.values():
            c.setData([], [])
        baud = int(self._baud_combo.currentText())
        self._start_btn.setEnabled(False)
        self._worker.start(port, baud)

    def _save_dump(self) -> None:
        if not self._store.rec_t:
            self._on_status("无数据，未写文件")
            return
        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
        os.makedirs(log_dir, exist_ok=True)
        path = os.path.join(log_dir, datetime.now().strftime("dec_%Y%m%d_%H%M%S.txt"))
        n = self._store.dump(path)
        self._on_status(f"已写入 {n} 行 -> {path}")
        QtWidgets.QMessageBox.information(self, "已保存", f"{n} 行已写入:\n{path}")

    def _on_sample(self, payload) -> None:
        t_rel, values = payload
        self._store.push(t_rel, values)

    def _on_status(self, msg: str) -> None:
        self._status_msg = msg

    def _on_error(self, msg: str) -> None:
        self._running = False
        self._start_btn.setEnabled(True)
        self._start_btn.setText("Start")
        self._on_status(f"ERROR: {msg}")
        QtWidgets.QMessageBox.warning(self, "Serial", msg)

    def _on_connected(self) -> None:
        self._running = True
        self._start_btn.setEnabled(True)
        self._start_btn.setText("Stop + Save")

    def _on_stopped(self) -> None:
        self._running = False
        self._start_btn.setEnabled(True)
        self._start_btn.setText("Start")

    def _refresh_plots(self) -> None:
        window = self._window_spin.value()
        have = bool(self._store.t)
        for name, curve in self._curves.items():
            t, v = self._store.window(name, window)
            if t.size:
                curve.setData(t, v)
        if have:
            t_last = self._store.t[-1]
            self._plots[0].setXRange(max(0.0, t_last - window), max(window, t_last), padding=0.02)

        last = self._store.last
        state = "REC" if self._running else "idle"
        msg = getattr(self, "_status_msg", "")
        if last:
            self._value_label.setText(
                f"[{state}] {self._store.hz:4.1f}Hz | "
                f"gY={last.get('gY', 0):+5.0f} gP={last.get('gP', 0):+5.0f} | "
                f"geoY={last.get('geoY', 0):+6.1f} outY={last.get('outY', 0):+6.1f} "
                f"(Δ{last.get('geoY', 0) - last.get('outY', 0):+5.1f}) | "
                f"geoP={last.get('geoP', 0):+6.1f} outP={last.get('outP', 0):+6.1f} "
                f"(Δ{last.get('geoP', 0) - last.get('outP', 0):+5.1f}) | "
                f"supY={last.get('supY', 0):.2f} supP={last.get('supP', 0):.2f}   {msg}"
            )
        else:
            self._value_label.setText(f"[{state}] {msg}")

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self._worker.stop()
        super().closeEvent(event)


def main() -> int:
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    win = ScopeWindow()
    win.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
