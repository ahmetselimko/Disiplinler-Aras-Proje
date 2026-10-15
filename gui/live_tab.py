"""Canlı izleme ve komut sekmesi."""

from __future__ import annotations

import math

import numpy as np
import pyqtgraph as pg
from PyQt5 import QtCore, QtWidgets

from pendulum_lab.comm import protocol as P
from pendulum_lab.comm.reader import COLUMNS
from pendulum_lab.comm.transport import SimTransport
from pendulum_lab.recorder import Recorder

from .widgets import MODE_COLORS, MODE_TR, PendulumView, mono_font, spin

C = {name: i for i, name in enumerate(COLUMNS)}


class LiveTab(QtWidgets.QWidget):
    def __init__(self, state, parent=None):
        super().__init__(parent)
        self.state = state
        root = QtWidgets.QHBoxLayout(self)

        # ---- sol: kayan grafikler
        self.glw = pg.GraphicsLayoutWidget()
        self.p_th = self.glw.addPlot(row=0, col=0, title="Sarkaç açısı θ [°]  (0 = dik)")
        self.p_x = self.glw.addPlot(row=1, col=0, title="Araba konumu x [m]")
        self.p_u = self.glw.addPlot(row=2, col=0, title="Kontrol girişi u")
        for pl in (self.p_th, self.p_x, self.p_u):
            pl.showGrid(x=True, y=True, alpha=0.25)
            pl.setClipToView(True)
            pl.setDownsampling(auto=True, mode="peak")
            pl.enableAutoRange(axis="y")
        self.p_x.setXLink(self.p_th)
        self.p_u.setXLink(self.p_th)
        self.p_u.setLabel("bottom", "zaman [s] (0 = şimdi)")
        self.c_th = self.p_th.plot(pen=pg.mkPen("#2980b9", width=2))
        self.c_x = self.p_x.plot(pen=pg.mkPen("#16a085", width=2))
        self.c_u = self.p_u.plot(pen=pg.mkPen("#c0392b", width=1.5))
        self.catch_lines = [pg.InfiniteLine(angle=0, pen=pg.mkPen("#27ae60", style=QtCore.Qt.DashLine))
                            for _ in range(2)]
        for ln in self.catch_lines:
            self.p_th.addItem(ln)
        self.rail_lines = [pg.InfiniteLine(angle=0, pen=pg.mkPen("#c0392b", style=QtCore.Qt.DashLine))
                           for _ in range(2)]
        for ln in self.rail_lines:
            self.p_x.addItem(ln)
        root.addWidget(self.glw, 3)

        # ---- sağ: animasyon + komutlar
        right = QtWidgets.QVBoxLayout()
        root.addLayout(right, 2)
        self.mode_lbl = QtWidgets.QLabel("BAĞLI DEĞİL")
        self.mode_lbl.setAlignment(QtCore.Qt.AlignCenter)
        self.mode_lbl.setMinimumHeight(44)
        self._set_mode_style("#bdc3c7")
        right.addWidget(self.mode_lbl)
        self.view = PendulumView(state.p.rail_half, state.p.l)
        self.view.setMinimumHeight(220)
        right.addWidget(self.view, 2)
        self.values = QtWidgets.QLabel("—")
        self.values.setFont(mono_font())
        right.addWidget(self.values)

        btns = QtWidgets.QGridLayout()
        self.b_home = QtWidgets.QPushButton("KALİBRE ET (Homing)")
        self.b_push = QtWidgets.QPushButton("K'yı Cihaza Gönder")
        self.b_start = QtWidgets.QPushButton("BAŞLAT")
        self.b_stop = QtWidgets.QPushButton("DURDUR")
        self.b_estop = QtWidgets.QPushButton("ACİL STOP  (Esc)")
        self.b_estop.setMinimumHeight(48)
        self.b_estop.setStyleSheet("QPushButton{background:#c0392b;color:white;font-weight:bold;font-size:14px}")
        self.b_start.setStyleSheet("QPushButton{background:#27ae60;color:white;font-weight:bold}")
        self.b_calib = QtWidgets.QPushButton("Açı Sıfırla (calib)")
        self.b_calib.setToolTip("Sarkaç aşağıda ve hareketsizken açı sıfırını yeniden bulur (yalnız IDLE)")
        self.b_kick = QtWidgets.QPushButton("Sarkacı Dürt (yalnız simülasyon)")
        self.b_rec = QtWidgets.QPushButton("● Kaydı Başlat")
        self.b_rec.setCheckable(True)
        btns.addWidget(self.b_home, 0, 0)
        btns.addWidget(self.b_push, 0, 1)
        btns.addWidget(self.b_start, 1, 0)
        btns.addWidget(self.b_stop, 1, 1)
        btns.addWidget(self.b_estop, 2, 0, 1, 2)
        btns.addWidget(self.b_calib, 3, 0)
        btns.addWidget(self.b_rec, 3, 1)
        btns.addWidget(self.b_kick, 4, 0, 1, 2)
        right.addLayout(btns)

        opts = QtWidgets.QHBoxLayout()
        self.xref = spin(0.0, -0.3, 0.3, 3, 0.01, "m")
        self.b_xref = QtWidgets.QPushButton("x_ref gönder")
        self.swing_cb = QtWidgets.QCheckBox("Swing-up açık")
        self.swing_cb.setChecked(True)
        self.window = spin(10.0, 1.0, 60.0, 0, 1, "s")
        self.pause_cb = QtWidgets.QCheckBox("Dondur")
        for w in (QtWidgets.QLabel("Hedef:"), self.xref, self.b_xref, self.swing_cb,
                  QtWidgets.QLabel("Pencere:"), self.window, self.pause_cb):
            opts.addWidget(w)
        right.addLayout(opts)

        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(500)
        self.log.setFont(mono_font())
        right.addWidget(self.log, 1)

        self.b_home.clicked.connect(lambda: state.send("home"))
        self.b_calib.clicked.connect(lambda: state.send("calib"))
        self.b_push.clicked.connect(state.push_to_device)
        self.b_start.clicked.connect(lambda: state.send("start"))
        self.b_stop.clicked.connect(lambda: state.send("stop"))
        self.b_estop.clicked.connect(self.estop)
        self.b_kick.clicked.connect(lambda: state.send("kick", dth=1.0))
        self.b_xref.clicked.connect(self.send_xref)
        self.swing_cb.toggled.connect(self.set_swingup)
        self.b_rec.toggled.connect(self.toggle_record)
        state.log.connect(self.append_log)
        state.connection_changed.connect(self._on_conn)
        state.params_changed.connect(self._on_params)
        self._on_params()
        self._on_conn(False)

    # ------------------------------------------------------------ eylemler
    def set_swingup(self, on: bool):
        self.state.swingup_enabled = on          # bağlı değilken de saklanır, bağlanınca gönderilir
        if self.state.connected:
            self.state.send("set_param", swingup=int(on))

    def send_xref(self):
        self.state.x_ref = self.state.clamp_xref(self.xref.value())
        self.xref.setValue(self.state.x_ref)
        self.state.send("set_param", x_ref=self.state.x_ref)

    def estop(self):
        if self.state.send("estop"):
            self.append_log("!!! ACİL STOP gönderildi")

    def toggle_record(self, on: bool):
        st = self.state
        if on:
            if st.reader is None:
                self.b_rec.setChecked(False)
                st.log.emit("Kayıt için önce bağlanın.")
                return
            path = Recorder.default_path(st.base_dir / "kayitlar")
            st.recorder = Recorder(path, meta={"K": st.K.tolist(), "params": st.p.to_dict()})
            st.reader.sinks.append(st.recorder)
            self.b_rec.setText("■ Kaydı Durdur")
            st.log.emit(f"Kayıt başladı: {path}")
        else:
            if st.recorder is not None:
                if st.reader is not None and st.recorder in st.reader.sinks:
                    st.reader.sinks.remove(st.recorder)
                st.recorder.close()
                st.log.emit(f"Kayıt kaydedildi: {st.recorder.path} ({st.recorder.rows} satır)")
                st.recorder = None
            self.b_rec.setText("● Kaydı Başlat")

    def append_log(self, text: str):
        self.log.appendPlainText(text)

    def handle_message(self, msg):
        if isinstance(msg, P.Ack):
            if not msg.ok:
                self.append_log(f"✗ {msg.cmd}: {msg.msg}")
            elif msg.cmd not in ("set_param",):
                self.append_log(f"✓ {msg.cmd} {msg.msg}".rstrip())
        elif isinstance(msg, P.Event):
            if msg.name == "recorder_error":
                self.append_log(f"!!! Kayıt durduruldu: {msg.data.get('msg')}")
                self.b_rec.setChecked(False)
                return
            extra = " ".join(f"{k}={v}" for k, v in msg.data.items())
            self.append_log(f"● olay: {msg.name} {extra}".rstrip())

    # ------------------------------------------------------------ güncelleme (60 FPS)
    def refresh(self, reader):
        if self.pause_cb.isChecked() or reader is None:
            return
        last = reader.last
        if last is not None:
            mode = last.mode
            self.mode_lbl.setText(MODE_TR.get(mode, mode))
            self._set_mode_style(MODE_COLORS.get(mode, "#bdc3c7"))
            self.view.set_state(last.x, last.th, self.xref.value())
            self.values.setText(
                f"t={last.t:8.3f} s   θ={math.degrees(last.th):+7.2f}°   θ̇={last.dth:+6.2f} rad/s\n"
                f"x={last.x:+.4f} m   ẋ={last.dx:+.3f} m/s   u={last.u:+6.2f}")
        rate = max(reader.rate_hz, 1.0)
        data = reader.buffer.latest(int(self.window.value() * rate * 1.2) + 10)
        if len(data) < 2:
            return
        t = data[:, C["t"]] - data[-1, C["t"]]
        keep = t >= -self.window.value()
        t = t[keep]
        self.c_th.setData(t, np.degrees(data[keep, C["th"]]))
        self.c_x.setData(t, data[keep, C["x"]])
        self.c_u.setData(t, data[keep, C["u"]])
        self.p_th.setXRange(-self.window.value(), 0, padding=0)

    # ------------------------------------------------------------ yardımcılar
    def _set_mode_style(self, color: str):
        self.mode_lbl.setStyleSheet(
            f"QLabel{{background:{color};color:white;font-size:20px;font-weight:bold;border-radius:6px}}")

    def _on_params(self):
        p = self.state.p
        lim = max(0.0, p.rail_half - 0.1)
        self.xref.setRange(-lim, lim)
        self.view.set_geometry(p.rail_half, p.l)
        for ln, v in zip(self.rail_lines, (-p.rail_half, p.rail_half)):
            ln.setValue(v)
        for ln, v in zip(self.catch_lines, (-self.state.catch_deg, self.state.catch_deg)):
            ln.setValue(v)

    def _on_conn(self, connected: bool):
        for b in (self.b_home, self.b_calib, self.b_push, self.b_start, self.b_stop, self.b_estop, self.b_xref,
                  self.b_rec):
            b.setEnabled(connected)
        reader = self.state.reader
        self.b_kick.setEnabled(connected and isinstance(getattr(reader, "transport", None), SimTransport))
        if not connected:
            if self.b_rec.isChecked():
                self.b_rec.setChecked(False)
            self.mode_lbl.setText("BAĞLI DEĞİL")
            self._set_mode_style("#bdc3c7")
