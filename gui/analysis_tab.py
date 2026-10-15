"""Çevrimdışı analiz: lineer olmayan simülasyon (+animasyon), basamak yanıtı, kök eğrisi."""

from __future__ import annotations

import copy
import math

import numpy as np
import pyqtgraph as pg
from PyQt5 import QtCore, QtWidgets

from pendulum_lab.analysis.linear import StateSpace, rlocus, step_info
from pendulum_lab.control.supervisor import MODE_CODES, Mode, Supervisor
from pendulum_lab.sim.simulator import SensorModel, simulate_closed_loop

from .widgets import PendulumView, mono_font, spin


class AnalysisTab(QtWidgets.QTabWidget):
    def __init__(self, state, parent=None):
        super().__init__(parent)
        self.state = state
        self.addTab(SimulationPage(state), "Lineer olmayan simülasyon")
        self.addTab(StepPage(state), "Basamak yanıtı (step)")
        self.addTab(RootLocusPage(state), "Kök eğrisi (rlocus)")


class SimulationPage(QtWidgets.QWidget):
    def __init__(self, state):
        super().__init__()
        self.state = state
        self.result = None
        root = QtWidgets.QHBoxLayout(self)

        form_w = QtWidgets.QWidget()
        form_w.setMaximumWidth(330)
        f = QtWidgets.QFormLayout(form_w)
        self.th0 = spin(180.0, -180, 180, 1, 1, "°")
        self.th0.setToolTip("0 = dik, 180 = aşağıda asılı")
        self.x0 = spin(0.0, -0.3, 0.3, 3, 0.01, "m")
        self.T = spin(15.0, 0.5, 120, 1, 1, "s")
        self.real_cb = QtWidgets.QCheckBox("Gerçekçi sensör (AS5600 12-bit, enkoder, türevle hız)")
        self.real_cb.setChecked(True)
        self.noise = spin(0.05, 0, 5, 3, 0.01, "°")
        self.delay = QtWidgets.QSpinBox()
        self.delay.setRange(0, 20)
        self.delay.setSuffix(" adım")
        self.swing_cb = QtWidgets.QCheckBox("Swing-up açık")
        self.swing_cb.setChecked(True)
        self.dist_t = spin(0.0, 0, 120, 2, 0.5, "s")
        self.dist_F = spin(0.0, -20, 20, 2, 0.5, "N")
        self.mismatch = spin(0.0, -50, 50, 0, 5, "%")
        self.mismatch.setToolTip("Gerçek sistemin m ve l değerleri tasarım modelinden bu kadar farklı olsun")
        self.run_btn = QtWidgets.QPushButton("▶ Simüle Et")
        self.run_btn.setStyleSheet("QPushButton{font-weight:bold;padding:6px}")
        f.addRow("Başlangıç θ₀", self.th0)
        f.addRow("Başlangıç x₀", self.x0)
        f.addRow("Süre", self.T)
        f.addRow(self.real_cb)
        f.addRow("Açı gürültüsü (σ)", self.noise)
        f.addRow("Giriş gecikmesi", self.delay)
        f.addRow(self.swing_cb)
        f.addRow("Dürtme anı", self.dist_t)
        f.addRow("Dürtme kuvveti (50 ms)", self.dist_F)
        f.addRow("Model uyumsuzluğu", self.mismatch)
        f.addRow(self.run_btn)
        self.info = QtWidgets.QLabel("")
        self.info.setWordWrap(True)
        self.info.setFont(mono_font())
        f.addRow(self.info)
        root.addWidget(form_w)

        mid = QtWidgets.QVBoxLayout()
        root.addLayout(mid, 1)
        self.glw = pg.GraphicsLayoutWidget()
        self.p_th = self.glw.addPlot(row=0, col=0, title="θ [°]")
        self.p_x = self.glw.addPlot(row=1, col=0, title="x [m]")
        self.p_u = self.glw.addPlot(row=2, col=0, title="u")
        self.p_m = self.glw.addPlot(row=3, col=0, title="mod  (3 = swing-up, 4 = LQR, 5 = hata)")
        for pl in (self.p_th, self.p_x, self.p_u, self.p_m):
            pl.showGrid(x=True, y=True, alpha=0.25)
        for pl in (self.p_x, self.p_u, self.p_m):
            pl.setXLink(self.p_th)
        self.p_m.setLabel("bottom", "t [s]")
        self.c_th = self.p_th.plot(pen=pg.mkPen("#2980b9", width=2))
        self.c_x = self.p_x.plot(pen=pg.mkPen("#16a085", width=2))
        self.c_u = self.p_u.plot(pen=pg.mkPen("#c0392b", width=1))
        self.c_m = self.p_m.plot(pen=pg.mkPen("#8e44ad", width=2), stepMode="left")
        self.cursor = [pg.InfiniteLine(angle=90, pen=pg.mkPen("#f39c12")) for _ in range(4)]
        for pl, c in zip((self.p_th, self.p_x, self.p_u, self.p_m), self.cursor):
            pl.addItem(c)
        mid.addWidget(self.glw)

        right = QtWidgets.QVBoxLayout()
        root.addLayout(right, 1)
        self.view = PendulumView(state.p.rail_half, state.p.l)
        right.addWidget(self.view, 1)
        pl = QtWidgets.QHBoxLayout()
        self.play_btn = QtWidgets.QPushButton("▶ Oynat")
        self.speed = QtWidgets.QComboBox()
        self.speed.addItems(["0.25x", "0.5x", "1x", "2x"])
        self.speed.setCurrentText("1x")
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        pl.addWidget(self.play_btn)
        pl.addWidget(self.speed)
        pl.addWidget(self.slider, 1)
        right.addLayout(pl)

        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(16)
        self.timer.timeout.connect(self._advance)
        self.run_btn.clicked.connect(self.run)
        self.play_btn.clicked.connect(self.toggle_play)
        self.slider.valueChanged.connect(self._show_frame)
        state.params_changed.connect(lambda: self.view.set_geometry(state.p.rail_half, state.p.l))

    def run(self):
        st = self.state
        real = copy.deepcopy(st.p)
        k = 1 + self.mismatch.value() / 100
        real.m, real.l = real.m * k, real.l * k
        sup = Supervisor(st.p, st.K, swingup=copy.deepcopy(st.swingup), catch_deg=st.catch_deg,
                         release_deg=st.release_deg, swingup_enabled=self.swing_cb.isChecked())
        sup.start()
        sensor = SensorModel(th_noise_std=math.radians(self.noise.value())) if self.real_cb.isChecked() else None
        t_d, F_d = self.dist_t.value(), self.dist_F.value()
        dist = (lambda t: F_d if t_d <= t < t_d + 0.05 else 0.0) if F_d else None
        QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
        try:
            r = simulate_closed_loop(real, sup, [self.x0.value(), 0, math.radians(self.th0.value()), 0],
                                     self.T.value(), dt=st.Ts, sensor=sensor, delay_steps=self.delay.value(),
                                     disturbance=dist)
        except FloatingPointError as e:
            QtWidgets.QMessageBox.warning(self, "Simülasyon", str(e))
            return
        finally:
            QtWidgets.QApplication.restoreOverrideCursor()
        self.result = r
        th_deg = np.degrees(np.arctan2(np.sin(r.th), np.cos(r.th)))
        self.c_th.setData(r.t, th_deg)
        self.c_x.setData(r.t, r.x)
        self.c_u.setData(r.t, r.u)
        codes = np.array([MODE_CODES.get(Mode(m), -1) if m else -1 for m in r.mode], dtype=float)
        self.c_m.setData(r.t, codes)
        bal = [i for i, m in enumerate(r.mode) if m == "BALANCE"]
        final = r.mode[-1]
        txt = [f"Son mod: {final}", f"max |x| = {np.abs(r.x).max():.3f} m  (ray ±{st.p.rail_half} m)",
               f"max |u| = {np.abs(r.u).max():.2f}"]
        if bal:
            txt.insert(1, f"LQR ilk devreye giriş: t = {r.t[bal[0]]:.2f} s")
        if final == "BALANCE":
            txt.append(f"son |θ| = {abs(th_deg[-1]):.2f}°")
        self.info.setText("\n".join(txt))
        self.slider.setRange(0, len(r.t) - 1)
        self.slider.setValue(0)
        self._show_frame(0)

    def toggle_play(self):
        if self.result is None:
            return
        if self.timer.isActive():
            self.timer.stop()
            self.play_btn.setText("▶ Oynat")
        else:
            if self.slider.value() >= self.slider.maximum():
                self.slider.setValue(0)
            self.timer.start()
            self.play_btn.setText("❚❚ Duraklat")

    def _advance(self):
        r = self.result
        step = max(1, round(0.016 * float(self.speed.currentText()[:-1]) / (r.t[1] - r.t[0])))
        v = self.slider.value() + step
        if v >= self.slider.maximum():
            v = self.slider.maximum()
            self.toggle_play()
        self.slider.setValue(v)

    def _show_frame(self, i: int):
        r = self.result
        if r is None or i >= len(r.t):
            return
        self.view.set_state(r.s[i, 0], r.s[i, 2], 0.0)
        for c in self.cursor:
            c.setValue(r.t[i])


class StepPage(QtWidgets.QWidget):
    def __init__(self, state):
        super().__init__()
        self.state = state
        lay = QtWidgets.QVBoxLayout(self)
        top = QtWidgets.QHBoxLayout()
        self.amp = spin(0.1, -0.3, 0.3, 3, 0.01, "m")
        self.T = spin(6.0, 0.5, 60, 1, 0.5, "s")
        self.btn = QtWidgets.QPushButton("Hesapla")
        for w in (QtWidgets.QLabel("Araba konum referansı basamağı:"), self.amp, QtWidgets.QLabel("Süre:"),
                  self.T, self.btn):
            top.addWidget(w)
        top.addStretch(1)
        lay.addLayout(top)
        self.plot_x = pg.PlotWidget(title="x(t) — kapalı çevrim (doğrusallaştırılmış model, ESP32 gibi Ts'de ayrık LQR)")
        self.plot_th = pg.PlotWidget(title="θ(t) [°]")
        for pw in (self.plot_x, self.plot_th):
            pw.showGrid(x=True, y=True, alpha=0.3)
        self.plot_th.setXLink(self.plot_x)
        self.plot_th.setLabel("bottom", "t [s]")
        self.cx = self.plot_x.plot(pen=pg.mkPen("#16a085", width=2))
        self.cth = self.plot_th.plot(pen=pg.mkPen("#2980b9", width=2))
        lay.addWidget(self.plot_x)
        lay.addWidget(self.plot_th)
        self.info = QtWidgets.QLabel("")
        self.info.setFont(mono_font())
        lay.addWidget(self.info)
        self.btn.clicked.connect(self.compute)
        state.gains_changed.connect(self.compute)
        self.compute()

    def compute(self):
        st = self.state
        t, y = StateSpace.from_params(st.p).closed_loop_step_discrete(st.K, st.Ts, self.T.value(), self.amp.value())
        self.cx.setData(t, y[:, 0])
        self.cth.setData(t, np.degrees(y[:, 1]))
        i = step_info(t, y[:, 0])
        if "overshoot_pct" in i:
            self.info.setText(
                f"aşım = %{i['overshoot_pct']:.1f}   ters yön sapması = %{i['undershoot_pct']:.1f} "
                f"(araba önce geri gider: minimum faz dışı)   yükselme = {i['rise_time']:.2f} s   "
                f"%2 oturma = {i['settling_time']:.2f} s   max|θ| = {np.degrees(np.abs(y[:, 1]).max()):.2f}°")


class RootLocusPage(QtWidgets.QWidget):
    def __init__(self, state):
        super().__init__()
        self.state = state
        lay = QtWidgets.QVBoxLayout(self)
        top = QtWidgets.QHBoxLayout()
        self.neg_cb = QtWidgets.QCheckBox("Negatif kazanç (k < 0)")
        self.neg_cb.setChecked(True)
        self.btn = QtWidgets.QPushButton("Çiz")
        top.addWidget(QtWidgets.QLabel("1 + k·G(s) = 0,  G(s) = θ(s)/u(s)  — yalnız θ'ya oransal geri besleme"))
        top.addWidget(self.neg_cb)
        top.addWidget(self.btn)
        top.addStretch(1)
        lay.addLayout(top)
        self.pw = pg.PlotWidget()
        self.pw.showGrid(x=True, y=True, alpha=0.3)
        self.pw.setLabel("bottom", "Re")
        self.pw.setLabel("left", "Im")
        self.pw.addItem(pg.InfiniteLine(angle=90, pos=0, pen=pg.mkPen("#7f8c8d")))
        self.locus = pg.ScatterPlotItem(size=3, brush=pg.mkBrush("#2980b9"), pen=None)
        self.poles = pg.ScatterPlotItem(symbol="x", size=14, pen=pg.mkPen("#c0392b", width=2))
        self.zeros = pg.ScatterPlotItem(symbol="o", size=12, brush=None, pen=pg.mkPen("#27ae60", width=2))
        for it in (self.locus, self.poles, self.zeros):
            self.pw.addItem(it)
        lay.addWidget(self.pw)
        self.info = QtWidgets.QLabel("")
        self.info.setWordWrap(True)
        self.info.setFont(mono_font())
        lay.addWidget(self.info)
        self.btn.clicked.connect(self.compute)
        self.neg_cb.toggled.connect(self.compute)
        state.params_changed.connect(self.compute)
        self.compute()

    def compute(self):
        num, den = StateSpace.from_params(self.state.p).tf(output=1)
        sign = -1 if self.neg_cb.isChecked() else 1
        gains, roots = rlocus(num, den, sign * np.concatenate([[0], np.logspace(-3, 4, 600)]))
        rr = roots.ravel()
        self.locus.setData(rr.real, rr.imag)
        pz = np.roots(den)
        z = np.roots(num) if len(num) > 1 else np.array([])
        self.poles.setData(pz.real, pz.imag)
        self.zeros.setData(z.real, z.imag)
        stable_any = np.any(np.all(roots.real < 0, axis=1))
        self.info.setText(
            f"pay = {np.array2string(num, precision=3)}   payda = {np.array2string(den, precision=3)}\n"
            + ("Bu yönde bir k ile tüm kökler sol yarı düzleme alınabiliyor." if stable_any else
               "Hiçbir k değeri tüm kökleri sol yarı düzleme alamıyor: s=0'daki kutup (araba konumu) yerinde "
               "kalıyor, yani yalnız θ'ya geri besleme arabayı kontrol etmiyor ve araba kayıp gider → "
               "x, ẋ, θ, θ̇'nın hepsini kullanan LQR (durum geri beslemesi) seçildi."))
