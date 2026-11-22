"""Çevrimdışı analiz: lineer olmayan simülasyon (+animasyon), basamak yanıtı, kök eğrisi."""

from __future__ import annotations

import copy
import math

import numpy as np
import pyqtgraph as pg
from PyQt5 import QtCore, QtWidgets

from pendulum_lab.analysis.linear import StateSpace, rlocus, step_info
from pendulum_lab.analysis.summary import compute_trial_summary
from pendulum_lab.control.lqr import design_lqr
from pendulum_lab.control.supervisor import MODE_CODES, Mode, Supervisor
from pendulum_lab.sim.matlab_compat import (MATLAB_DT, MATLAB_F_MAX, MATLAB_Q, MATLAB_R, MATLAB_RAIL_LIMIT,
                                            force_model,
                                            gain_to_matlab, matlab_reference_params,
                                            simulate_matlab_compatible)
from pendulum_lab.sim.simulator import SensorModel, simulate_closed_loop

from .widgets import PendulumView, mono_font, spin, setup_mode_axis


def pulse_disturbance(t_start: float, force: float, duration: float):
    """t_start anından itibaren duration saniye boyunca force [N] uygulayan bozucu; force = 0 ise None."""
    if not force:
        return None
    return lambda t: force if t_start <= t < t_start + duration else 0.0


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
        self.dx0 = spin(0.0, -2.0, 2.0, 3, 0.05, "m/s")
        self.dx0.setToolTip("Arabanın başlangıç hızı ẋ₀")
        self.dth0 = spin(0.0, -1000.0, 1000.0, 1, 10.0, "°/s")
        self.dth0.setToolTip("Sarkacın başlangıç açısal hızı θ̇₀")
        self.T = spin(15.0, 0.5, 120, 1, 1, "s")
        self.real_cb = QtWidgets.QCheckBox("Gerçekçi sensör (AS5600 12-bit, enkoder, türevle hız)")
        self.real_cb.setChecked(True)
        self.kalman_cb = QtWidgets.QCheckBox("Kalman filtresi ile hız kestirimi")
        self.kalman_cb.setChecked(True)
        self.noise = spin(0.05, 0, 5, 3, 0.01, "°")
        self.delay = QtWidgets.QSpinBox()
        self.delay.setRange(0, 20)
        self.delay.setSuffix(" adım")
        self.swing_cb = QtWidgets.QCheckBox("Swing-up açık")
        self.swing_cb.setChecked(True)
        self.dist_t = spin(0.0, 0, 120, 2, 0.5, "s")
        self.dist_F = spin(0.0, -20, 20, 2, 0.5, "N")
        self.dist_dur = spin(50.0, 1, 2000, 0, 10, "ms")
        self.dist_dur.setToolTip("Dürtme kuvvetinin ne kadar süre uygulanacağı")
        self.dist_at = QtWidgets.QComboBox()
        self.dist_at.addItem("sarkaca", "pole")
        self.dist_at.addItem("arabaya", "cart")
        self.dist_at.setToolTip("sarkaca: ağırlık merkezine yatay kuvvet\n"
                                "arabaya: arabaya yatay kuvvet (Matlab'daki \"Apply push\" ile aynı; "
                                "Matlab uyumlu denemede hep arabaya)")
        self.mismatch = spin(0.0, -50, 50, 0, 5, "%")
        self.mismatch.setToolTip("Gerçek sistemin m ve l değerleri tasarım modelinden bu kadar farklı olsun")
        self.matlab_cb = QtWidgets.QCheckBox("Matlab uyumlu (cart_pendulum_application.m)")
        self.matlab_cb.setToolTip(
            "Mekatronik ekibinin Matlab denemesiyle aynı: sürekli zamanlı LQR, kuvvet girişi (motor yok),\n"
            f"ideal sensör, RK4 dt = {MATLAB_DT * 1000:g} ms; araba ±{MATLAB_RAIL_LIMIT:g} m'yi aşınca veya |θ| ≥ 90° "
            "olunca deneme durur. Swing-up yoktur.")
        self.matlab_ref_cb = QtWidgets.QCheckBox("Matlab'ın kendi ayarlarıyla (parametreler, Q, R)")
        self.matlab_ref_cb.setChecked(True)
        self.matlab_ref_cb.setEnabled(False)
        p_ref = matlab_reference_params()
        self.matlab_ref_cb.setToolTip(
            f"Açık: cart_pendulum_lqr.m'deki değerler (M={p_ref.M:g}, m={p_ref.m:g}, l={p_ref.l:g}, I={p_ref.I:g}, "
            f"b={p_ref.b:g}; Q=diag(25, 4, 32.83, 1), R={MATLAB_R:g}, kuvvet sınırı ±{MATLAB_F_MAX:g} N). "
            "Sonuç Matlab'ın sonucuyla karşılaştırılabilir.\n"
            "Kapalı: projenin parametreleri ve Tasarım sekmesindeki Q, R; K kuvvet girişli model için yeniden "
            "hesaplanır, kuvvet sınırı yoktur.\nBu durumda K Matlab'ınkiyle aynı değildir (yalnız Matlab'ın işaret "
            "kuralıyla gösterilir).")
        self.run_btn = QtWidgets.QPushButton("▶ Simüle Et")
        self.run_btn.setStyleSheet("QPushButton{font-weight:bold;padding:6px}")
        f.addRow(self.matlab_cb)
        f.addRow(self.matlab_ref_cb)
        f.addRow("Başlangıç θ₀", self.th0)
        f.addRow("Başlangıç x₀", self.x0)
        f.addRow("Başlangıç hızı ẋ₀", self.dx0)
        f.addRow("Başl. açısal hız θ̇₀", self.dth0)
        f.addRow("Süre", self.T)
        f.addRow(self.real_cb)
        f.addRow(self.kalman_cb)
        f.addRow("Açı gürültüsü (σ)", self.noise)
        f.addRow("Giriş gecikmesi", self.delay)
        f.addRow(self.swing_cb)
        f.addRow("Dürtme anı", self.dist_t)
        f.addRow("Dürtme kuvveti", self.dist_F)
        f.addRow("Dürtme süresi", self.dist_dur)
        f.addRow("Dürtme yeri", self.dist_at)
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
        self.p_e = self.glw.addPlot(row=3, col=0, title="Enerji E [J]  (0 = dik)")
        self.p_m = self.glw.addPlot(row=4, col=0, title="mod  (3 = swing-up, 4 = LQR, 5 = hata)")
        for pl in (self.p_th, self.p_x, self.p_u, self.p_e, self.p_m):
            pl.showGrid(x=True, y=True, alpha=0.25)
        for pl in (self.p_x, self.p_u, self.p_e, self.p_m):
            pl.setXLink(self.p_th)
        self.p_m.setLabel("bottom", "t [s]")
        self.c_th = self.p_th.plot(pen=pg.mkPen("#2980b9", width=2))
        self.c_x = self.p_x.plot(pen=pg.mkPen("#16a085", width=2))
        self.c_u = self.p_u.plot(pen=pg.mkPen("#c0392b", width=1))
        self.c_e = self.p_e.plot(pen=pg.mkPen("#e67e22", width=1.8))
        self.c_m = self.p_m.plot(pen=pg.mkPen("#8e44ad", width=2), stepMode="left")
        self.p_e.addItem(pg.InfiniteLine(angle=0, pen=pg.mkPen("#27ae60", style=QtCore.Qt.DashLine)))
        setup_mode_axis(self.p_m)

        self.cursor = [pg.InfiniteLine(angle=90, pen=pg.mkPen("#f39c12")) for _ in range(5)]
        for pl, c in zip((self.p_th, self.p_x, self.p_u, self.p_e, self.p_m), self.cursor):
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
        self.matlab_cb.toggled.connect(self._on_matlab_toggled)
        self.play_btn.clicked.connect(self.toggle_play)
        self.slider.valueChanged.connect(self._show_frame)
        state.params_changed.connect(lambda: self.view.set_geometry(state.p.rail_half, state.p.l))

    def _on_matlab_toggled(self, on: bool):
        """Matlab denemesinde olmayan ayarları kapat; θ₀'ı Matlab'ın durma sınırının içine al.
        Dürtme açık kalır ama Matlab'daki "Apply push" gibi hep arabaya uygulanır."""
        for w in (self.real_cb, self.kalman_cb, self.noise, self.delay, self.swing_cb, self.dist_at):
            w.setEnabled(not on)
        self.matlab_ref_cb.setEnabled(on)
        if on:
            self.dist_at.setCurrentIndex(self.dist_at.findData("cart"))
        if on and abs(self.th0.value()) >= 90:
            self.th0.setValue(10.0)

    def _run_matlab(self, mismatch: float):
        """Matlab uyumlu deneme: K, kuvvet girişli model için sürekli LQR ile (cart_pendulum_lqr.m gibi).

        Matlab ayarları seçiliyse model, Q ve R cart_pendulum_lqr.m'dekilerdir (sonuç Matlab'ınkiyle
        karşılaştırılabilir); değilse projenin parametreleri ve Tasarım sekmesindeki Q, R kullanılır.
        Dönüş: (sonuç, bilgi satırları, gerçek sistem parametreleri, kuvvet sınırı [N] veya None)."""
        st = self.state
        th0 = self.th0.value()
        if abs(th0) >= 90:
            QtWidgets.QMessageBox.warning(self, "Simülasyon",
                                          "Matlab uyumlu denemede |θ₀| < 90° olmalı (swing-up yok, deneme hemen durur).")
            return None, None, None, None
        if self.matlab_ref_cb.isChecked():
            design, Q, R, x_ref, f_max = matlab_reference_params(), MATLAB_Q, MATLAB_R, 0.0, MATLAB_F_MAX
            source = "Matlab'ın ayarları (cart_pendulum_lqr.m)"
        else:
            design, Q, R, x_ref, f_max = force_model(st.p), st.Q, st.R, st.x_ref, None
            source = "proje parametreleri + Tasarım Q, R (Matlab'ın K'sı DEĞİL)"
        real = copy.deepcopy(design)
        real.m, real.l = real.m * mismatch, real.l * mismatch
        K, _, _ = design_lqr(design, Q, [[R]])
        K = K.ravel()
        s0 = [self.x0.value(), self.dx0.value(), math.radians(th0), math.radians(self.dth0.value())]
        dist = pulse_disturbance(self.dist_t.value(), self.dist_F.value(), self.dist_dur.value() / 1000)
        r = simulate_matlab_compatible(real, K, s0, self.T.value(), x_ref=x_ref, f_max=f_max,
                                       disturbance=dist)       # arabaya (Matlab'daki "Apply push")
        Km = gain_to_matlab(K)
        reason = {"time": "süre doldu", "rail": f"|x| ≥ {MATLAB_RAIL_LIMIT:g} m", "angle": "|θ| ≥ 90°"}[r.stop_reason]
        head = [f"Matlab uyumlu deneme — bitiş: {reason} (t = {r.t[-1]:.2f} s)",
                f"Ayarlar: {source}",
                "K (proje işaretinde, θ sağa +) = [" + ", ".join(f"{v:.2f}" for v in K) + "]",
                "K (Matlab işaretinde, θ sola +) = [" + ", ".join(f"{v:.2f}" for v in Km) + "]",
                "Giriş kuvvettir [N], " + (f"±{f_max:g} N ile sınırlı." if f_max else "sınır yoktur.")]
        return r, head, real, f_max

    def run(self):
        st = self.state
        k = 1 + self.mismatch.value() / 100
        if self.matlab_cb.isChecked():
            try:
                r, head, real, f_max = self._run_matlab(k)
            except FloatingPointError as e:
                QtWidgets.QMessageBox.warning(self, "Simülasyon", str(e))
                return
            if r is not None:
                self._show_result(r, real, head, u_max=f_max or 1e6, rail_half=MATLAB_RAIL_LIMIT)
            return
        real = copy.deepcopy(st.p)
        real.m, real.l = real.m * k, real.l * k
        sup = Supervisor(st.p, st.K, swingup=copy.deepcopy(st.swingup), catch_deg=st.catch_deg,
                         catch_dth=st.catch_dth, catch_x=st.catch_x, catch_dx=st.catch_dx,
                         release_deg=st.release_deg, swingup_enabled=self.swing_cb.isChecked())
        sup.x_ref = st.x_ref   # cihaza gönderilen hedef konumla aynı (Matlab dalı da kullanıyor)
        sup.start()
        sensor = (SensorModel.for_params(st.p, th_noise_std=math.radians(self.noise.value()))
                  if self.real_cb.isChecked() else None)
        dist = pulse_disturbance(self.dist_t.value(), self.dist_F.value(), self.dist_dur.value() / 1000)
        s0 = [self.x0.value(), self.dx0.value(), math.radians(self.th0.value()), math.radians(self.dth0.value())]
        QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
        try:
            r = simulate_closed_loop(real, sup, s0,
                                     self.T.value(), dt=st.Ts, sensor=sensor, delay_steps=self.delay.value(),
                                     disturbance=dist, use_kalman=self.kalman_cb.isChecked(),
                                     disturbance_target=self.dist_at.currentData())
        except FloatingPointError as e:
            QtWidgets.QMessageBox.warning(self, "Simülasyon", str(e))
            return
        finally:
            QtWidgets.QApplication.restoreOverrideCursor()
        self._show_result(r, real, [], u_max=st.p.u_max, rail_half=st.p.rail_half)

    def _show_result(self, r, real, head_lines, u_max, rail_half):
        self.result = r
        th_deg = np.degrees(np.arctan2(np.sin(r.th), np.cos(r.th)))
        self.c_th.setData(r.t, th_deg)
        self.c_x.setData(r.t, r.x)
        self.c_u.setData(r.t, r.u)
        E = 0.5 * real.J * (r.s[:, 3] ** 2) + real.m * real.g * real.l * (np.cos(r.th) - 1.0)
        self.c_e.setData(r.t, E)
        codes = np.array([MODE_CODES.get(Mode(m), -1) if m else -1 for m in r.mode], dtype=float)
        self.c_m.setData(r.t, codes)
        summary = compute_trial_summary(
            r.t, r.x, r.th, r.u, modes=r.mode,
            u_max=u_max, rail_half=rail_half
        )
        self.info.setText("\n".join(head_lines + summary.format_lines(rail_half=rail_half, u_max=u_max)))
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
