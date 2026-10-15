"""Model parametreleri + LQR tasarım sekmesi."""

from __future__ import annotations

import numpy as np
import pyqtgraph as pg
from PyQt5 import QtWidgets

from pendulum_lab.analysis.linear import StateSpace
from pendulum_lab.control.lqr import design_lqr
from pendulum_lab.control.swingup import EnergySwingUp
from pendulum_lab.export import write_gains_header
from pendulum_lab.model.motor import max_cart_force, max_cart_speed
from pendulum_lab.model.params import (INPUT_MODES, MotorParams, PendulumParams, load_params,
                                       pendulum_from_geometry, save_params)

from .widgets import mono_font, spin

PARAM_FIELDS = [  # alan, etiket, birim, min, max, ondalık
    ("M", "Araba kütlesi M", "kg", 0.01, 50, 3),
    ("m", "Sarkaç kütlesi m", "kg", 0.001, 10, 4),
    ("l", "Pivot → ağırlık merkezi l", "m", 0.01, 3, 4),
    ("I", "Ağırlık merkezi ataleti I", "kg·m²", 0, 1, 6),
    ("b", "Araba sürtünmesi b", "N·s/m", 0, 200, 4),
    ("c", "Pivot sürtünmesi c", "N·m·s/rad", 0, 1, 5),
    ("rail_half", "Ray yarı boyu", "m", 0.05, 5, 3),
    ("u_max", "Giriş sınırı u_max", "", 0.1, 200, 2),
]
MOTOR_FIELDS = [
    ("R", "Sargı direnci R", "Ω", 0.01, 100, 3),
    ("Kt", "Tork sabiti Kt", "N·m/A", 1e-5, 5, 5),
    ("Ke", "Zıt EMK sabiti Ke", "V·s/rad", 1e-5, 5, 5),
    ("gear", "Redüktör oranı", "", 1, 1000, 1),
    ("r_pulley", "Kasnak yarıçapı", "m", 0.001, 0.2, 5),
    ("V_max", "Besleme gerilimi", "V", 1, 60, 1),
    ("J_rotor", "Rotor ataleti (motor mili)", "kg·m²", 0, 1e-2, 8),
]


# Qt'nin kendi diyaloğu: Windows'un yerel diyaloğunda tuş olayları Qt'ye ulaşmaz ve Esc acil stop'u çalışmaz
NO_NATIVE = QtWidgets.QFileDialog.DontUseNativeDialog


def _fmt(a) -> str:
    return np.array2string(np.asarray(a), precision=4, suppress_small=True, max_line_width=110)


class DesignTab(QtWidgets.QWidget):
    def __init__(self, state, parent=None):
        super().__init__(parent)
        self.state = state
        root = QtWidgets.QHBoxLayout(self)

        # ------------------------------------------------ sol: girişler (kaydırılabilir)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        left_w = QtWidgets.QWidget()
        left = QtWidgets.QVBoxLayout(left_w)
        scroll.setWidget(left_w)
        scroll.setMinimumWidth(400)
        root.addWidget(scroll, 0)

        p = state.p
        g_par = QtWidgets.QGroupBox("Fiziksel parametreler (mekatronik ekibinden)")
        f = QtWidgets.QFormLayout(g_par)
        self.w_par = {}
        for key, label, unit, lo, hi, dec in PARAM_FIELDS:
            self.w_par[key] = spin(getattr(p, key), lo, hi, dec, suffix=unit)
            f.addRow(label, self.w_par[key])
        self.w_mode = QtWidgets.QComboBox()
        self.w_mode.addItems(INPUT_MODES)
        self.w_mode.setCurrentText(p.input_mode)
        self.w_mode.setToolTip("voltage: DC motor (BTS7960)  ·  force: ideal kuvvet  ·  acceleration: step motor")
        f.addRow("Giriş türü", self.w_mode)
        left.addWidget(g_par)

        g_mot = QtWidgets.QGroupBox("DC motor (giriş türü = voltage)")
        f = QtWidgets.QFormLayout(g_mot)
        self.w_mot = {}
        for key, label, unit, lo, hi, dec in MOTOR_FIELDS:
            self.w_mot[key] = spin(getattr(p.motor, key), lo, hi, dec, suffix=unit)
            f.addRow(label, self.w_mot[key])
        left.addWidget(g_mot)

        g_geo = QtWidgets.QGroupBox("Geometri hesaplayıcı (küre mi küp mü?)")
        f = QtWidgets.QFormLayout(g_geo)
        self.g_rod_m = spin(0.05, 0, 5, 4, suffix="kg")
        self.g_rod_l = spin(0.40, 0.01, 3, 3, suffix="m")
        self.g_bob_m = spin(0.15, 0, 5, 4, suffix="kg")
        self.g_shape = QtWidgets.QComboBox()
        self.g_shape.addItems(["sphere", "cube", "point"])
        self.g_size = spin(0.025, 0, 0.5, 4, suffix="m")
        self.g_size.setToolTip("küre: yarıçap, küp: kenar uzunluğu")
        self.g_btn = QtWidgets.QPushButton("Hesapla → m, l, I alanlarına aktar")
        f.addRow("Çubuk kütlesi", self.g_rod_m)
        f.addRow("Çubuk boyu", self.g_rod_l)
        f.addRow("Uç kütle", self.g_bob_m)
        f.addRow("Uç kütle şekli", self.g_shape)
        f.addRow("Yarıçap / kenar", self.g_size)
        f.addRow(self.g_btn)
        left.addWidget(g_geo)

        g_lqr = QtWidgets.QGroupBox("LQR ağırlıkları  ·  J = ∫ (sᵀQs + Ru²) dt")
        f = QtWidgets.QFormLayout(g_lqr)
        names = ["Q[x]  konum", "Q[ẋ]  hız", "Q[θ]  açı", "Q[θ̇]  açısal hız"]
        self.w_Q = [spin(q, 0, 1e6, 3) for q in state.Q]
        for n, w in zip(names, self.w_Q):
            f.addRow(n, w)
        self.w_R = spin(state.R, 1e-6, 1e6, 4)
        f.addRow("R  (giriş cezası)", self.w_R)
        self.w_Ts = spin(state.Ts * 1000, 1, 20, 0, 1, "ms")   # firmware 1 ms FreeRTOS tikiyle çalışır
        f.addRow("ESP32 kontrol periyodu Ts", self.w_Ts)
        left.addWidget(g_lqr)

        g_su = QtWidgets.QGroupBox("Swing-up ve geçiş")
        f = QtWidgets.QFormLayout(g_su)
        su = state.swingup
        self.w_su = {
            "k_E": spin(su.k_E, 0, 1e4, 2), "a_max": spin(su.a_max, 0, 50, 2, suffix="m/s²"),
            "k_x": spin(su.k_x, 0, 1e3, 2), "k_v": spin(su.k_v, 0, 1e3, 2),
        }
        for k, w in self.w_su.items():
            f.addRow(k, w)
        self.w_catch = spin(state.catch_deg, 1, 45, 1, suffix="°")
        self.w_release = spin(state.release_deg, 2, 90, 1, suffix="°")
        f.addRow("LQR devreye girme |θ| <", self.w_catch)
        f.addRow("LQR bırakma |θ| >", self.w_release)
        left.addWidget(g_su)

        bl = QtWidgets.QGridLayout()
        self.b_calc = QtWidgets.QPushButton("K Hesapla")
        self.b_calc.setStyleSheet("QPushButton{font-weight:bold;padding:6px}")
        self.b_send = QtWidgets.QPushButton("Cihaza Gönder")
        self.b_export = QtWidgets.QPushButton("gains.h Dışa Aktar")
        self.b_load = QtWidgets.QPushButton("Parametre Yükle…")
        self.b_save = QtWidgets.QPushButton("Parametre Kaydet…")
        bl.addWidget(self.b_calc, 0, 0, 1, 2)
        bl.addWidget(self.b_send, 1, 0)
        bl.addWidget(self.b_export, 1, 1)
        bl.addWidget(self.b_load, 2, 0)
        bl.addWidget(self.b_save, 2, 1)
        left.addLayout(bl)
        left.addStretch(1)

        # ------------------------------------------------ sağ: sonuçlar
        right = QtWidgets.QVBoxLayout()
        root.addLayout(right, 1)
        self.report = QtWidgets.QPlainTextEdit()
        self.report.setReadOnly(True)
        self.report.setFont(mono_font())
        right.addWidget(self.report, 3)
        self.pz = pg.PlotWidget(title="Kutup haritası (s-düzlemi):  × açık çevrim   ● kapalı çevrim (LQR)")
        self.pz.showGrid(x=True, y=True, alpha=0.3)
        self.pz.setLabel("bottom", "Re")
        self.pz.setLabel("left", "Im")
        self.pz.getAxis("left").enableAutoSIPrefix(False)
        self.pz.addItem(pg.InfiniteLine(angle=90, pos=0, pen=pg.mkPen("#7f8c8d")))
        self.pz.addItem(pg.InfiniteLine(angle=0, pos=0, pen=pg.mkPen("#7f8c8d")))
        self.ol = pg.ScatterPlotItem(symbol="x", size=14, pen=pg.mkPen("#c0392b", width=2))
        self.cl = pg.ScatterPlotItem(symbol="o", size=10, brush=pg.mkBrush("#2980b9"))
        self.pz.addItem(self.ol)
        self.pz.addItem(self.cl)
        right.addWidget(self.pz, 2)

        self.b_calc.clicked.connect(self.calculate)
        self.b_send.clicked.connect(self.send)
        self.b_export.clicked.connect(self.export)
        self.b_load.clicked.connect(self.load)
        self.b_save.clicked.connect(self.save)
        self.g_btn.clicked.connect(self.apply_geometry)
        state.connection_changed.connect(self.b_send.setEnabled)
        self.b_send.setEnabled(False)
        self.calculate()

    # ------------------------------------------------------------ girişler → durum
    def _params_from_widgets(self) -> PendulumParams:
        motor = MotorParams(**{k: w.value() for k, w in self.w_mot.items()})
        return PendulumParams(**{k: w.value() for k, w in self.w_par.items()},
                              input_mode=self.w_mode.currentText(), motor=motor, note=self.state.p.note)

    def _params_to_widgets(self, p: PendulumParams):
        for k, w in self.w_par.items():
            w.setValue(getattr(p, k))
        for k, w in self.w_mot.items():
            w.setValue(getattr(p.motor, k))
        self.w_mode.setCurrentText(p.input_mode)

    def apply_geometry(self):
        m, l, I = pendulum_from_geometry(self.g_rod_m.value(), self.g_rod_l.value(), self.g_bob_m.value(),
                                         self.g_shape.currentText(), self.g_size.value())
        self.w_par["m"].setValue(m)
        self.w_par["l"].setValue(l)
        self.w_par["I"].setValue(I)
        self.state.log.emit(f"Geometri: m={m:.4f} kg, l={l:.4f} m, I={I:.6f} kg·m² ({self.g_shape.currentText()})")
        self.calculate()

    def calculate(self):
        st = self.state
        try:
            p = self._params_from_widgets()
            if self.w_release.value() <= self.w_catch.value():
                raise ValueError("Bırakma açısı, devreye girme açısından büyük olmalı (histerezis).")
            Q = [w.value() for w in self.w_Q]
            R, Ts = self.w_R.value(), self.w_Ts.value() / 1000
            Kc, _, evc = design_lqr(p, Q, [[R]])
            Kd, _, evd = design_lqr(p, Q, [[R]], Ts=Ts)
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "Tasarım hatası", str(e))
            return
        st.p, st.Q, st.R, st.Ts = p, Q, R, Ts
        st.swingup = EnergySwingUp(**{k: w.value() for k, w in self.w_su.items()})
        st.catch_deg, st.release_deg = self.w_catch.value(), self.w_release.value()
        st.K, st.cl_poles = Kd.ravel(), evd
        st.params_changed.emit()
        st.gains_changed.emit()

        ss = StateSpace.from_params(p)
        ol = ss.poles()
        lines = [
            f"Giriş türü: {p.input_mode}    J = I + m l² = {p.J:.6f} kg·m²    ω_n = {p.omega_n:.2f} rad/s",
        ]
        if p.input_mode == "voltage":
            lines.append(f"Motor: α = {p.motor.alpha:.3f} N/V, β = {p.motor.beta:.3f} N·s/m, "
                         f"en yüksek araba hızı ≈ {max_cart_speed(p):.2f} m/s, kuvvet ≈ {max_cart_force(p):.1f} N")
        lines += [
            "", "A =", _fmt(ss.A), "", "B =", _fmt(ss.B.ravel()), "",
            f"Açık çevrim kutupları: {_fmt(ol)}",
            f"  → {'KARARSIZ (beklenen: sarkaç dik konumda düşer)' if np.any(ol.real > 1e-9) else 'kararlı'}",
            f"Kontrol edilebilirlik: rank {np.linalg.matrix_rank(ss.ctrb())}/4    "
            f"Gözlenebilirlik (x, θ ölçülerek): rank {np.linalg.matrix_rank(ss.obsv())}/4",
            "", f"Sürekli LQR  K = {_fmt(Kc.ravel())}",
            f"Kapalı çevrim kutupları: {_fmt(evc)}",
            "", f"Ayrık LQR (Ts = {Ts * 1000:.2f} ms)  K = {_fmt(Kd.ravel())}   ← ESP32'ye gönderilecek",
            f"|z| = {_fmt(np.abs(evd))}  → {'kararlı' if np.all(np.abs(evd) < 1) else 'KARARSIZ'}",
            "", "Kontrol kanunu: u = −(K0·(x−x_ref) + K1·ẋ + K2·θ + K3·θ̇)",
        ]
        if p.input_mode == "voltage" and max_cart_speed(p) < 0.8:
            lines.append("UYARI: araba en yüksek hızı düşük (<0.8 m/s); swing-up için redüktör oranı / kasnak gözden geçirilmeli.")
        self.report.setPlainText("\n".join(lines))
        self.ol.setData(ol.real, ol.imag)
        self.cl.setData(evc.real, evc.imag)

    # ------------------------------------------------------------ dosya / cihaz
    def send(self):
        self.calculate()
        self.state.push_to_device()

    def export(self):
        self.calculate()
        st = self.state
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "gains.h kaydet", str(st.base_dir / "firmware" / "esp32_pendulum" / "gains.h"), "C header (*.h)", options=NO_NATIVE)
        if path:
            write_gains_header(path, st.p, st.K, st.Ts, Q=st.Q, R=st.R, swingup=st.swingup,
                               catch_deg=st.catch_deg, release_deg=st.release_deg)
            st.log.emit(f"gains.h yazıldı: {path}")

    def load(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Parametre dosyası", str(self.state.params_path.parent),
                                                        "JSON (*.json)", options=NO_NATIVE)
        if path:
            try:
                self._params_to_widgets(load_params(path))
            except Exception as e:
                QtWidgets.QMessageBox.warning(self, "Yüklenemedi", str(e))
                return
            self.calculate()

    def save(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Parametreleri kaydet", str(self.state.params_path),
                                                        "JSON (*.json)", options=NO_NATIVE)
        if path:
            self.calculate()
            save_params(self.state.p, path)
            self.state.log.emit(f"Parametreler kaydedildi: {path}")
