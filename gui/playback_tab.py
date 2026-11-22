"""Kayıt oynatma ve simülasyonla model doğrulama sekmesi (Issue #15)."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pyqtgraph as pg
from PyQt5 import QtCore, QtWidgets

from pendulum_lab.control.supervisor import MODE_CODES, Mode
from pendulum_lab.model.dynamics import energy
from pendulum_lab.recorder import (
    RecordedSession,
    ReplayComparison,
    compare_recording_with_simulation,
    load_recording,
)

from .widgets import PendulumView, mono_font, setup_mode_axis

NO_NATIVE = QtWidgets.QFileDialog.DontUseNativeDialog


class PlaybackTab(QtWidgets.QWidget):
    """Kaydedilmiş CSV telemetrilerini açar, animasyonla oynatır ve simülasyonla karşılaştırır."""

    def __init__(self, state, parent=None):
        super().__init__(parent)
        self.state = state
        self.session: RecordedSession | None = None
        self.comparison: ReplayComparison | None = None

        root = QtWidgets.QHBoxLayout(self)

        # ---- sol: dosya seçimi, ayarlar ve özet metrikler
        left_w = QtWidgets.QWidget()
        left_w.setMaximumWidth(360)
        left_l = QtWidgets.QVBoxLayout(left_w)

        file_row = QtWidgets.QHBoxLayout()
        self.file_cb = QtWidgets.QComboBox()
        self.file_cb.setSizeAdjustPolicy(QtWidgets.QComboBox.AdjustToMinimumContentsLengthWithIcon)
        self.b_open = QtWidgets.QPushButton("📂 CSV Seç…")
        self.b_refresh = QtWidgets.QPushButton("↻")
        self.b_refresh.setFixedWidth(30)
        self.b_refresh.setToolTip("kayitlar/ klasörünü yenile")
        file_row.addWidget(self.file_cb, 1)
        file_row.addWidget(self.b_refresh)
        file_row.addWidget(self.b_open)
        left_l.addLayout(file_row)

        form = QtWidgets.QFormLayout()
        self.active_cb = QtWidgets.QCheckBox("Yalnız aktif denemeyi hizala (BAŞLAT → bitiş)")
        self.active_cb.setChecked(True)
        self.rec_cfg_cb = QtWidgets.QCheckBox("Kayıttaki model ve K ayarlarını kullan")
        self.rec_cfg_cb.setChecked(True)
        self.real_cb = QtWidgets.QCheckBox("Gerçekçi sensör (AS5600 12-bit, enkoder)")
        self.real_cb.setChecked(True)
        self.kalman_cb = QtWidgets.QCheckBox("Kalman filtresi ile hız kestirimi")
        self.kalman_cb.setChecked(True)
        self.anim_src = QtWidgets.QComboBox()
        self.anim_src.addItems(["Kayıt", "Simülasyon"])
        self.b_run = QtWidgets.QPushButton("▶ Karşılaştır / Yenile")
        self.b_run.setStyleSheet("QPushButton{font-weight:bold;padding:6px}")

        form.addRow(self.active_cb)
        form.addRow(self.rec_cfg_cb)
        form.addRow(self.real_cb)
        form.addRow(self.kalman_cb)
        form.addRow("Animasyon kaynağı:", self.anim_src)
        form.addRow(self.b_run)
        left_l.addLayout(form)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        self.info = QtWidgets.QLabel("kayitlar/ içinden bir CSV seçin veya 'CSV Seç…' ile yükleyin.")
        self.info.setWordWrap(True)
        self.info.setAlignment(QtCore.Qt.AlignTop | QtCore.Qt.AlignLeft)
        self.info.setFont(mono_font())
        self.info.setMargin(4)
        scroll.setWidget(self.info)
        left_l.addWidget(scroll, 1)
        root.addWidget(left_w)

        # ---- orta: üst üste karşılaştırma grafikleri
        mid = QtWidgets.QVBoxLayout()
        root.addLayout(mid, 1)
        self.glw = pg.GraphicsLayoutWidget()
        self.p_th = self.glw.addPlot(row=0, col=0, title="θ [°]  (düz: Kayıt, kesikli: Simülasyon)")
        self.p_x = self.glw.addPlot(row=1, col=0, title="x [m]")
        self.p_u = self.glw.addPlot(row=2, col=0, title="u")
        self.p_e = self.glw.addPlot(row=3, col=0, title="Enerji E [J]  (0 = dik)")
        self.p_m = self.glw.addPlot(row=4, col=0, title="mod")
        plots = (self.p_th, self.p_x, self.p_u, self.p_e, self.p_m)
        for pl in plots:
            pl.showGrid(x=True, y=True, alpha=0.25)
        for pl in plots[1:]:
            pl.setXLink(self.p_th)
        self.p_m.setLabel("bottom", "t [s]")

        self.p_th.addLegend(offset=(10, 5))
        dash = QtCore.Qt.DashLine
        self.c_rec_th = self.p_th.plot(pen=pg.mkPen("#2980b9", width=2), name="Kayıt")
        self.c_sim_th = self.p_th.plot(pen=pg.mkPen("#1b4f72", width=1.8, style=dash), name="Simülasyon")
        self.c_rec_x = self.p_x.plot(pen=pg.mkPen("#16a085", width=2))
        self.c_sim_x = self.p_x.plot(pen=pg.mkPen("#0e6251", width=1.8, style=dash))
        self.c_rec_u = self.p_u.plot(pen=pg.mkPen("#c0392b", width=1.5))
        self.c_sim_u = self.p_u.plot(pen=pg.mkPen("#78281f", width=1.5, style=dash))
        self.c_rec_e = self.p_e.plot(pen=pg.mkPen("#e67e22", width=1.8))
        self.c_sim_e = self.p_e.plot(pen=pg.mkPen("#935116", width=1.6, style=dash))
        self.c_rec_m = self.p_m.plot(pen=pg.mkPen("#8e44ad", width=2), stepMode="left")
        self.c_sim_m = self.p_m.plot(pen=pg.mkPen("#4a235a", width=1.6, style=dash), stepMode="left")

        self.p_e.addItem(pg.InfiniteLine(angle=0, pen=pg.mkPen("#27ae60", style=dash)))
        setup_mode_axis(self.p_m)

        self.cursor = [pg.InfiniteLine(angle=90, pen=pg.mkPen("#f39c12")) for _ in range(5)]
        for pl, c in zip(plots, self.cursor):
            pl.addItem(c)
        mid.addWidget(self.glw)

        # ---- sağ: sarkaç animasyonu + oynatma kontrolleri
        right = QtWidgets.QVBoxLayout()
        root.addLayout(right, 1)
        self.view = PendulumView(state.p.rail_half, state.p.l)
        right.addWidget(self.view, 1)
        pl_ctrl = QtWidgets.QHBoxLayout()
        self.play_btn = QtWidgets.QPushButton("▶ Oynat")
        self.speed = QtWidgets.QComboBox()
        self.speed.addItems(["0.25x", "0.5x", "1x", "2x"])
        self.speed.setCurrentText("1x")
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        pl_ctrl.addWidget(self.play_btn)
        pl_ctrl.addWidget(self.speed)
        pl_ctrl.addWidget(self.slider, 1)
        right.addLayout(pl_ctrl)

        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(16)
        self.timer.timeout.connect(self._advance)
        self._clock = QtCore.QElapsedTimer()
        self._play_t0 = 0.0                       # oynatmanın başladığı kayıt zamanı [s]
        self.speed.currentIndexChanged.connect(self._rebase_clock)
        self.slider.sliderReleased.connect(self._rebase_clock)   # oynatırken elle sarma
        # çubuğa tıklama / ok tuşları: değer actionTriggered'dan SONRA değişir → saat bir sonraki olayda kurulur
        self.slider.actionTriggered.connect(lambda *_: QtCore.QTimer.singleShot(0, self._rebase_clock))

        self.b_refresh.clicked.connect(self.refresh_file_list)
        self.b_open.clicked.connect(self.open_dialog)
        self.b_run.clicked.connect(self._on_run_clicked)
        self.play_btn.clicked.connect(self.toggle_play)
        self.slider.valueChanged.connect(self._show_frame)
        self.anim_src.currentIndexChanged.connect(lambda: self._show_frame(self.slider.value()))

        self.refresh_file_list()

    def refresh_file_list(self):
        """kayitlar/ klasöründeki CSV dosyalarını listeler."""
        cur = self.file_cb.currentData()
        rec_dir = self.state.base_dir / "kayitlar"
        files = sorted(rec_dir.glob("*.csv"), reverse=True) if rec_dir.exists() else []
        self.file_cb.blockSignals(True)
        self.file_cb.clear()
        for f in files:
            self.file_cb.addItem(f.name, str(f))
        if cur:
            idx = self.file_cb.findData(cur)
            if idx >= 0:
                self.file_cb.setCurrentIndex(idx)
        self.file_cb.blockSignals(False)

    def open_dialog(self):
        rec_dir = self.state.base_dir / "kayitlar"
        start_dir = str(rec_dir if rec_dir.exists() else self.state.base_dir)
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Kayıt CSV seç", start_dir, "CSV (*.csv)", options=NO_NATIVE
        )
        if path:
            self.load_file(path)

    def _on_run_clicked(self):
        data = self.file_cb.currentData()
        if self.session is None or (data and Path(data) != self.session.path):
            if data:
                self.load_file(data)
            else:
                self.open_dialog()
            return
        self.run_comparison()

    def load_file(self, path: str | Path):
        """Seçilen CSV kaydını yükler ve simülasyonla karşılaştırmayı çalıştırır."""
        p = Path(path)
        try:
            self.session = load_recording(p)
        except Exception as exc:
            QtWidgets.QMessageBox.warning(self, "Kayıt yüklenemedi", str(exc))
            return

        if self.file_cb.findData(str(p)) < 0:
            self.file_cb.addItem(p.name, str(p))
        self.file_cb.setCurrentIndex(self.file_cb.findData(str(p)))
        self.run_comparison()

    def run_comparison(self):
        """Yüklü kayıt için aynı başlangıç koşullu simülasyonu koşup grafikleri günceller."""
        if self.session is None:
            return
        if self.timer.isActive():
            self.toggle_play()

        QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
        try:
            comp = compare_recording_with_simulation(
                self.session,
                active_only=self.active_cb.isChecked(),
                use_recorded_config=self.rec_cfg_cb.isChecked(),
                fallback_state=self.state,
                realistic_sensor=self.real_cb.isChecked(),
                use_kalman=self.kalman_cb.isChecked(),
            )
        except Exception as exc:
            QtWidgets.QMessageBox.warning(self, "Karşılaştırma hatası", str(exc))
            return
        finally:
            QtWidgets.QApplication.restoreOverrideCursor()

        self.comparison = comp
        p, sim = comp.params, comp.sim
        self.view.set_geometry(p.rail_half, p.l)

        rec_th_deg = np.degrees(np.arctan2(np.sin(comp.th), np.cos(comp.th)))
        sim_th_deg = np.degrees(np.arctan2(np.sin(sim.th), np.cos(sim.th)))
        self.c_rec_th.setData(comp.t, rec_th_deg)
        self.c_sim_th.setData(sim.t, sim_th_deg)
        self.c_rec_x.setData(comp.t, comp.x)
        self.c_sim_x.setData(sim.t, sim.x)
        self.c_rec_u.setData(comp.t, comp.u)
        self.c_sim_u.setData(sim.t, sim.u)

        rec_e = np.array([energy(p, th, dth) for th, dth in zip(comp.th, comp.dth)])
        sim_e = np.array([energy(p, th, dth) for th, dth in zip(sim.th, sim.s[:, 3])])
        self.c_rec_e.setData(comp.t, rec_e)
        self.c_sim_e.setData(sim.t, sim_e)

        rec_codes = np.array([MODE_CODES.get(Mode(m), -1) if m in Mode._value2member_map_ else -1 for m in comp.mode], dtype=float)
        sim_codes = np.array([MODE_CODES.get(Mode(m), -1) if m else -1 for m in sim.mode], dtype=float)
        self.c_rec_m.setData(comp.t, rec_codes)
        self.c_sim_m.setData(sim.t, sim_codes)

        src_lbl = self.session.meta.get("kaynak", "belirtilmemiş")
        lines = [
            f"Dosya: {self.session.path.name}  ({len(comp.t)} örnek, {comp.t[-1]:.2f} s)",
            f"Kaynak: {src_lbl}",
        ]
        if comp.t_start > 0:
            lines.append(f"Simülasyon BAŞLAT anında başlar: t = {comp.t_start:.2f} s")
        if comp.changed_during_trial:
            lines += [
                f"UYARI: deneme sırasında değişen ayar(lar): {', '.join(comp.changed_during_trial)}",
                "  simülasyon BAŞLAT anındaki ayarlarla koşuldu; sonrası farklılık gösterebilir.",
            ]
        if comp.sim_actions_during_trial:
            lines += [
                f"UYARI: kayıtta {', '.join(comp.sim_actions_during_trial)} var; karşılaştırma simülasyonu "
                "bunları içermez, RMSE yanıltıcı olabilir.",
            ]
        lines += [
            "Metrikler yalnız aktif deneme aralığında hesaplanır.",
            f"RMSE(θ) = {comp.rmse_th_deg:.2f}°   max|Δθ| = {comp.max_err_th_deg:.2f}°",
            f"RMSE(x) = {comp.rmse_x:.4f} m   max|Δx| = {comp.max_err_x:.4f} m",
            f"RMSE(u) = {comp.rmse_u:.3f}",
            "",
            "--- KAYIT ÖZETİ ---",
            *comp.rec_summary.format_lines(rail_half=p.rail_half, u_max=p.u_max),
            "",
            "--- SİMÜLASYON ÖZETİ ---",
            *comp.sim_summary.format_lines(rail_half=p.rail_half, u_max=p.u_max),
        ]
        self.info.setText("\n".join(lines))

        self.slider.setRange(0, len(comp.t) - 1)
        self.slider.setValue(0)
        self._show_frame(0)

    def toggle_play(self):
        if self.comparison is None:
            return
        if self.timer.isActive():
            self.timer.stop()
            self.play_btn.setText("▶ Oynat")
        else:
            if self.slider.value() >= self.slider.maximum():
                self.slider.setValue(0)
            self._rebase_clock()
            self.timer.start()
            self.play_btn.setText("❚❚ Duraklat")

    def _speed_factor(self) -> float:
        return float(self.speed.currentText().rstrip("x"))

    def _rebase_clock(self, *_):
        """Oynatma saatini şu anki kareye sabitler (başlat / hız değişimi)."""
        if self.comparison is not None:
            self._play_t0 = float(self.comparison.t[self.slider.value()])
        self._clock.restart()

    def frame_at(self, elapsed_s: float) -> int:
        """Başlangıçtan elapsed_s gerçek saniye sonra gösterilecek kare: CSV zaman damgalarına göre
        (örnek aralıkları düzensiz olsa da) t = t0 + hız × geçen süre olan son satır."""
        t = self.comparison.t
        target = self._play_t0 + elapsed_s * self._speed_factor()
        return int(min(max(np.searchsorted(t, target, side="right") - 1, 0), len(t) - 1))

    def _advance(self):
        comp = self.comparison
        if comp is None or len(comp.t) < 2:
            return
        v = self.frame_at(self._clock.elapsed() / 1000.0)
        if v >= self.slider.maximum():
            v = self.slider.maximum()
            self.toggle_play()
        if v != self.slider.value():
            self.slider.setValue(v)

    def _show_frame(self, i: int):
        comp = self.comparison
        if comp is None or i >= len(comp.t):
            return
        ti = float(comp.t[i])
        if self.anim_src.currentText() == "Simülasyon":
            x = float(np.interp(ti, comp.sim.t, comp.sim.x))
            th = float(np.interp(ti, comp.sim.t, comp.sim.th))
        else:
            x = float(comp.x[i])
            th = float(comp.th[i])
        self.view.set_state(x, th, 0.0)
        for c in self.cursor:
            c.setValue(ti)
