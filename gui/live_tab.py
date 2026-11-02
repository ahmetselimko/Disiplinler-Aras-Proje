"""Canlı izleme ve komut sekmesi."""

from __future__ import annotations

import math
import time

import numpy as np
import pyqtgraph as pg
import copy

from PyQt5 import QtCore, QtWidgets

from pendulum_lab.comm import protocol as P
from pendulum_lab.comm.reader import COLUMNS
from pendulum_lab.comm.transport import SimTransport
from pendulum_lab.recorder import Recorder

from .widgets import MODE_COLORS, MODE_TR, PendulumView, mono_font, push_with_confirm, spin

C = {name: i for i, name in enumerate(COLUMNS)}
STOPPED_COLOR = "#5d6d7e"   # DURDUR'dan sonraki IDLE ("Durduruldu")


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
        self.p_e = self.glw.addPlot(row=3, col=0, title="Sarkaç enerjisi E [J]  (0 = dik)")
        self.p_m = self.glw.addPlot(row=4, col=0, title="Kontrolcü modu  (3 = swing-up, 4 = LQR)")
        for pl in (self.p_th, self.p_x, self.p_u, self.p_e, self.p_m):
            pl.showGrid(x=True, y=True, alpha=0.25)
            pl.setClipToView(True)
            pl.setDownsampling(auto=True, mode="peak")
            pl.enableAutoRange(axis="y")
        for pl in (self.p_x, self.p_u, self.p_e, self.p_m):
            pl.setXLink(self.p_th)
        self.p_m.setLabel("bottom", "zaman [s] (0 = şimdi)")
        self.c_th = self.p_th.plot(pen=pg.mkPen("#2980b9", width=2))
        self.c_x = self.p_x.plot(pen=pg.mkPen("#16a085", width=2))
        self.c_u = self.p_u.plot(pen=pg.mkPen("#c0392b", width=1.5))
        self.c_e = self.p_e.plot(pen=pg.mkPen("#e67e22", width=1.8))
        self.c_m = self.p_m.plot(pen=pg.mkPen("#8e44ad", width=2), stepMode="left")
        # "peak" seyreltme kesikli mod değerleri arasında olmayan ara değerler üretebilir
        self.c_m.setDownsampling(ds=1, auto=False)

        self.p_e.addItem(pg.InfiniteLine(angle=0, pen=pg.mkPen("#27ae60", style=QtCore.Qt.DashLine)))
        ax_m = self.p_m.getAxis("left")
        ax_m.setTicks([[(0, "IDLE"), (1, "HOME"), (2, "WAIT"), (3, "SWING"), (4, "LQR"), (5, "FAULT")]])
        self.p_m.setYRange(-0.5, 5.5, padding=0)

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

        self.summary_box = QtWidgets.QGroupBox("Sonuç Özeti")
        summary_l = QtWidgets.QVBoxLayout(self.summary_box)
        summary_l.setContentsMargins(6, 4, 6, 4)
        self.summary_lbl = QtWidgets.QLabel("Deneme bekleniyor (BAŞLAT ile başlar)")
        self.summary_lbl.setFont(mono_font())
        self.summary_lbl.setWordWrap(True)
        summary_l.addWidget(self.summary_lbl)
        right.addWidget(self.summary_box)

        self._trial_reader = None
        self._last_summary_render = 0.0

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
        self.b_friction = QtWidgets.QPushButton("Sürtünme Ölç")
        self.b_friction.setToolTip("KALİBRE ET'ten sonra, sarkaç aşağıda dururken: araba iki yönde ~8 cm'er "
                                   "gider, motor + kayış sürtünmesini gerilim olarak ölçer")
        self.b_pause = QtWidgets.QPushButton("Duraklat")
        self.b_pause.setCheckable(True)
        self.b_pause.setToolTip("Yalnız simülasyon: fizik ve kontrol donar, telemetri son değerlerle sürer")
        self.b_reset = QtWidgets.QPushButton("Sıfırla")
        self.b_reset.setToolTip("Yalnız simülasyon: BEKLEMEDE'ye döner, sarkaç θ₀ açısında durağan konur "
                                "(homing yapıldıysa araba rayın ortasına)")
        self.th0 = spin(180.0, -180.0, 180.0, 0, 5, "°")
        self.th0.setToolTip("Sıfırlamada başlangıç açısı θ₀ (0 = dik, 180 = aşağıda)")
        self.b_rec = QtWidgets.QPushButton("● Kaydı Başlat")
        self.b_rec.setCheckable(True)
        btns.addWidget(self.b_home, 0, 0)
        btns.addWidget(self.b_push, 0, 1)
        btns.addWidget(self.b_start, 1, 0)
        btns.addWidget(self.b_stop, 1, 1)
        btns.addWidget(self.b_estop, 2, 0, 1, 2)
        btns.addWidget(self.b_calib, 3, 0)
        btns.addWidget(self.b_rec, 3, 1)
        btns.addWidget(self.b_kick, 4, 0)
        btns.addWidget(self.b_friction, 4, 1)
        btns.addWidget(self.b_pause, 5, 0)
        reset_row = QtWidgets.QHBoxLayout()
        for w in (self.b_reset, QtWidgets.QLabel("θ₀:"), self.th0):
            reset_row.addWidget(w)
        btns.addLayout(reset_row, 5, 1)
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
        self.log.setMinimumHeight(120)           # özet kutusu eklenince kayıt alanı birkaç satıra inmesin
        right.addWidget(self.log, 1)

        self._stopped = False      # son mod komutu DURDUR muydu (IDLE "Durduruldu" gösterilir)
        self.b_home.clicked.connect(lambda: self.send_mode_cmd("home"))
        self.b_calib.clicked.connect(lambda: state.send("calib"))
        self.b_push.clicked.connect(lambda: push_with_confirm(self, state))
        self.b_start.clicked.connect(lambda: self.send_mode_cmd("start"))
        self.b_stop.clicked.connect(lambda: self.send_mode_cmd("stop"))
        self.b_estop.clicked.connect(self.estop)
        self.b_kick.clicked.connect(lambda: state.send("kick", dth=1.0))
        self.b_friction.clicked.connect(self.measure_friction)
        self.b_pause.toggled.connect(self.toggle_pause)
        self.b_reset.clicked.connect(self.reset_sim)
        self.b_xref.clicked.connect(self.send_xref)
        self.swing_cb.toggled.connect(self.set_swingup)
        self.b_rec.toggled.connect(self.toggle_record)
        state.log.connect(self.append_log)
        state.connection_changed.connect(self._on_conn)
        state.params_changed.connect(self._on_params)
        self._on_params()
        self._on_conn(False)

    # ------------------------------------------------------------ eylemler
    def send_mode_cmd(self, cmd: str):
        # start reddedilebilir: kabul edilirse mod zaten IDLE'dan çıkar, "Durduruldu" burada silinmez
        if self.state.send(cmd) and cmd != "start":
            self._stopped = cmd == "stop"

    def measure_friction(self):
        if self.state.real_device:
            ans = QtWidgets.QMessageBox.question(
                self, "Sürtünme ölçümü", "Araba önce sağa, sonra sola yaklaşık 8 cm'er gidecek. Ray boş olmalı, "
                "sarkaç aşağıda hareketsiz durmalı; elinizi acil stop'ta tutun.\n\nBaşlatılsın mı?")
            if ans != QtWidgets.QMessageBox.Yes:
                return
        self.state.send("friction")

    def on_friction(self, data: dict):
        """Ölçüm sonucu: göster; onaylanırsa T_f'yi parametrelere yaz ve telafiyi cihaza gönder."""
        st = self.state
        if not data.get("ok"):
            self.append_log(f"✗ sürtünme ölçümü: {data.get('reason', '?')}")
            return
        m, p = st.p.motor, st.p
        v_c = float(data["v_c"])
        F_c = max(0.0, v_c) * m.alpha
        T_f = F_c * m.r_pulley / m.gear
        k_model = (m.beta + p.b_cart) / m.alpha
        text = (f"Sürtünmeyi yenen gerilim: V_c = {v_c:.3f} V  (+x {data['v_pos']:.3f} V, −x {data['v_neg']:.3f} V)\n"
                f"Modeldeki motor sabitleriyle: F_c ≈ {F_c:.2f} N arabada, T_f ≈ {T_f:.5f} N·m\n"
                f"Eğim (β+b)/α: ölçülen {data['slope']:.2f}, model {k_model:.2f} V·s/m "
                "(fark büyükse motor sabitleri gözden geçirilmeli)")
        self.append_log("● sürtünme ölçümü: " + text.replace("\n", " | "))
        ans = QtWidgets.QMessageBox.question(
            self, "Sürtünme ölçümü", text + "\n\nT_f parametrelere yazılsın ve telafi cihaza gönderilsin mi?")
        if ans != QtWidgets.QMessageBox.Yes:
            return
        st.p = copy.deepcopy(st.p)
        st.p.motor.T_f = T_f
        st.params_replaced.emit()
        st.send("set_param", fc_volt=min(6.0, max(0.0, v_c)))
        st.log.emit(f"T_f = {T_f:.5f} N·m yazıldı ve telafi gönderildi. Kalıcı olması için Tasarım sekmesinde "
                    "Parametre Kaydet; firmware'e gömmek için gains.h Dışa Aktar.")

    def toggle_pause(self, on: bool):
        if not self.state.send("pause", on=int(on)):
            on = not on                          # gönderilemedi: buton eski hâline dönsün
        self._set_paused(on)

    def reset_sim(self):
        if self.state.send("reset", th0_deg=self.th0.value()):
            self._stopped = False
            self._set_paused(False)              # cihaz sıfırlamada duraklatmayı da kaldırır

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
            real = st.real_device              # kaynak dosya adında ve başlıkta: kayıtlar karışmasın
            path = Recorder.default_path(st.base_dir / "kayitlar", prefix="gercek" if real else "sim")
            source = f"gerçek kart ({st.reader.transport.name})" if real else "simülasyon (sanal ESP32)"
            st.recorder = Recorder(path, meta={"kaynak": source, **st.get_metadata()})
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
            if msg.name == "friction":
                self.on_friction(msg.data)
                return
            if msg.name == "recorder_error":
                self.append_log(f"!!! Kayıt durduruldu: {msg.data.get('msg')}")
                self.b_rec.setChecked(False)
                return
            extra = " ".join(f"{k}={v}" for k, v in msg.data.items())
            self.append_log(f"● olay: {msg.name} {extra}".rstrip())

    # ------------------------------------------------------------ güncelleme (60 FPS)
    def refresh(self, reader):
        self.refresh_summary(reader)
        if self.pause_cb.isChecked() or reader is None:
            return
        last = reader.last
        if last is not None:
            self._show_mode(last.mode)
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
        th = data[keep, C["th"]]
        dth = data[keep, C["dth"]]
        self.c_th.setData(t, np.degrees(th))
        self.c_x.setData(t, data[keep, C["x"]])
        self.c_u.setData(t, data[keep, C["u"]])

        p = self.state.p
        E = 0.5 * p.J * (dth ** 2) + p.m * p.g * p.l * (np.cos(th) - 1.0)
        self.c_e.setData(t, E)

        self.c_m.setData(t, data[keep, C["mode"]])
        self.p_th.setXRange(-self.window.value(), 0, padding=0)

    # ------------------------------------------------------------ yardımcılar
    def _show_mode(self, mode: str):
        if mode == "IDLE" and self._stopped:     # yalnız arayüzde; cihazda ayrı bir mod yok
            text, color = "DURDURULDU", STOPPED_COLOR
        else:
            text, color = MODE_TR.get(mode, mode), MODE_COLORS.get(mode, "#bdc3c7")
        if self.b_pause.isChecked():
            text = "⏸ " + text
        self.mode_lbl.setText(text)
        self._set_mode_style(color)

    def _set_paused(self, on: bool):
        self.b_pause.blockSignals(True)          # komut göndermeden yalnız butonu güncelle
        self.b_pause.setChecked(on)
        self.b_pause.blockSignals(False)
        self.b_pause.setText("Devam Et" if on else "Duraklat")

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
                  self.b_rec, self.b_friction):
            b.setEnabled(connected)
        reader = self.state.reader
        sim = connected and isinstance(getattr(reader, "transport", None), SimTransport)
        for w in (self.b_kick, self.b_pause, self.b_reset, self.th0):
            w.setEnabled(sim)
        self._stopped = False
        self._set_paused(False)                  # yeni bağlantıda sanal cihaz duraklatılmamış başlar
        if not connected:
            if self.b_rec.isChecked():
                self.b_rec.setChecked(False)
            self.mode_lbl.setText("BAĞLI DEĞİL")
            self._set_mode_style("#bdc3c7")
            self.refresh_summary(self._trial_reader, force=True)
        else:
            self._trial_reader = reader
            self.summary_lbl.setText("Deneme bekleniyor (BAŞLAT ile başlar)")

    def refresh_summary(self, reader, force=False):
        """Thread-güvenli O(1) anlık görüntüyü yazar; veri toplama çizime hiç bağlı değildir."""
        if reader is None or not hasattr(reader, "trials"):
            return
        now = time.monotonic()
        if not force and now - self._last_summary_render < 0.25:
            return
        self._last_summary_render = now
        summary, active = reader.trials.snapshot()
        if summary is None:
            return
        p = self.state.p
        title = "Canlı Deneme Sürüyor:" if active else "Son Deneme Özeti:"
        lines = summary.format_lines(rail_half=p.rail_half, u_max=reader.trials.u_max)
        self.summary_lbl.setText(title + "\n" + "\n".join(lines))
