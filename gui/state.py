"""Sekmeler arasında paylaşılan uygulama durumu: parametreler, LQR ağırlıkları, kazançlar, bağlantı."""

from __future__ import annotations

from pathlib import Path

import numpy as np
from PyQt5 import QtCore

from pendulum_lab.control.lqr import design_lqr
from pendulum_lab.control.swingup import EnergySwingUp
from pendulum_lab.model.params import PendulumParams, load_params


class AppState(QtCore.QObject):
    params_changed = QtCore.pyqtSignal()
    gains_changed = QtCore.pyqtSignal()
    connection_changed = QtCore.pyqtSignal(bool)
    log = QtCore.pyqtSignal(str)

    def __init__(self, params_path: str | Path):
        super().__init__()
        self.base_dir = Path(params_path).resolve().parent.parent
        self.params_path = Path(params_path)
        self.startup_warning = ""
        try:
            self.p = load_params(self.params_path) if self.params_path.exists() else PendulumParams()
        except (ValueError, TypeError, OSError) as e:   # bozuk/geçersiz dosya: varsayılanlarla aç, uyar
            self.p = PendulumParams()
            self.startup_warning = f"{self.params_path} okunamadı ({e}); varsayılan parametreler kullanılıyor."
        self.Q = [10.0, 1.0, 100.0, 1.0]
        self.R = 0.1
        self.Ts = 0.002
        self.swingup = EnergySwingUp()
        self.catch_deg = 15.0
        self.release_deg = 30.0
        self.swingup_enabled = True
        self.x_ref = 0.0
        self.device_Ts: float | None = None     # cihazın bildirdiği kontrol periyodu
        self.K = np.zeros(4)
        self.cl_poles = np.array([])
        self.reader = None
        self.recorder = None
        self.compute_gains()

    def compute_gains(self):
        K, _, ev = design_lqr(self.p, self.Q, [[self.R]], Ts=self.Ts)
        self.K, self.cl_poles = K.ravel(), ev
        self.gains_changed.emit()

    # ------------------------------------------------------------ bağlantı
    @property
    def connected(self) -> bool:
        return self.reader is not None

    def set_reader(self, reader):
        self.reader = reader
        self.device_Ts = None
        self.connection_changed.emit(reader is not None)

    def on_device_info(self, data: dict):
        """ping yanıtı veya boot olayı geldi: Ts'yi öğren ve ayarları (yeniden) yükle.
        Cihaz yeniden başladıysa PC'den gelen ayarları kaybetmiştir; bu yüzden boot'ta da çağrılır."""
        if "Ts" in data:
            self.device_Ts = float(data["Ts"])
        ts = f"  Ts={self.device_Ts * 1000:g} ms" if self.device_Ts else "  (Ts bildirilmedi)"
        self.log.emit(f"Cihaz: fw={data.get('fw', '?')}{ts}")
        self.push_to_device()

    def send(self, cmd: str, **fields) -> bool:
        if self.reader is None:
            self.log.emit("Bağlı değil — önce cihaza bağlanın.")
            return False
        try:
            self.reader.send(cmd, **fields)
            return True
        except Exception as e:  # geçersiz komut veya port hatası
            self.log.emit(f"Gönderilemedi ({cmd}): {e}")
            return False

    def push_to_device(self) -> bool:
        """Güncel K ve tüm swing-up/geçiş ayarlarını (swing-up açık/kapalı ve x_ref dahil) cihaza yükler.
        K, cihazın kontrol periyodu için tasarlanmamışsa GÖNDERİLMEZ."""
        if self.device_Ts is not None and abs(self.device_Ts - self.Ts) > 1e-9:
            self.log.emit(f"UYARI: cihaz Ts = {self.device_Ts * 1000:g} ms, tasarım Ts = {self.Ts * 1000:g} ms. "
                          "K gönderilmedi — Tasarım sekmesinde Ts'yi cihazla aynı yapıp tekrar hesaplayın.")
            return False
        su = self.swingup
        self.x_ref = self.clamp_xref(self.x_ref)
        # Cihaz set_param'ı "hepsi ya da hiçbiri" uygular; güvenlik açısından önemli swing-up anahtarı
        # başka bir alanın reddiyle kaybolmasın diye AYRI mesajlarla gönderilir.
        ok = self.send("set_gain", K=self.K.tolist())
        ok = ok and self.send("set_param", swingup=int(self.swingup_enabled))
        ok = ok and self.send("set_param", catch_deg=self.catch_deg, release_deg=self.release_deg)
        ok = ok and self.send("set_param", k_E=su.k_E, a_max=su.a_max, k_x=su.k_x, k_v=su.k_v)
        ok = ok and self.send("set_param", u_max=self.p.u_max)
        ok = ok and self.send("set_param", x_ref=self.x_ref)
        if ok:
            self.log.emit("K = [" + ", ".join(f"{k:.3f}" for k in self.K) + "] ve ayarlar gönderildi "
                          "(reddedilen olursa aşağıda ✗ ile görünür).")
        return ok

    def clamp_xref(self, x: float) -> float:
        """Cihazın kuralı: |x_ref| ≤ rail_half − 0.1 m."""
        lim = max(0.0, self.p.rail_half - 0.1)
        return max(-lim, min(lim, x))
