"""Sekmeler arasında paylaşılan uygulama durumu: parametreler, LQR ağırlıkları, kazançlar, bağlantı."""

from __future__ import annotations

import time
from pathlib import Path

import numpy as np
from PyQt5 import QtCore

from pendulum_lab.comm import protocol as P
from pendulum_lab.comm.transport import SimTransport
from pendulum_lab.control.lqr import design_lqr
from pendulum_lab.control.supervisor import CATCH_DEG, CATCH_DTH, CATCH_DX, CATCH_X, RELEASE_DEG
from pendulum_lab.control.swingup import EnergySwingUp
from pendulum_lab.model.params import PendulumParams, load_params


class AppState(QtCore.QObject):
    params_changed = QtCore.pyqtSignal()
    gains_changed = QtCore.pyqtSignal()
    connection_changed = QtCore.pyqtSignal(bool)
    params_replaced = QtCore.pyqtSignal()      # parametreler sekme dışından değişti (ör. sürtünme ölçümü)
    log = QtCore.pyqtSignal(str)

    def __init__(self, params_path: str | Path):
        super().__init__()
        self.base_dir = Path(params_path).resolve().parent.parent
        self.params_path = Path(params_path)
        self.startup_warning = ""
        # params.json okunamazsa genel yer tutucu varsayılanlarla açılır (motor/kasnak projeninkinden farklı:
        # 10:1, 12.7 mm). Bu durumda K gönderilmez ve gains.h dışa aktarılmaz (firmware'deki 44 CPR'la x yanlış
        # ölçeklenirdi); bir parametre dosyası Yüklenince kalkar (clear_params_fallback), kaydetmek kaldırmaz.
        self.params_fallback = False
        try:
            if self.params_path.exists():
                self.p = load_params(self.params_path)
            else:
                self.p = PendulumParams()
                self.params_fallback = True
                self.startup_warning = (f"{self.params_path} bulunamadı; GENEL varsayılan parametreler kullanılıyor. "
                                        "K gönderilmez ve gains.h dışa aktarılmaz; Tasarım sekmesinden bir parametre "
                                        "dosyası yükleyin.")
        except (ValueError, TypeError, OSError) as e:   # bozuk/geçersiz dosya: varsayılanlarla aç, uyar
            self.p = PendulumParams()
            self.params_fallback = True
            self.startup_warning = (f"{self.params_path} okunamadı ({e}); GENEL varsayılan parametreler kullanılıyor. "
                                    "K gönderilmez ve gains.h dışa aktarılmaz; Tasarım sekmesinden bir parametre "
                                    "dosyası yükleyin.")
        self.Q = [10.0, 1.0, 100.0, 1.0]
        self.R = 0.1
        self.Ts = 0.002
        self.swingup = EnergySwingUp()
        self.catch_deg = CATCH_DEG      # LQR'a geçiş / bırakma açıları [°] (supervisor.py'deki ortak varsayılan)
        self.release_deg = RELEASE_DEG
        self.catch_dth = CATCH_DTH      # LQR'a girişte |θ̇| sınırı [rad/s]
        self.catch_x = CATCH_X          # LQR'a girişte |x| sınırı [m]
        self.catch_dx = CATCH_DX        # LQR'a girişte |ẋ| sınırı [m/s]
        self.swingup_enabled = True
        self.x_ref = 0.0
        self.device_Ts: float | None = None     # cihazın bildirdiği kontrol periyodu
        self.device_info_seen = False
        self.device_theta = None
        self._last_info = -1e9
        self.K = np.zeros(4)
        self.cl_poles = np.array([])
        self.reader = None
        self.recorder = None
        self.gains_changed.connect(self._record_design_snapshot)
        self.compute_gains()

    def compute_gains(self):
        K, _, ev = design_lqr(self.p, self.Q, [[self.R]], Ts=self.Ts)
        self.K, self.cl_poles = K.ravel(), ev
        self.gains_changed.emit()

    # ------------------------------------------------------------ bağlantı
    @property
    def connected(self) -> bool:
        return self.reader is not None

    @property
    def real_device(self) -> bool:
        """Gerçek karta (seri port) bağlı mı; sanal ESP32'de False."""
        return self.reader is not None and not isinstance(self.reader.transport, SimTransport)

    @property
    def theta_mismatch(self) -> bool:
        """Bağlı cihaz θ kuralını bizimkiyle aynı bildirmedi (θ'yı bildirmeyen eski firmware dahil).
        Cihaz bilgisi (ping yanıtı / boot) henüz gelmediyse de True: K yalnız olumlu onaydan sonra gider."""
        return self.reader is not None and self.device_theta != P.THETA_CONVENTION

    @property
    def placeholder_on_real_device(self) -> bool:
        """Yer tutucu parametrelerle hesaplanmış K gerçek motoru sürebilir mi (uyarı gerekir)."""
        return self.real_device and self.p.placeholder

    def set_reader(self, reader):
        self.reader = reader
        self.device_Ts = None
        self.device_info_seen = False   # ping yanıtı / boot olayı geldi mi
        self.device_theta = None        # cihazın bildirdiği θ kuralı ("sol"); eski firmware bildirmez
        self._last_info = -1e9          # yeni bağlantı: ilk cihaz bilgisi ayarları mutlaka yükler
        self.connection_changed.emit(reader is not None)

    def on_device_info(self, data: dict, boot: bool = False):
        """ping yanıtı veya boot olayı geldi: Ts'yi öğren ve ayarları (yeniden) yükle.
        Cihaz yeniden başladıysa PC'den gelen ayarları kaybetmiştir; bu yüzden boot'ta HER ZAMAN yüklenir.
        Bağlanırken boot olayı ve ping yanıtı art arda gelebilir: boot'tan sonraki 1 s içinde gelen ping
        yanıtı aynı ayarları ikinci kez göndermez."""
        if "Ts" in data:
            self.device_Ts = float(data["Ts"])
        self.device_info_seen = True             # boot ya da ping yanıtı
        self.device_theta = data.get("theta")
        now = time.monotonic()
        if not boot and now - self._last_info < 1.0:
            return
        self._last_info = now
        ts = f"  Ts={self.device_Ts * 1000:g} ms" if self.device_Ts else "  (Ts bildirilmedi)"
        self.log.emit(f"Cihaz: fw={data.get('fw', '?')}{ts}")
        if self.theta_mismatch:
            self.log.emit(f"UYARI: cihaz θ kuralını bildirmedi ya da farklı bildirdi (theta={self.device_theta!r}); "
                          f"bu arayüz θ'yı '{P.THETA_CONVENTION}' (sola yatınca pozitif, #50) sayar. Eski firmware "
                          "olabilir: K GÖNDERİLMEZ (θ kazançları ters çalışır, sarkaç düşer); θ grafikleri ve "
                          "animasyon ters işaretlidir; kayıtlara cihaz_theta_pozitif: sag yazılır (oynatmada çevrilir). "
                          "Firmware'i güncelleyin.")
        if data.get("angle_ok") is False:      # açılıştaki calib olayı bağlanmadan önce gelip kaybolmuş olabilir
            self.log.emit("UYARI: cihazın açı kalibrasyonu BAŞARISIZ — sarkacı aşağıda hareketsiz tutup "
                          "'Açı Sıfırla (calib)'a basın; olmadan BAŞLAT reddedilir.")
        if self.placeholder_on_real_device:
            self.send_swingup()                  # güvenlik: K gitmese de swing-up kapalıysa kart bunu bilmeli
            self.log.emit("UYARI: parametreler YER TUTUCU — K ve ayarlar (swing-up açık/kapalı hariç) gerçek karta "
                          "otomatik GÖNDERİLMEDİ; "
                          "kart derlendiği gains.h ile çalışır. Ölçülmüş değerleri girin ya da "
                          "'K'yı Cihaza Gönder' ile onaylayarak gönderin.")
            return
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

    def push_to_device(self, force: bool = False) -> bool:
        """Güncel K ve tüm swing-up/geçiş ayarlarını (swing-up açık/kapalı ve x_ref dahil) cihaza yükler.
        K, cihazın kontrol periyodu için tasarlanmamışsa GÖNDERİLMEZ. Yer tutucu parametrelerle gerçek
        karta yalnız force=True ile (kullanıcı onayıyla) gönderilir.
        Swing-up açık/kapalı anahtarı güvenlikle ilgili olduğu için K'yı durduran engellerden ÖNCE ve her
        durumda gönderilir (kart resetlenince firmware varsayılanı olan "açık"a döner)."""
        self.send_swingup()
        if self.p.input_mode != "voltage":
            # firmware yalnız DC motor (gerilim girişi) için yazıldı; kuvvet/ivme için tasarlanan K'nın birimi
            # ve ölçeği farklıdır, cihazda arabayı raydan kaçırır (gains.h dışa aktarımı da #error verir)
            self.log.emit(f"K gönderilmedi: giriş türü '{self.p.input_mode}'. Cihaz yalnız 'voltage' (DC motor) "
                          "girişini destekler — Tasarım sekmesinde giriş türünü voltage yapıp K'yı yeniden hesaplayın.")
            return False
        if self.params_fallback:
            self.log.emit("K gönderilmedi: parametre dosyası okunamadı, genel varsayılanlar kullanılıyor "
                          "(motor/kasnak projeninkinden farklı). Tasarım sekmesinden bir parametre dosyası yükleyin.")
            return False
        if self.theta_mismatch:
            if not self.device_info_seen:
                self.send("ping")                # yanıt kaybolmuş olabilir: cihaz bilgisini yeniden iste
                self.log.emit("K gönderilmedi: cihaz bilgisi (ping yanıtı) henüz gelmedi; θ kuralı doğrulanamadı. "
                              "Cihaz bilgisi yeniden istendi; birkaç saniye sonra tekrar deneyin (gelmezse yeniden "
                              "bağlanın).")
            else:
                self.log.emit("K gönderilmedi: cihazın θ kuralı bu arayüzünkiyle aynı değil (eski firmware?). "
                              "Firmware'i güncelleyip yeniden bağlanın.")
            return False
        if self.placeholder_on_real_device and not force:
            self.log.emit("K gönderilmedi: parametreler yer tutucu ve gerçek karta bağlısınız.")
            return False
        if self.device_Ts is not None and abs(self.device_Ts - self.Ts) > 1e-9:
            self.log.emit(f"UYARI: cihaz Ts = {self.device_Ts * 1000:g} ms, tasarım Ts = {self.Ts * 1000:g} ms. "
                          "K gönderilmedi — Tasarım sekmesinde Ts'yi cihazla aynı yapıp tekrar hesaplayın.")
            return False
        su = self.swingup
        self.x_ref = self.clamp_xref(self.x_ref)
        # Cihaz set_param'ı "hepsi ya da hiçbiri" uygular; güvenlik açısından önemli swing-up anahtarı
        # başka bir alanın reddiyle kaybolmasın diye AYRI mesajlarla gönderilir.
        # Her mesaj bağımsız gönderilir: biri yerelde reddedilirse (ör. aralık dışı) diğerleri yine gider,
        # cihaz yarı güncellenmiş kalmaz; gönderilemeyenler topluca bildirilir.
        msgs = [("set_gain", dict(K=self.K.tolist())),
                ("set_param", dict(catch_deg=self.catch_deg, release_deg=self.release_deg)),
                ("set_param", dict(catch_dth=self.catch_dth, catch_x=self.catch_x, catch_dx=self.catch_dx)),
                ("set_param", dict(k_E=su.k_E, a_max=su.a_max, k_x=su.k_x, k_v=su.k_v, E_ratio=su.E_ratio)),
                ("set_param", dict(u_max=self.p.u_max))]
        if self.p.input_mode == "voltage":       # Coulomb telafisi, gerilim olarak (firmware: fc_N = fc_volt·α)
            msgs.append(("set_param", dict(fc_volt=min(6.0, self.p.F_coulomb / self.p.motor.alpha))))
        msgs.append(("set_param", dict(x_ref=self.x_ref)))
        failed = [", ".join(f) for cmd, f in msgs if not self.send(cmd, **f)]
        if failed:
            self.log.emit("UYARI: şu ayarlar GÖNDERİLEMEDİ, cihaz eski değerlerini kullanıyor: " + "; ".join(failed))
            return False
        self.log.emit("K = [" + ", ".join(f"{k:.3f}" for k in self.K) + "] ve ayarlar gönderildi "
                      "(reddedilen olursa aşağıda ✗ ile görünür).")
        return True

    def clear_params_fallback(self):
        """Bir parametre dosyası yüklendi: K gönderimi ve dışa aktarma yeniden serbest (kaydetmek kaldırmaz)."""
        self.params_fallback = False

    def send_swingup(self) -> bool:
        """Swing-up açık/kapalı anahtarını tek başına gönderir (bağlıysa)."""
        if self.reader is None:
            return False
        return self.send("set_param", swingup=int(self.swingup_enabled))

    def clamp_xref(self, x: float) -> float:
        """Cihazın kuralları: |x_ref| ≤ rail_half − 0.1 m ve protokol aralığı (SET_PARAM_LIMITS x_ref)."""
        lim = max(0.0, min(P.SET_PARAM_LIMITS["x_ref"][1], self.p.rail_half - 0.1))
        return max(-lim, min(lim, x))

    def get_metadata(self) -> dict:
        """CSV başlığı için tüm parametre, kazanç ve ayarları sözlük olarak döndürür."""
        su = self.swingup
        return {
            "schema_version": 1,
            # cihazın θ kuralı: eski (θ sağa +) firmware'e bağlıyken telemetri eski kuraldadır, load_recording
            # yalnız θ, θ̇ sütunlarını çevirir. Bilgi gelmeden kayıt başlatılmaz (live_tab.toggle_record).
            "cihaz_theta_pozitif": (self.device_theta if self.device_theta == P.THETA_CONVENTION else "sag")
                                   if self.device_info_seen else None,
            "settings_scope": "PC tasarım anlık görüntüsü; gönderilen komutlar ve cihaz ACK/olayları event satırlarında",
            "device_Ts": self.device_Ts,
            "device_acknowledged_settings": dict(getattr(self.reader, "applied_settings", {})),
            "units": {"t": "s", "x": "m", "dx": "m/s", "th": "rad", "dth": "rad/s",
                      "u": {"voltage": "V", "force": "N", "acceleration": "m/s^2"}[self.p.input_mode]},
            "summary_definition": {"threshold_deg": 2.0, "min_duration_s": 1.0,
                                   "time_origin": "ilk WAIT/SWING_UP/BALANCE telemetrisi",
                                   "saturation_integral": "sol örnek sabit; eksik sıra numarası ve duraklatma aralığı hariç",
                                   "max_telemetry_gap_s": 1.5},
            "K": self.K.tolist(),
            "Q": list(self.Q),
            "R": float(self.R),
            "Ts": float(self.Ts),
            "catch_deg": float(self.catch_deg),
            "release_deg": float(self.release_deg),
            "catch_dth": float(self.catch_dth),
            "catch_x": float(self.catch_x),
            "catch_dx": float(self.catch_dx),
            "swingup_enabled": bool(self.swingup_enabled),
            "x_ref": float(self.x_ref),
            "swingup": {
                "k_E": float(su.k_E),
                "a_max": float(su.a_max),
                "k_x": float(su.k_x),
                "k_v": float(su.k_v),
                "a_kick": float(su.a_kick),
                "E_ratio": float(su.E_ratio),
            },
            "params": self.p.to_dict(),
        }

    def _record_design_snapshot(self):
        if self.recorder is not None:
            try:
                self.recorder.write_event("pc_design_snapshot", {"settings": self.get_metadata()})
            except (OSError, ValueError) as exc:
                self.log.emit(f"Tasarım ayarları kayda yazılamadı: {exc}")
