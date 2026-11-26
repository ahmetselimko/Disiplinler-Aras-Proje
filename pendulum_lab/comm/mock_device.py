"""Sanal ESP32: firmware'in yapacağı her şeyi gerçek zamanlı simüle eder.

- 500 Hz kontrol döngüsü (Supervisor: swing-up + LQR), sensör kuantalama, türevle hız kestirimi
- homing: sol limit switch'e yavaşça gider, sıfırı ayarlar, rayın ortasına döner (süre sınırlı;
  yanlış taraftaki switch'e çarparsa FAULT)
- fiziksel ray sonları (limit switch olayı + FAULT)
- end_sensors (firmware END_SENSORS ile aynı, #50 Aşama 4):
  0: uç sensörü yok. "home" aracı hareket ettirmez; araba durağansa bulunduğu nokta x = 0 olur ("elle
     ortalama"). Ray uçları yalnız mekanik durdurucudur (olay/FAULT yok); güvenliği yazılımsal ray sınırı sağlar
  1: her uçta tek sensör (ray sonunda): homing referansı ve "limit" FAULT'u
  2: her uçta iki kademe: içte optik (OPT_FROM_END kadar içeride; homing referansı, çalışırken "limit"
     FAULT'u), dışta mikro switch (ray sonunda; çalışırken "hard_limit", homing'de "homing_overrun"; basılıyken
     start/home/friction her modda reddedilir; aynı taraftaki optik görmüyorsa "sensor_mismatch" olayı)
  limit_switches=False eski adla end_sensors=0 demektir
- protokoldeki tüm komutlar, firmware ile AYNI kabul/ret kurallarıyla; ek olarak yalnız
  simülasyonda "kick" (sarkacı dürt), "pause" (fiziği dondur/sürdür) ve "reset" (başlangıç durumuna dön)
- gyro=True: MPU6050 jiroskobu (firmware USE_MPU6050 1): telemetride "gy", set_param gyro_mix (türev bölgesinde
  θ̇ = γ·gy + (1 − γ)·θ̇_türev; Kalman bölgesinde Kalman aynen), calib olayında gyro_bias. gyro_fail = True
  okumaları bozar: MPU_FAIL_LIMIT okumadan sonra FAULT YOK, karışım 0'a düşer, {"evt":"gyro","ok":false} bir kez
- "friction": sürtünme ölçümü (control/friction.py). Ölçülen telafi (set_param fc_volt) yalnız
  kontrolcünün modelini (sup.p) değiştirir; fiziği (self.p) değiştirmez

Arayüz ve okuyucu, gerçek kart gelmeden bu cihazla uçtan uca geliştirilir/test edilir.
"""

from __future__ import annotations

import math
import random
import threading
import time

import numpy as np

import copy

from ..control import friction as FR
from ..control.lqr import design_lqr
from ..control.supervisor import Mode, Supervisor
from ..filters import KalmanFilter, VelocityEstimator
from ..model.dynamics import accel_to_input
from ..model.params import PendulumParams
from ..sim.simulator import SensorModel, mix_gyro, rk4_step
from . import protocol as P

DEFAULT_Q = (10.0, 1.0, 100.0, 1.0)
DEFAULT_R = 0.1
# bağlantı bekçisinin izlediği modlar (motor hareket ettirilirken); firmware'deki linkWatched() ile aynı
LINK_WATCH_MODES = (Mode.HOMING, Mode.SWING_UP, Mode.WAIT, Mode.BALANCE, Mode.FRICTION)
HOME_SPEED = 0.10        # [m/s]
HOME_TIMEOUT = 20.0      # [s]
HOME_KP = 20.0           # homing hız döngüsü oransal kazancı [1/s]
HOME_KI = 30.0           # integral kazancı [1/s²]: araba hedef hıza ulaşamadıkça itme artar → sürtünmeyi yener (#30)
HOME_RUNAWAY_V = 0.30    # homing'de |ẋ| bunu aşarsa (normal en çok 0.15) kontrol kaçmıştır: yön ters → FAULT
HOME_SAT_S = 0.30        # homing'de gerilim bu süreden uzun doyumda kalırsa → FAULT (firmware ile aynı)
HOME_I_MAX = 10.0        # integral sınırı [m/s²] (yer tutucu değerlerle arabada ≈ 5 N)
HOME_STILL_DX = 0.01     # elle ortalamada araba hızı bundan düşük olmalı [m/s] (firmware ile aynı)
HOME_SW_CLEAR = 0.05     # sol switch'ten bu kadar sağa gidildiği hâlde hâlâ basılıysa sensör takılı → FAULT (firmware ile aynı)
HOME_BACKOFF = 0.01      # END_SENSORS 2: homing optik etkinken başlarsa optik bırakıldıktan sonra bu kadar daha sağa (firmware ile aynı)
MPU_FAIL_LIMIT = 3       # MPU6050 art arda bu kadar okunamazsa devre dışı (firmware ile aynı)
GYRO_CHECK_RAD_S = 3.0                            # firmware tutarlılık denetimi: |gy − θ̇_türev| eşiği
GYRO_CHECK_N = 25                                 # art arda adım (50 ms)
MPU_PWR_CHECK_N = 50                              # firmware: bu kadar adımda bir PWR_MGMT_1 (uyku/reset) okunur
MOCK_I2C_US = 250                                 # ping i2c_us: sanal değer (400 kHz tahmini)
MPU_BIAS_SAMPLES = 200   # calib: jiroskop bias ortalaması (firmware ile aynı)
OPT_FROM_END = 0.03      # END_SENSORS 2: optik sensör, mikro switch'ten (ray sonundan) bu kadar içeride [m] (firmware ile aynı)
FW_NAME = "mock-1.2"     # 1.2: θ sola + (#50); firmware gibi boot/ping'de "theta":"sol"


class MockDevice:
    def __init__(self, params: PendulumParams | None = None, Ts: float = 0.002, telemetry_hz: int = 200,
                 seed: int | None = None, realtime: bool = True, x0: float | None = None,
                 limit_switches: bool = True, link_timeout: float | None = None, enc_sign: float = 1.0,
                 end_sensors: int | None = None, gyro: bool = False):
        self.end_sensors = (1 if limit_switches else 0) if end_sensors is None else int(end_sensors)
        if self.end_sensors not in (0, 1, 2):
            raise ValueError("end_sensors 0, 1 ya da 2 olmalı")
        self.limit_switches = self.end_sensors > 0
        # test: "L"/"R" homing referans sensörü (1: switch, 2: optik) hep etkin okur (kopuk kablo, NC/TCST2103)
        self.stuck_switch: str | None = None
        self.stuck_hard: str | None = None      # test (end_sensors 2): "L"/"R" mikro switch hep basılı
        self._sens_prev = {}                    # sensör seviyelerinin önceki adımı (olaylar yalnız kenarda)
        # enc_sign = −1: enkoder ters bağlanmış gibi (firmware'de X_SIGN yanlış) — kablolama hatası testleri için
        self.enc_sign = enc_sign
        self._home_sat = 0.0
        # bağlantı bekçisi (protokol: hb, LINK_TIMEOUT_S). None: kapalı (testler adım adım sürer, hb göndermez);
        # arayüzün sanal ESP32'si firmware gibi P.LINK_TIMEOUT_S ile açar.
        self.link_timeout = link_timeout
        self._last_rx_k = 0
        self._calib_evt = False
        self.p = params or PendulumParams()
        self.Ts = Ts
        self.telemetry_hz = telemetry_hz
        self.realtime = realtime
        self.output = lambda data: None          # transport bağlar
        rng = random.Random(seed)
        # gerçek (fiziksel) durum; araba açılışta bilinmeyen bir yerde
        x_start = rng.uniform(-0.15, 0.15) if x0 is None else x0
        self.s = [x_start, 0.0, math.pi + rng.uniform(-0.03, 0.03), 0.0]
        self.x_offset = self.s[0]                 # enkoder açılışta 0 okur
        self.homed = False
        K, _, _ = design_lqr(self.p, DEFAULT_Q, [[DEFAULT_R]], Ts=Ts)
        self.sup = Supervisor(self.p, K)
        self.sensor = SensorModel.for_params(self.p, seed=seed, gyro=gyro)
        # MPU6050 (firmware USE_MPU6050): açılışta calib bias'ı ölçer (sanal: gerçek bias + küçük ölçüm hatası)
        self.gyro = gyro
        self.mpu_ok = gyro
        self.gyro_fail = False                    # test: okumalar başarısız (kopuk kablo / I²C hatası)
        self.gyro_frozen = False                  # test: MPU resetlenip uykuda: okuma başarılı ama ham değer 0
        self._mpu_bad = 0
        self._mpu_k = 0
        self._mpu_fails = 0
        self.gyro_mix = 0.0
        self._gy = 0.0
        self._gyro_bias = self._measure_gyro_bias() if gyro else 0.0
        self.vx = VelocityEstimator(Ts)
        self.vth = VelocityEstimator(Ts, angular=True)
        self.kf = KalmanFilter.from_params(self.sup.p, Ts)
        self._in_kf = False
        self._u_prev = 0.0
        self.k = 0
        self.n = 0
        self._tel_acc = 0.0
        self._home_phase = 0
        self._home_t0 = 0.0
        self._home_r0 = False
        self._home_i = 0.0                        # homing hız döngüsünün integrali [m/s²]
        self._manual_home = False                 # switch'siz: bir sonraki adımda bulunulan yer x = 0
        self._kick_F, self._kick_left = 0.0, 0.0  # süren dürtme: kuvvet [N] ve kalan süre [s]
        self._kick_at = "pole"                    # dürtmenin uygulandığı yer: "pole" (sarkaç) / "cart" (araba)
        self._rx = bytearray()
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._running = False
        self.paused = False                       # yalnız simülasyon: fizik ve kontrol donuk
        self._last_tel: tuple | None = None       # duraklatmada tekrar gönderilen son ölçüm (x, ẋ, θ, θ̇, u)
        self._friction: FR.FrictionTest | None = None
        self._s_hat = [0.0, 0.0, math.pi, 0.0]    # son ölçülen/kestirilen durum (komut kabul kuralları için)

    # ------------------------------------------------------------ yaşam döngüsü
    def start(self):
        self._running = True
        self._emit(P.encode_event("boot", fw=FW_NAME, theta=P.THETA_CONVENTION, Ts=self.Ts))
        if self.mpu_ok:                                  # firmware: açılış kalibrasyonunun sonucu (protokol §3.2)
            self._emit(P.encode_event("calib", ok=True, gyro_bias=round(self._gyro_bias, 5)))
        else:
            self._emit(P.encode_event("calib", ok=True))
        if self.realtime:
            self._thread = threading.Thread(target=self._run, name="MockDevice", daemon=True)
            self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)

    def _emit(self, data: bytes):
        self.output(data)

    def feed(self, data: bytes):
        """PC'den gelen baytlar (seri RX)."""
        with self._lock:
            self._rx.extend(data)

    def _run(self):
        t0 = time.perf_counter()
        while self._running:
            due = int((time.perf_counter() - t0) / self.Ts)
            # Windows'ta sleep ~1–2 ms oynar; geride kalınan adımlar telafi edilir (en çok 50 adım)
            for _ in range(min(due - self.k, 50)):
                self.step()
            if due - self.k > 50:
                self.k = due  # aşırı gecikmede yetişmeye çalışma, atla
            time.sleep(0.001)

    # ------------------------------------------------------------ hata
    def _fault(self, reason: str):
        """FAULT'a geç ve olayı YALNIZ BİR KEZ, sebebiyle gönder."""
        if self.sup.mode != Mode.FAULT:
            self.sup.mode = Mode.FAULT
            self._home_phase = 0
            self._emit(P.encode_event("fault", reason=reason))

    # ------------------------------------------------------------ komutlar
    def _process_commands(self):
        with self._lock:
            if b"\n" not in self._rx:
                return
            *lines, rest = bytes(self._rx).split(b"\n")
            self._rx = bytearray(rest)
        for line in lines:
            if not line.strip():
                continue
            self._last_rx_k = self.k               # bağlantı bekçisi: herhangi bir satır bağlantının canlı olduğunu gösterir
            try:
                cmd, args = P.parse_command(line)
                reply = self._handle(cmd, args)
                if reply:                          # hb yanıtsız
                    self._emit(reply)
                if self._calib_evt:                # firmware gibi: önce ACK, sonra calib olayı
                    self._calib_evt = False
                    if self.mpu_ok and self.gyro_fail:  # firmware: bias okunamadı → jiroskop kapanır
                        self.mpu_ok = False
                        self._emit(P.encode_event("gyro", ok=False))
                    if self.mpu_ok:                    # firmware: calib jiroskop sıfırını da ölçer
                        self._gyro_bias = self._measure_gyro_bias()
                        self._emit(P.encode_event("calib", ok=True, gyro_bias=round(self._gyro_bias, 5)))
                    else:
                        self._emit(P.encode_event("calib", ok=True))
            except (P.ProtocolError, ValueError, TypeError, KeyError) as e:
                # bozuk alanlı komut (ör. "F": null) cihazı öldürmesin; firmware de "?" ile reddeder
                self._emit(P.encode_ack("?", False, str(e)))

    def _handle(self, cmd: str, a: dict) -> bytes:
        sup = self.sup
        mode = sup.mode
        if cmd == "hb":
            return b""
        if cmd == "ping":
            # angle_ok: sanal sensör her zaman kalibre (firmware'de açılış/calib sonucunu söyler)
            extra = {"gyro": self.mpu_ok, "i2c_us": MOCK_I2C_US} if self.gyro else {}
            return P.encode_ack("ping", True, "pong", Ts=self.Ts, fw=FW_NAME, theta=P.THETA_CONVENTION,
                                homed=self.homed, angle_ok=True, **extra)
        if cmd == "set_gain":  # çalışırken de kabul edilir (firmware kritik bölgede değiştirir)
            sup.set_gain(a.get("K", []))
            return P.encode_ack("set_gain")
        if cmd == "set_param":
            err = self._check_params(a)
            if err:
                return P.encode_ack("set_param", False, err)
            for key, val in a.items():       # hepsi geçerli → hepsini uygula
                self._apply_param(key, float(val))
            return P.encode_ack("set_param")
        hard = self._hard_side()
        if hard and cmd in ("start", "home", "friction"):   # mikro switch basılı: her modda reddedilir
            side = "sol" if hard == "L" else "sağ"
            return P.encode_ack(cmd, False, f"{side} mikro switch basılı: arabayı elle uçtan uzaklaştırın")
        if cmd == "home":
            if mode not in (Mode.IDLE, Mode.FAULT):
                return P.encode_ack("home", False, "önce stop")
            if not self.limit_switches:
                if abs(self._s_hat[1]) > HOME_STILL_DX:
                    return P.encode_ack("home", False,
                                        "araba hareket ediyor: elle ortaya koyup durunca tekrar deneyin")
                self._manual_home = True          # firmware gibi: ACK önce, sıfırlama ve "homed" olayı kontrol adımında
                return P.encode_ack("home")
            # yeni homing: eski sıfır noktası artık geçersiz (yarıda stop edilirse start reddedilmeli)
            self.homed = False
            self._home_sat = 0.0
            sup.mode, self._home_phase, self._home_t0 = Mode.HOMING, 1, self.k * self.Ts
            self._home_r0, self._home_i = self._ref("R"), 0.0
            self._home_x0 = self.enc_sign * self.s[0]          # enkoder okuması (firmware home_x0)
            self._home_rel = None
            if self.end_sensors == 2 and self._ref("L"):
                # optik zaten etkin: araba optik ile uç arasında; sıfır kaymasın diye önce optikten çekil (faz 3)
                self._home_phase = 3
            return P.encode_ack("home")
        if cmd == "calib":
            if mode != Mode.IDLE:
                return P.encode_ack("calib", False, "yalnız IDLE modunda, sarkaç aşağıda ve hareketsizken")
            self._calib_evt = True
            return P.encode_ack("calib")      # sanal sensör zaten doğru kalibre
        if cmd == "start":
            if not self.homed:
                return P.encode_ack("start", False, "önce home (kalibrasyon) yapılmalı")
            if mode == Mode.IDLE:
                # enkoder yönü ve home ofseti firmware'deki gibi uygulanır; sadeleştirme: kuantalama ve hız
                # filtresi yok (firmware S.lx/S.ldx kullanır, fark 0.2 m/s eşiğinde birkaç ms gecikme)
                blocked = sup.start_block_reason(self.enc_sign * self.s[0] - self.x_offset, self.enc_sign * self.s[1])
                if blocked:
                    return P.encode_ack("start", False, blocked)
            ok, msg = sup.start()
            return P.encode_ack("start", ok, msg)
        if cmd == "friction":
            if not self.homed:
                return P.encode_ack("friction", False, "önce home (kalibrasyon) yapılmalı")
            if mode != Mode.IDLE:
                return P.encode_ack("friction", False, "yalnız IDLE modunda")
            blocked = FR.start_block_reason(self.p, *self._s_hat)
            if blocked:
                return P.encode_ack("friction", False, blocked)
            sup.mode, self._friction = Mode.FRICTION, FR.FrictionTest(self._s_hat[0])
            return P.encode_ack("friction")
        if cmd == "stop":
            sup.stop()
            self._home_phase = 0
            return P.encode_ack("stop")
        if cmd == "estop":
            self._fault("estop")
            return P.encode_ack("estop")
        if cmd == "kick":
            if "F" not in a:                      # eski biçim: açısal hıza anlık ekleme
                self.s[3] += float(a.get("dth", 1.5))
                return P.encode_ack("kick")
            F, dur, at = float(a["F"]), float(a.get("dur", 0.05)), str(a.get("at", "pole"))
            if not (P.KICK_F_LIMITS[0] <= F <= P.KICK_F_LIMITS[1]):
                return P.encode_ack("kick", False, "F aralık dışı [{}, {}] N".format(*P.KICK_F_LIMITS))
            if not (P.KICK_DUR_LIMITS[0] <= dur <= P.KICK_DUR_LIMITS[1]):
                return P.encode_ack("kick", False, "dur aralık dışı [{}, {}] s".format(*P.KICK_DUR_LIMITS))
            if at not in P.KICK_TARGETS:
                return P.encode_ack("kick", False, "at şunlardan biri olmalı: " + ", ".join(P.KICK_TARGETS))
            self._kick_F, self._kick_left, self._kick_at = F, dur, at
            return P.encode_ack("kick")
        if cmd == "pause":
            self.paused = bool(int(a.get("on", 1)))
            return P.encode_ack("pause")
        if cmd == "reset":
            th0 = float(a.get("th0_deg", 180.0))
            if not -180.0 <= th0 <= 180.0:
                return P.encode_ack("reset", False, "th0_deg −180…180 aralığında olmalı")
            self._reset(math.radians(th0))
            return P.encode_ack("reset")
        return P.encode_ack(cmd, False, "desteklenmiyor")

    def _reset(self, th0: float):
        """IDLE'a dön, duraklatmayı kaldır, sarkacı θ₀'da durağan koy; kalibre edildiyse araba x = 0 noktasına."""
        self.sup.stop()
        self._home_phase = 0
        self.paused = False
        self._kick_F, self._kick_left = 0.0, 0.0
        self.s = [self.x_offset if self.homed else self.s[0], 0.0, th0, 0.0]   # ölçülen x = 0 noktası
        for est in (self.vx, self.vth):           # açı sıçraması sahte bir hız tepesi üretmesin
            est.prev, est.lp.y = None, None
        self.kf.reset()
        self._in_kf = False
        self._u_prev = 0.0

    def _check_params(self, a: dict) -> str:
        """Firmware'deki doğrulamanın aynısı: protokol aralıkları + cihaza özgü sınırlar."""
        try:
            P.validate_params(a)
        except P.ProtocolError as e:
            return str(e)
        catch = float(a.get("catch_deg", math.degrees(self.sup.catch)))
        release = float(a.get("release_deg", math.degrees(self.sup.release)))
        if catch >= release:
            return "catch_deg < release_deg olmalı"
        if "x_ref" in a and abs(float(a["x_ref"])) > self.p.rail_half - 0.1 + 1e-4:   # firmware ile aynı tolerans
            return "x_ref ray sınırına çok yakın"
        if "u_max" in a and self.p.input_mode == "voltage" and float(a["u_max"]) > self.p.motor.V_max:
            return "u_max besleme gerilimini aşamaz"
        return ""

    def _apply_param(self, key: str, val: float):
        sup, su = self.sup, self.sup.swingup
        if key == "catch_deg":
            sup.catch = math.radians(val)
        elif key == "release_deg":
            sup.release = math.radians(val)
        elif key in ("catch_dth", "catch_x", "catch_dx"):
            setattr(sup, key, val)
        elif key == "swingup":
            sup.swingup_enabled = bool(val)
        elif key in ("k_E", "a_max", "k_x", "k_v", "E_ratio"):
            setattr(su, key, val)
        elif key == "x_ref":
            sup.x_ref = val
        elif key == "u_max":
            # fizik ve kontrolcünün modeli: fc_volt sup.p'yi kopyaladıktan sonra da ikisi birlikte değişmeli
            # (firmware tek bir u_max kullanır)
            self.p.u_max = val
            sup.p.u_max = val
        elif key == "telemetry_hz":
            self.telemetry_hz = int(val)
        elif key == "gyro_mix":
            self.gyro_mix = val                   # jiroskop yoksa etkisiz (firmware USE_MPU6050 0 gibi)
        elif key == "fc_volt":
            # telafi kontrolcünün modelinde: F_c = α·V_c → T_f = F_c·r/N (firmware: P_FC yerine fc_volt·α)
            m = self.p.motor
            sup.p = copy.deepcopy(sup.p)
            sup.p.motor.T_f = val * m.alpha * m.r_pulley / m.gear

    # ------------------------------------------------------------ 500 Hz döngü
    def _at(self, side: str, from_end: float) -> bool:
        """Araba (gerçek konum) o uçta, ray sonundan from_end kadar içerideki noktayı geçti mi."""
        lim = self.p.rail_half - from_end
        return self.s[0] <= -lim + 1e-9 if side == "L" else self.s[0] >= lim - 1e-9

    def _ref(self, side: str) -> bool:
        """Homing referansı ve çalışırken "limit" FAULT'unu veren sensör: 1'de switch, 2'de optik."""
        if self.end_sensors == 0:
            return False
        if self.stuck_switch == side:
            return True
        return self._at(side, OPT_FROM_END if self.end_sensors == 2 else 0.0)

    def _hard(self, side: str) -> bool:
        """END_SENSORS 2: dış kademe mikro switch (ray sonunda)."""
        if self.end_sensors != 2:
            return False
        return self.stuck_hard == side or self._at(side, 0.0)

    def _hard_side(self) -> str | None:
        return "L" if self._hard("L") else "R" if self._hard("R") else None

    _switch = _ref                              # eski ad (testler)

    def _homing_input(self, s_hat) -> float:
        p = self.p
        if self.k * self.Ts - self._home_t0 > HOME_TIMEOUT:
            self._fault("homing_timeout")
            return 0.0
        if self._hard_side():                 # optik kaçırıldı ya da çalışmıyor: araba ray sonunda
            self._fault("homing_overrun")
            return 0.0
        if not self._ref("R"):
            self._home_r0 = False
        elif not self._home_r0 or self.k * self.Ts - self._home_t0 > 1.0:
            # yanlış taraf (motor/enkoder yönü ters). Başta sağ switch basılıysa (ör. sağ limit hatasından
            # sonra) 1 s içinde sola çekilip bırakması beklenir.
            self._fault("homing_wrong_side")
            return 0.0
        if self._home_phase == 3:             # optikten geri çekil (yalnız END_SENSORS 2)
            # firmware ile aynı else-if sırası; geçiş adımlarında firmware motoru bir adım sürmez (u = 0)
            x_enc = self.enc_sign * self.s[0]
            if self._ref("L"):
                if x_enc - self._home_x0 > HOME_SW_CLEAR:
                    self._fault("homing_switch")          # optik bırakmıyor: takılı / kopuk
                    return 0.0
                if self._home_rel is not None:
                    self._home_rel = None                 # optik yeniden etkin: baştan say
                    return 0.0
            elif self._home_rel is None:
                self._home_rel = x_enc
                return 0.0
            elif x_enc - self._home_rel >= HOME_BACKOFF:
                self._home_phase, self._home_i = 1, 0.0
                return 0.0
            v_des = HOME_SPEED
        elif self._home_phase == 1:
            if self._ref("L"):                # switch SEVİYESİ (zaten basılıysa da çalışır)
                # referans sensörün konumu: 1'de −rail_half, 2'de optik (OPT_FROM_END kadar içeride)
                self.x_offset = self.enc_sign * self.s[0] + p.rail_half - self._ref_from_end()
                self._home_phase, self._home_i = 2, 0.0
                self.vx.prev = None
                self.kf.reset()
                self._in_kf = False
                self._u_prev = 0.0
                return 0.0
            v_des = -HOME_SPEED
        else:                                 # ortaya dön
            if self._ref("L") and s_hat[0] > -p.rail_half + self._ref_from_end() + HOME_SW_CLEAR:
                self._fault("homing_switch")    # sol sensör hep "basılı": sıfır yanlış olurdu
                return 0.0
            v_des = max(-0.15, min(0.15, -2.0 * s_hat[0]))
            if abs(s_hat[0]) < 0.002 and abs(s_hat[1]) < 0.02:
                self.sup.mode = Mode.IDLE
                self._home_phase = 0
                self.homed = True
                self._emit(P.encode_event("homed"))
                return 0.0
        started = self.k * self.Ts - self._home_t0 > 0.2   # ilk 0.2 s: hız kestirimi oturuyor
        if started and abs(s_hat[1]) > HOME_RUNAWAY_V:   # motor/enkoder yönü ters: hız döngüsü kontrolden çıktı
            self._fault("homing_runaway")
            return 0.0
        e = v_des - s_hat[1]
        self._home_i = max(-HOME_I_MAX, min(HOME_I_MAX, self._home_i + HOME_KI * e * self.Ts))
        # Coulomb telafisi yok: sürtünmeyi integral karşılar, T_f tahmini yanlış olsa da homing biter (#30).
        # Firmware gibi kontrolcünün modeli (sup.p) kullanılır, fiziğin gerçek sürtünmesi (self.p) değil.
        u = accel_to_input(self.sup.p, s_hat, HOME_KP * e + self._home_i, coulomb=False)
        self._home_sat = self._home_sat + self.Ts if abs(u) >= 0.98 * p.u_max else 0.0
        if self._home_sat > HOME_SAT_S:
            self._fault("homing_runaway")
            return 0.0
        return max(-p.u_max, min(p.u_max, u))

    def _measure_gyro_bias(self) -> float:
        """calib: sarkaç hareketsizken MPU_BIAS_SAMPLES örneğin ortalaması (firmware calibrateGyro)."""
        rest = [self.s[0], 0.0, self.s[2], 0.0]
        return sum(self.sensor.measure_gyro(rest) for _ in range(MPU_BIAS_SAMPLES)) / MPU_BIAS_SAMPLES

    def _ref_from_end(self) -> float:
        return OPT_FROM_END if self.end_sensors == 2 else 0.0

    def step(self):
        self._process_commands()
        if (self.link_timeout is not None and self.sup.mode in LINK_WATCH_MODES
                and (self.k - self._last_rx_k) * self.Ts > self.link_timeout):
            self._fault("link")                   # PC'den satır gelmiyor: USB koptu / arayüz çöktü
        if self._manual_home:
            self._manual_home = False
            self.x_offset = self.enc_sign * self.s[0]   # enkoderin şu anki okuması x = 0
            self.vx.prev = None
            self.sup.mode, self._home_phase, self.homed = Mode.IDLE, 0, True
            self._emit(P.encode_event("homed"))
        if self.paused:
            self._paused_step()
            return
        p, sup = self.p, self.sup
        xm, thm = self.sensor.measure([self.enc_sign * self.s[0] - self.x_offset, 0, self.s[2], 0])
        dx_lp, dth_lp = self.vx.update(xm), self.vth.update(thm)
        if self.mpu_ok:                           # firmware: türev bölgesinde karışım; Kalman bölgesinde Kalman
            if self.gyro_fail:
                self._mpu_fails += 1
                if self._mpu_fails >= MPU_FAIL_LIMIT:
                    self.mpu_ok = False           # FAULT yok: yedek sensör
                    self._emit(P.encode_event("gyro", ok=False))
            else:
                self._mpu_fails = 0
                raw = 0.0 if self.gyro_frozen else self.sensor.measure_gyro(self.s)
                self._gy = raw - self._gyro_bias
                # firmware ile aynı tutarlılık denetimi (dth_lp: karışımsız türev)
                self._mpu_bad = self._mpu_bad + 1 if abs(self._gy - dth_lp) > GYRO_CHECK_RAD_S else 0
                self._mpu_k += 1                  # firmware: k % MPU_PWR_CHECK_N == 0'da PWR_MGMT_1
                if self._mpu_bad >= GYRO_CHECK_N or (self.gyro_frozen and self._mpu_k % MPU_PWR_CHECK_N == 0):
                    self.mpu_ok = False
                    self._emit(P.encode_event("gyro", ok=False))
            if self.mpu_ok:
                dth_lp = mix_gyro(dth_lp, self._gy, self.gyro_mix)
        th_wrap = math.atan2(math.sin(thm), math.cos(thm))
        if self.homed and sup.mode in (Mode.SWING_UP, Mode.WAIT, Mode.BALANCE) and abs(th_wrap) < sup.release:
            if not self._in_kf:
                self.kf.reset([xm, dx_lp, th_wrap, dth_lp])
                self._in_kf = True
            else:
                self.kf.step([xm, thm], self._u_prev)
            s_hat = [xm, float(self.kf.s[1]), thm, float(self.kf.s[3])]
        else:
            self._in_kf = False
            s_hat = [xm, dx_lp, thm, dth_lp]

        self._s_hat = s_hat
        self._end_sensors()                    # firmware: sensörler ve limit FAULT'u mod makinesinden önce
        if sup.mode == Mode.HOMING:
            u = self._homing_input(s_hat)
        elif sup.mode == Mode.FRICTION:
            u = self._friction_input(s_hat)
        elif not self.homed:
            u = 0.0                           # kalibrasyonsuz konum güvenilmez → yalnız IDLE/FAULT
        else:
            before = sup.mode
            u, mode = sup.update(s_hat)
            if mode == Mode.FAULT and before != Mode.FAULT:
                self._emit(P.encode_event("fault", reason="rail"))   # yazılımsal ray sınırı
        mode = sup.mode
        if mode in (Mode.IDLE, Mode.FAULT):
            u = 0.0
        self._u_prev = u
        u_phys = u
        if mode in (Mode.IDLE, Mode.FAULT) and p.input_mode == "voltage":
            # firmware sürücüyü (EN) kapatır → motor fren yapmaz, araba serbest; zıt EMK'yı sıfırlayan gerilim
            u_phys = p.motor.beta * self.s[1] / p.motor.alpha

        h = self.Ts / 2
        for _ in range(2):
            # süren dürtme: simulator.simulate_closed_loop'taki bozucuyla aynı model (dynamics.accelerations)
            F = self._kick_F if self._kick_left > 0 else 0.0
            f_cart, f_pole = (F, 0.0) if self._kick_at == "cart" else (0.0, F)
            self.s = rk4_step(p, self.s, u_phys, h, f_cart, f_pole)
            if self._kick_left > 0:
                self._kick_left -= h
        self._rail_end_stops()

        self.k += 1
        self._last_tel = (xm, s_hat[1], thm, s_hat[3], u)
        self._telemetry(*self._last_tel, mode)

    def _friction_input(self, s_hat) -> float:
        p = self.p
        if abs(s_hat[0]) > p.rail_half - self.sup.soft_margin:
            self._fault("rail")
            return 0.0
        u, result = self._friction.step(*s_hat, self.Ts, p.u_max)
        if result is not None:
            self.sup.mode, self._friction = Mode.IDLE, None
            self._emit(P.encode_event("friction", **result))
        return max(-p.u_max, min(p.u_max, u))

    def _paused_step(self):
        """Duraklatılmışken: fizik, sensör ve kontrol donar; saat ve telemetri (son değerlerle) sürer."""
        if self.sup.mode == Mode.HOMING:
            self._home_t0 += self.Ts                      # duraklatma homing süre sınırından sayılmasın
        self.k += 1
        if self._last_tel is not None:
            xm, dx, thm, dth, u = self._last_tel
            mode = self.sup.mode                          # duraklatmada gelen stop/estop da görünsün
            self._telemetry(xm, dx, thm, dth, 0.0 if mode in (Mode.IDLE, Mode.FAULT) else u, mode)

    def _telemetry(self, xm: float, dx: float, thm: float, dth: float, u: float, mode: Mode):
        self._tel_acc += self.telemetry_hz * self.Ts      # 200 Hz / 500 Hz = her 2.5 adımda bir
        if self._tel_acc >= 1.0:
            self._tel_acc -= 1.0
            self._emit(P.encode_telemetry(self.n, self.k * self.Ts, xm, dx, thm, dth, u, mode.value,
                                          self._gy if self.mpu_ok else None))
            self.n += 1

    def _rail_end_stops(self):
        """Fiziksel ray sonu (mekanik durdurucu)."""
        lim = self.p.rail_half
        if abs(self.s[0]) >= lim:
            self.s[0] = math.copysign(lim, self.s[0])
            self.s[1] = 0.0

    def _end_sensors(self):
        """Uç sensörleri, firmware'deki sırayla: adımın başında okunur (mod makinesinden ÖNCE); olaylar yalnız
        etkinleşme kenarında; çalışma modunda FAULT (mikro switch önce)."""
        if self.end_sensors == 0:                 # sensör yok: yalnız mekanik durdurucu
            return
        run = self.sup.mode in (Mode.SWING_UP, Mode.WAIT, Mode.BALANCE, Mode.FRICTION)
        for side in ("L", "R"):
            ref = self._at(side, self._ref_from_end())      # takılı sensör (stuck_*) olay üretmez, homing'i etkiler
            hard = self._hard(side) and self.stuck_hard != side
            if ref and not self._sens_prev.get(("ref", side)):
                if self.end_sensors == 2:
                    self._emit(P.encode_event("limit", side=side, stage="opt"))
                else:
                    self._emit(P.encode_event("limit", side=side))
            if hard and not self._sens_prev.get(("hard", side)):
                self._emit(P.encode_event("limit", side=side, stage="hard"))
                if not self._ref(side):                         # dış kademe basılı, iç kademe görmüyor
                    self._emit(P.encode_event("sensor_mismatch", side=side))
            self._sens_prev[("ref", side)], self._sens_prev[("hard", side)] = ref, hard
        # FAULT: mikro switch önce (dış kademe), sonra referans sensör; homing kendi denetler
        if self.end_sensors == 2 and self._hard_side() and self.sup.mode not in (Mode.IDLE, Mode.HOMING, Mode.FAULT):
            self._fault("hard_limit")
        elif run and (self._ref("L") or self._ref("R")):
            self._fault("limit")

    @property
    def true_state(self) -> np.ndarray:
        return np.array(self.s)
