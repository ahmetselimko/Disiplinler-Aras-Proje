"""Sanal ESP32: firmware'in yapacağı her şeyi gerçek zamanlı simüle eder.

- 500 Hz kontrol döngüsü (Supervisor: swing-up + LQR), sensör kuantalama, türevle hız kestirimi
- homing: sol limit switch'e yavaşça gider, sıfırı ayarlar, rayın ortasına döner (süre sınırlı;
  yanlış taraftaki switch'e çarparsa FAULT)
- fiziksel ray sonları (limit switch olayı + FAULT)
- protokoldeki tüm komutlar, firmware ile AYNI kabul/ret kurallarıyla; ek olarak yalnız
  simülasyonda "kick" (sarkacı dürt)

Arayüz ve okuyucu, gerçek kart gelmeden bu cihazla uçtan uca geliştirilir/test edilir.
"""

from __future__ import annotations

import math
import random
import threading
import time

import numpy as np

from ..control.lqr import design_lqr
from ..control.supervisor import Mode, Supervisor
from ..filters import VelocityEstimator
from ..model.dynamics import accel_to_input
from ..model.params import PendulumParams
from ..sim.simulator import SensorModel, rk4_step
from . import protocol as P

DEFAULT_Q = (10.0, 1.0, 100.0, 1.0)
DEFAULT_R = 0.1
HOME_SPEED = 0.10        # [m/s]
HOME_TIMEOUT = 20.0      # [s]
FW_NAME = "mock-1.1"


class MockDevice:
    def __init__(self, params: PendulumParams | None = None, Ts: float = 0.002, telemetry_hz: int = 200,
                 seed: int | None = None, realtime: bool = True, x0: float | None = None):
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
        self.sensor = SensorModel(seed=seed)
        self.vx = VelocityEstimator(Ts)
        self.vth = VelocityEstimator(Ts, angular=True)
        self.k = 0
        self.n = 0
        self._tel_acc = 0.0
        self._home_phase = 0
        self._home_t0 = 0.0
        self._home_r0 = False
        self._rx = bytearray()
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._running = False
        self._at_limit = False

    # ------------------------------------------------------------ yaşam döngüsü
    def start(self):
        self._running = True
        self._emit(P.encode_event("boot", fw=FW_NAME, Ts=self.Ts))
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
            try:
                cmd, args = P.parse_command(line)
                self._emit(self._handle(cmd, args))
            except (P.ProtocolError, ValueError) as e:
                self._emit(P.encode_ack("?", False, str(e)))

    def _handle(self, cmd: str, a: dict) -> bytes:
        sup = self.sup
        mode = sup.mode
        if cmd == "ping":
            return P.encode_ack("ping", True, "pong", Ts=self.Ts, fw=FW_NAME, homed=self.homed)
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
        if cmd == "home":
            if mode not in (Mode.IDLE, Mode.FAULT):
                return P.encode_ack("home", False, "önce stop")
            sup.mode, self._home_phase, self._home_t0 = Mode.HOMING, 1, self.k * self.Ts
            self._home_r0 = self._switch("R")
            return P.encode_ack("home")
        if cmd == "calib":
            if mode != Mode.IDLE:
                return P.encode_ack("calib", False, "yalnız IDLE modunda, sarkaç aşağıda ve hareketsizken")
            return P.encode_ack("calib")      # sanal sensör zaten doğru kalibre
        if cmd == "start":
            if not self.homed:
                return P.encode_ack("start", False, "önce home (kalibrasyon) yapılmalı")
            ok, msg = sup.start()
            return P.encode_ack("start", ok, msg)
        if cmd == "stop":
            sup.stop()
            self._home_phase = 0
            return P.encode_ack("stop")
        if cmd == "estop":
            self._fault("estop")
            return P.encode_ack("estop")
        if cmd == "kick":
            self.s[3] += float(a.get("dth", 1.5))
            return P.encode_ack("kick")
        return P.encode_ack(cmd, False, "desteklenmiyor")

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
        if "x_ref" in a and abs(float(a["x_ref"])) > self.p.rail_half - 0.1:
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
        elif key == "swingup":
            sup.swingup_enabled = bool(val)
        elif key in ("k_E", "a_max", "k_x", "k_v"):
            setattr(su, key, val)
        elif key == "x_ref":
            sup.x_ref = val
        elif key == "u_max":
            self.p.u_max = val
        elif key == "telemetry_hz":
            self.telemetry_hz = int(val)

    # ------------------------------------------------------------ 500 Hz döngü
    def _switch(self, side: str) -> bool:
        lim = self.p.rail_half
        return self.s[0] <= -lim + 1e-9 if side == "L" else self.s[0] >= lim - 1e-9

    def _homing_input(self, s_hat) -> float:
        p = self.p
        if self.k * self.Ts - self._home_t0 > HOME_TIMEOUT:
            self._fault("homing_timeout")
            return 0.0
        if not self._switch("R"):
            self._home_r0 = False
        elif not self._home_r0 or self.k * self.Ts - self._home_t0 > 1.0:
            # yanlış taraf (motor/enkoder yönü ters). Başta sağ switch basılıysa (ör. sağ limit hatasından
            # sonra) 1 s içinde sola çekilip bırakması beklenir.
            self._fault("homing_wrong_side")
            return 0.0
        if self._home_phase == 1:
            if self._switch("L"):             # switch SEVİYESİ (zaten basılıysa da çalışır)
                self.x_offset = self.s[0] + p.rail_half   # ölçülen konum artık −rail_half
                self._home_phase = 2
                self.vx.prev = None
                return 0.0
            v_des = -HOME_SPEED
        else:                                 # ortaya dön
            v_des = max(-0.15, min(0.15, -2.0 * s_hat[0]))
            if abs(s_hat[0]) < 0.002 and abs(s_hat[1]) < 0.02:
                self.sup.mode = Mode.IDLE
                self._home_phase = 0
                self.homed = True
                self._emit(P.encode_event("homed"))
                return 0.0
        a = 20.0 * (v_des - s_hat[1])
        return max(-p.u_max, min(p.u_max, accel_to_input(p, s_hat, a)))

    def step(self):
        self._process_commands()
        p, sup = self.p, self.sup
        xm, thm = self.sensor.measure([self.s[0] - self.x_offset, 0, self.s[2], 0])
        s_hat = [xm, self.vx.update(xm), thm, self.vth.update(thm)]

        if sup.mode == Mode.HOMING:
            u = self._homing_input(s_hat)
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
        u_phys = u
        if mode in (Mode.IDLE, Mode.FAULT) and p.input_mode == "voltage":
            # firmware sürücüyü (EN) kapatır → motor fren yapmaz, araba serbest; zıt EMK'yı sıfırlayan gerilim
            u_phys = p.motor.beta * self.s[1] / p.motor.alpha

        h = self.Ts / 2
        for _ in range(2):
            self.s = rk4_step(p, self.s, u_phys, h)
        self._rail_end_stops()

        self.k += 1
        self._tel_acc += self.telemetry_hz * self.Ts      # 200 Hz / 500 Hz = her 2.5 adımda bir
        if self._tel_acc >= 1.0:
            self._tel_acc -= 1.0
            self._emit(P.encode_telemetry(self.n, self.k * self.Ts, xm, s_hat[1], thm, s_hat[3], u, mode.value))
            self.n += 1

    def _rail_end_stops(self):
        """Fiziksel ray sonu: araba durur, limit switch basılır (olay yalnız ilk temasta)."""
        lim = self.p.rail_half
        if abs(self.s[0]) < lim:
            self._at_limit = False
            return
        side = "L" if self.s[0] < 0 else "R"
        self.s[0] = math.copysign(lim, self.s[0])
        self.s[1] = 0.0
        if self._at_limit:
            return
        self._at_limit = True
        self._emit(P.encode_event("limit", side=side))
        if self.sup.mode not in (Mode.IDLE, Mode.HOMING, Mode.FAULT):
            self._fault("limit")

    @property
    def true_state(self) -> np.ndarray:
        return np.array(self.s)
