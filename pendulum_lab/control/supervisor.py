"""Mod yöneticisi: swing-up ↔ LQR geçişi ve güvenlik.

Toplantı kararı: LQR, sinθ≈θ varsayımı geçerli olduğu bölgede (|θ| < ~10–15°) açık; dışarıda
enerji bazlı swing-up çalışır. Geçiş yazılımsaldır ve histerezislidir (eşikte titreme olmasın):
    SWING_UP → BALANCE :  |θ| < catch_deg  ve |θ̇| < catch_dth
    BALANCE  → SWING_UP:  |θ| > release_deg
    her mod  → FAULT   :  |x| > rail_half − soft_margin   (motor gücü kesilir)
    FAULT → IDLE yalnız STOP ile; BAŞLAT yalnız IDLE'dan kabul edilir (hatadan doğrudan çıkılmaz).

ESP32 firmware'indeki mantık (firmware/esp32_pendulum) bu sınıfla BİREBİR aynı tutulmalıdır.
"""

from __future__ import annotations

import math
from enum import Enum

import numpy as np

from ..model.dynamics import accel_to_input, wrap_angle
from ..model.params import PendulumParams
from .swingup import EnergySwingUp


class Mode(str, Enum):
    IDLE = "IDLE"          # motor kapalı
    HOMING = "HOMING"      # limit switch'e gidip sıfır noktası bulunuyor (firmware/sanal cihaz)
    WAIT = "WAIT"          # swing-up kapalı; sarkacın elle dik konuma getirilmesi bekleniyor
    SWING_UP = "SWING_UP"
    BALANCE = "BALANCE"    # LQR
    FAULT = "FAULT"        # ray sınırı / acil durum — motor kapalı


MODE_CODES = {m: i for i, m in enumerate(Mode)}
CODE_MODES = {i: m for m, i in MODE_CODES.items()}


class Supervisor:
    def __init__(self, p: PendulumParams, K, swingup: EnergySwingUp | None = None,
                 catch_deg: float = 15.0, release_deg: float = 30.0, catch_dth: float = 5.0,
                 soft_margin: float = 0.03, swingup_enabled: bool = True):
        self.p = p
        self.K = np.asarray(K, dtype=float).reshape(-1)
        self.swingup = swingup or EnergySwingUp()
        self.catch = math.radians(catch_deg)
        self.release = math.radians(release_deg)
        self.catch_dth = catch_dth
        self.soft_margin = soft_margin
        self.swingup_enabled = swingup_enabled
        self.x_ref = 0.0
        self.mode = Mode.IDLE

    # --- komutlar ---
    def start(self) -> tuple[bool, str]:
        """Yalnız IDLE'dan başlar. (kabul edildi mi, sebep) döndürür."""
        if self.mode == Mode.FAULT:
            return False, "hata modunda: önce stop"
        if self.mode != Mode.IDLE:
            return False, f"{self.mode.value} modunda başlatılamaz"
        self.mode = Mode.SWING_UP if self.swingup_enabled else Mode.WAIT
        return True, ""

    def stop(self):
        self.mode = Mode.IDLE

    def set_gain(self, K):
        K = np.asarray(K, dtype=float).reshape(-1)
        if K.size != 4 or not np.all(np.isfinite(K)):
            raise ValueError("K 4 elemanlı ve sonlu olmalı")
        self.K = K

    # --- döngü ---
    def lqr_input(self, s) -> float:
        e0 = s[0] - self.x_ref
        e2 = wrap_angle(s[2])
        k = self.K
        return -(k[0] * e0 + k[1] * s[1] + k[2] * e2 + k[3] * s[3])

    def update(self, s) -> tuple[float, Mode]:
        """Bir kontrol adımı. s = [x, dx, th, dth] (ölçülen/kestirilen). (u, mod) döndürür."""
        p = self.p
        if self.mode in (Mode.IDLE, Mode.HOMING, Mode.FAULT):
            return 0.0, self.mode
        if abs(s[0]) > p.rail_half - self.soft_margin:
            self.mode = Mode.FAULT
            return 0.0, self.mode

        th = abs(wrap_angle(s[2]))
        if self.mode == Mode.BALANCE and th > self.release:
            self.mode = Mode.SWING_UP if self.swingup_enabled else Mode.WAIT
        elif self.mode in (Mode.SWING_UP, Mode.WAIT) and th < self.catch and abs(s[3]) < self.catch_dth:
            self.mode = Mode.BALANCE

        if self.mode == Mode.BALANCE:
            u = self.lqr_input(s)
        elif self.mode == Mode.SWING_UP:
            u = accel_to_input(p, s, self.swingup.accel(p, s))
        else:
            u = 0.0
        return max(-p.u_max, min(p.u_max, u)), self.mode
