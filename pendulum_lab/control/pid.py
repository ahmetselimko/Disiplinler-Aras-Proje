"""Ayrık PID — Matlab `pid(Kp, Ki, Kd)` karşılığı (LQR ile karşılaştırma / motor hız döngüsü için).

Türev ölçüm üzerinden alınır (referans basamağında türev sıçraması olmaz) ve alçak geçiren
filtrelenir; integral, çıkış doyduğunda dondurulur (anti-windup).
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class PID:
    Kp: float
    Ki: float = 0.0
    Kd: float = 0.0
    dt: float = 0.002
    u_min: float = float("-inf")
    u_max: float = float("inf")
    tau_d: float = 0.01         # türev filtresi zaman sabiti [s]
    _i: float = field(default=0.0, init=False, repr=False)
    _d: float = field(default=0.0, init=False, repr=False)
    _y_prev: float | None = field(default=None, init=False, repr=False)

    def reset(self):
        self._i, self._d, self._y_prev = 0.0, 0.0, None

    def update(self, ref: float, y: float) -> float:
        e = ref - y
        if self._y_prev is not None:
            raw_d = -(y - self._y_prev) / self.dt
            a = self.dt / (self.tau_d + self.dt)
            self._d += a * (raw_d - self._d)
        self._y_prev = y
        u = self.Kp * e + self._i + self.Kd * self._d
        u_sat = min(self.u_max, max(self.u_min, u))
        if u == u_sat:  # doyumda değilken integrali büyüt
            self._i += self.Ki * e * self.dt
        return u_sat
