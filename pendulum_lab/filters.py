"""Sinyal işleme — Matlab `butter`, `kalman` karşılıkları ve gerçek zamanlı küçük filtreler.

AS5600 ve enkoder yalnızca KONUM verir; LQR'ın istediği θ̇ ve ẋ sayısal türev + alçak geçiren
filtreyle kestirilir (VelocityEstimator). Firmware aynı yöntemi kullanır.
Kesim frekansı 25 Hz: 12-bit AS5600 kuantalamasının u'daki titreşimini 40 Hz'e göre ~%27 azaltır;
15 Hz ve altında faz gecikmesi yüzünden swing-up sonrası yakalama başarısız olur (simülasyonla ölçüldü).
"""

from __future__ import annotations

import math

import numpy as np
from scipy import signal


class LowPass:
    """Birinci derece IIR alçak geçiren: y += a (x − y), a = dt / (τ + dt)."""

    def __init__(self, fc_hz: float, dt: float):
        tau = 1.0 / (2 * math.pi * fc_hz)
        self.a = dt / (tau + dt)
        self.y: float | None = None

    def update(self, x: float) -> float:
        self.y = x if self.y is None else self.y + self.a * (x - self.y)
        return self.y


class VelocityEstimator:
    """Konumdan hız: geri fark + alçak geçiren. Açı için sarma (wrap) farkını doğru alır."""

    def __init__(self, dt: float, fc_hz: float = 25.0, angular: bool = False):
        self.dt, self.angular = dt, angular
        self.lp = LowPass(fc_hz, dt)
        self.prev: float | None = None

    def update(self, pos: float) -> float:
        if self.prev is None:
            self.prev = pos
            return self.lp.update(0.0)
        d = pos - self.prev
        if self.angular:
            d = math.atan2(math.sin(d), math.cos(d))
        self.prev = pos
        return self.lp.update(d / self.dt)


def butter_lowpass(data, fc_hz: float, fs_hz: float, order: int = 2):
    """Kayıtlı veriyi faz kaymasız (filtfilt) Butterworth ile süzer — çevrimdışı analiz için."""
    b, a = signal.butter(order, fc_hz / (fs_hz / 2))
    return signal.filtfilt(b, a, data)


class KalmanFilter:
    """Ayrık lineer Kalman filtresi (opsiyonel). Model: s⁺ = Ad s + Bd u + w,  y = C s + v.

    Qn: süreç gürültüsü kovaryansı, Rn: ölçüm gürültüsü kovaryansı.
    """

    def __init__(self, Ad, Bd, C, Qn, Rn, s0=None, P0=None):
        self.Ad, self.Bd, self.C = map(np.asarray, (Ad, Bd, C))
        n = self.Ad.shape[0]
        self.Qn, self.Rn = np.atleast_2d(Qn), np.atleast_2d(Rn)
        self.s = np.zeros(n) if s0 is None else np.asarray(s0, float)
        self.P = np.eye(n) if P0 is None else np.asarray(P0, float)

    def step(self, y, u: float = 0.0) -> np.ndarray:
        # tahmin
        self.s = self.Ad @ self.s + (self.Bd * u).ravel()
        self.P = self.Ad @ self.P @ self.Ad.T + self.Qn
        # düzeltme
        S = self.C @ self.P @ self.C.T + self.Rn
        Kk = self.P @ self.C.T @ np.linalg.inv(S)
        self.s = self.s + Kk @ (np.asarray(y, float) - self.C @ self.s)
        self.P = (np.eye(len(self.s)) - Kk @ self.C) @ self.P
        return self.s
