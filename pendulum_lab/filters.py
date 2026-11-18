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
from scipy.linalg import solve_discrete_are


KALMAN_QN = (1e-9, 1e-5, 1e-9, 1e-4)       # [x, dx, th, dth] süreç gürültüsü kovaryans köşegeni
KALMAN_TH_RES = 2 * math.pi / 4096         # 12-bit AS5600 açı çözünürlüğü [rad]
ENC_CPR_MOTOR = 48.0                       # motor milindeki enkoder (Pololu 25D: 48 CPR, 4x); firmware'deki ENC_CPR_MOTOR ile aynı
# motor parametresi olmayan modeller için varsayılan enkoder çözünürlüğü [m/sayım]: 48 CPR, 10:1, GT2 40 diş
KALMAN_ENC_RES = 2 * math.pi * 0.0127 / (ENC_CPR_MOTOR * 10.0)


def encoder_resolution(p) -> float:
    """Araba konumunun bir enkoder sayımı [m]. Firmware'deki X_PER_COUNT ile aynı formül:
    2π·r_kasnak / (ENC_CPR_MOTOR · redüktör). DC motorlu olmayan modelde varsayılan değer."""
    if getattr(p, "input_mode", "") == "voltage":
        return 2 * math.pi * p.motor.r_pulley / (ENC_CPR_MOTOR * p.motor.gear)
    return KALMAN_ENC_RES


def _wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _kalman_noise_matrices(p, Qn, Rn) -> tuple[np.ndarray, np.ndarray]:
    """Süreç (Qn) ve ölçüm (Rn) kovaryans matrislerini hazırlar."""
    Qn_mat = np.diag(Qn) if np.ndim(Qn) == 1 else np.atleast_2d(np.asarray(Qn, float))
    if Rn is None:
        x_res = encoder_resolution(p)
        Rn_mat = np.diag([(x_res ** 2) / 12.0, (KALMAN_TH_RES ** 2) / 12.0])
    else:
        Rn_mat = np.diag(Rn) if np.ndim(Rn) == 1 else np.atleast_2d(np.asarray(Rn, float))
    return Qn_mat, Rn_mat


def design_kalman(p, dt: float = 0.002, Qn=KALMAN_QN, Rn=None) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Dik konum etrafındaki ayrık model için kararlı hal Kalman kazancını hesaplar.

    Dönüş: (Ad, Bd, L, P_post). L: 4x2 kararlı hal Kalman kazancı.
    """
    from .control.lqr import c2d
    from .model.linearize import linearize

    Ad, Bd = c2d(*linearize(p), dt)
    C = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]])
    Qn_mat, Rn_mat = _kalman_noise_matrices(p, Qn, Rn)
    P_prior = solve_discrete_are(Ad.T, C.T, Qn_mat, Rn_mat)
    S = C @ P_prior @ C.T + Rn_mat
    L = P_prior @ C.T @ np.linalg.inv(S)
    P_post = (np.eye(4) - L @ C) @ P_prior
    return Ad, Bd, L, P_post


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

    def prime(self, pos: float, vel: float) -> None:
        """Kestiriciyi bilinen bir hızla başlatır: bir sonraki update(pos) tam olarak vel döndürür.

        Kayıttan başlatılan simülasyonda başlangıç hızı ilk adımda sıfırlanmasın diye kullanılır;
        vel = 0 iken davranış başlatılmamış kestiriciyle aynıdır.
        """
        self.prev = pos - vel * self.dt
        self.lp.y = float(vel)

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
    """Ayrık lineer Kalman filtresi. Model: s⁺ = Ad s + Bd u + w,  y = C s + v.

    Qn: süreç gürültüsü kovaryansı, Rn: ölçüm gürültüsü kovaryansı.
    """

    def __init__(self, Ad, Bd, C, Qn, Rn, s0=None, P0=None, angular_index: int | None = 2):
        self.Ad, self.Bd, self.C = map(np.asarray, (Ad, Bd, C))
        n = self.Ad.shape[0]
        self.Qn, self.Rn = np.atleast_2d(Qn), np.atleast_2d(Rn)
        self.s = np.zeros(n) if s0 is None else np.asarray(s0, float)
        self.P = np.eye(n) if P0 is None else np.asarray(P0, float)
        self.angular_index = angular_index
        self._P0 = self.P.copy()
        self.L: np.ndarray | None = None

    @classmethod
    def from_params(cls, p, dt: float = 0.002, Qn=KALMAN_QN, Rn=None, s0=None) -> "KalmanFilter":
        """Fiziksel parametrelerden ayrık lineer Kalman filtresi kurar."""
        Ad, Bd, L, P_post = design_kalman(p, dt, Qn=Qn, Rn=Rn)
        C = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]])
        Qn_mat, Rn_mat = _kalman_noise_matrices(p, Qn, Rn)
        kf = cls(Ad, Bd, C, Qn_mat, Rn_mat, s0=s0, P0=P_post)
        kf.L = L
        return kf

    def reset(self, s0=None):
        """Filtre durumunu ve kovaryansını başlangıca döndürür."""
        self.s = np.zeros(len(self.s)) if s0 is None else np.asarray(s0, float).copy()
        if self.angular_index is not None and self.angular_index < len(self.s):
            self.s[self.angular_index] = _wrap_angle(float(self.s[self.angular_index]))
        self.P = self._P0.copy()

    def step(self, y, u: float = 0.0) -> np.ndarray:
        # tahmin
        self.s = self.Ad @ self.s + (self.Bd * u).ravel()
        if self.angular_index is not None and self.angular_index < len(self.s):
            self.s[self.angular_index] = _wrap_angle(float(self.s[self.angular_index]))
        self.P = self.Ad @ self.P @ self.Ad.T + self.Qn
        # düzeltme
        S = self.C @ self.P @ self.C.T + self.Rn
        Kk = self.P @ self.C.T @ np.linalg.inv(S)
        inn = np.asarray(y, float) - self.C @ self.s
        if self.angular_index == 2 and inn.size >= 2:
            inn[1] = _wrap_angle(float(inn[1]))
        self.s = self.s + Kk @ inn
        if self.angular_index is not None and self.angular_index < len(self.s):
            self.s[self.angular_index] = _wrap_angle(float(self.s[self.angular_index]))
        self.P = (np.eye(len(self.s)) - Kk @ self.C) @ self.P
        return self.s
