"""Mekatronik ekibinin Matlab/Simulink modeliyle uyumluluk.

Matlab dosyaları (`cart_pendulum_lqr.m`, `cart_pendulum_application.m`, `cart_pendulum_lqr_sim.slx`)
issue #1'deki ön karşılaştırmaya göre bu projeyle aynı denklemleri kullanır (dosyalar
repoya eklenince doğrudan doğrulanacak). İki fark vardır:

1. θ işareti: Matlab'da sarkaç SOLA yatınca θ > 0, bu projede (Python, firmware, docs/protokol.md)
   +x yönüne (SAĞA) yatınca θ > 0. Durum dönüşümü s_matlab = T·s, T = diag(1, 1, −1, −1).
   Bu yüzden K'nın θ ve θ̇ kazançlarının işareti terstir:  K_matlab = K·T.
2. Deneme biçimi (`cart_pendulum_application.m`): sürekli zamanlı kontrolcü (örnekleme yok),
   kuvvet girişi, ideal sensör, sabit adımlı RK4 (dt = 2.5 ms); araba ±0.2 m'yi aşınca veya
   |θ| ≥ 90° olunca deneme durur.

`simulate_matlab_compatible` bu deneme biçimini Python'da tekrarlar; Analiz sekmesindeki
"Matlab uyumlu" seçeneği bunu kullanır. Karşılaştırma testi: tests/test_matlab_compat.py
"""

from __future__ import annotations

import copy
import math
from dataclasses import dataclass

import numpy as np

from ..model.dynamics import accelerations
from ..model.params import PendulumParams

# s_matlab = MATLAB_SIGN @ s_proje  (θ ve θ̇'nın işareti ters). T⁻¹ = T.
MATLAB_SIGN = np.diag([1.0, 1.0, -1.0, -1.0])

# cart_pendulum_lqr.m içindeki tasarım ağırlıkları
MATLAB_Q = (25.0, 4.0, 32.83, 1.0)
MATLAB_R = 0.01
# cart_pendulum_lqr.m'nin bu değerlerle bulduğu K (Matlab'ın θ işaretinde, issue #1 ve #10)
MATLAB_K_REFERENCE = (-50.0, -45.37, 168.66, 33.25)

# cart_pendulum_application.m deneme ayarları
MATLAB_DT = 0.0025          # RK4 adımı [s]
MATLAB_RAIL_LIMIT = 0.2     # |x| bu değeri aşınca deneme durur [m]
MATLAB_THETA_LIMIT = math.pi / 2   # |θ| bu değere ulaşınca deneme durur [rad]


def matlab_reference_params() -> PendulumParams:
    """cart_pendulum_lqr.m'deki fiziksel parametreler (kuvvet girişi, pivot sürtünmesi yok)."""
    return PendulumParams(M=0.5, m=0.2, l=0.3, I=0.006, b=0.1, c=0.0, g=9.81,
                          input_mode="force", u_max=1e6, rail_half=MATLAB_RAIL_LIMIT,
                          note="Matlab cart_pendulum_lqr.m referans değerleri")


def state_to_matlab(s) -> np.ndarray:
    """Proje durumu [x, ẋ, θ, θ̇] → Matlab durumu (θ işareti ters)."""
    return MATLAB_SIGN @ np.asarray(s, dtype=float)


def state_from_matlab(s) -> np.ndarray:
    """Matlab durumu → proje durumu (dönüşüm kendi tersidir)."""
    return MATLAB_SIGN @ np.asarray(s, dtype=float)


def gain_to_matlab(K) -> np.ndarray:
    """Projede hesaplanan K → Matlab'da aynı kontrolcüyü veren K (θ ve θ̇ kazançlarının işareti ters)."""
    return np.asarray(K, dtype=float).reshape(-1) @ MATLAB_SIGN


def gain_from_matlab(K) -> np.ndarray:
    """Matlab'da hesaplanan K → bu projede (ve ESP32'de) kullanılacak K."""
    return np.asarray(K, dtype=float).reshape(-1) @ MATLAB_SIGN


def force_model(p: PendulumParams) -> PendulumParams:
    """Aynı mekanik parametrelerle kuvvet girişli model (Matlab modeli gibi motor yok, doyum yok)."""
    q = copy.deepcopy(p)
    q.input_mode = "force"
    q.u_max = 1e6
    return q


@dataclass
class MatlabSimResult:
    t: np.ndarray          # (n,)
    s: np.ndarray          # (n, 4) proje işaretinde durum [x, ẋ, θ, θ̇]
    u: np.ndarray          # (n,) kuvvet [N]
    stop_reason: str       # "time" (süre bitti), "rail" (|x| > sınır), "angle" (|θ| ≥ 90°)

    @property
    def x(self): return self.s[:, 0]

    @property
    def th(self): return self.s[:, 2]

    @property
    def mode(self) -> list:
        """Analiz sekmesinin mod grafiği için: dengede BALANCE, durduysa son nokta FAULT."""
        m = ["BALANCE"] * len(self.t)
        if self.stop_reason != "time" and m:
            m[-1] = "FAULT"
        return m


def simulate_matlab_compatible(p: PendulumParams, K, s0, T: float = 10.0, *, x_ref: float = 0.0,
                               dt: float = MATLAB_DT, rail_limit: float = MATLAB_RAIL_LIMIT,
                               theta_limit: float = MATLAB_THETA_LIMIT,
                               disturbance=None) -> MatlabSimResult:
    """`cart_pendulum_application.m` ile aynı deneme: sürekli LQR, kuvvet girişi, RK4.

    p        : mekanik parametreler (input_mode ne olursa olsun kuvvet girişiyle simüle edilir)
    K        : PROJE işaretinde LQR kazancı, u = −K·(s − [x_ref, 0, 0, 0]).
               Matlab'dan gelen K için önce gain_from_matlab(K) kullanın.
    s0       : proje işaretinde başlangıç durumu
    disturbance(t) -> ek kuvvet [N] (arabaya), isteğe bağlı

    Kontrolcü sürekli zamanlıdır: u her RK4 alt değerlendirmesinde o anki durumdan yeniden hesaplanır
    (Matlab'da u'nun türev fonksiyonunun içinde hesaplanması gibi). Sensör idealdir.
    """
    q = force_model(p)
    K = np.asarray(K, dtype=float).reshape(-1)
    if K.size != 4 or not np.all(np.isfinite(K)):
        raise ValueError("K 4 elemanlı ve sonlu olmalı")
    k0, k1, k2, k3 = (float(v) for v in K)
    n_max = int(round(T / dt))

    def control(z) -> float:
        return -(k0 * (z[0] - x_ref) + k1 * z[1] + k2 * z[2] + k3 * z[3])

    def f(z, t):
        u = control(z) + (disturbance(t) if disturbance else 0.0)
        ddx, ddth = accelerations(q, z, u)
        return (z[1], ddx, z[3], ddth)

    s = [float(v) for v in s0]
    ts, S, U = [0.0], [list(s)], [control(s)]
    reason = "time"
    for k in range(n_max):
        t = k * dt
        k1_ = f(s, t)
        k2_ = f([s[i] + 0.5 * dt * k1_[i] for i in range(4)], t + 0.5 * dt)
        k3_ = f([s[i] + 0.5 * dt * k2_[i] for i in range(4)], t + 0.5 * dt)
        k4_ = f([s[i] + dt * k3_[i] for i in range(4)], t + dt)
        s = [s[i] + dt / 6.0 * (k1_[i] + 2 * k2_[i] + 2 * k3_[i] + k4_[i]) for i in range(4)]
        if not all(math.isfinite(v) for v in s):
            raise FloatingPointError(f"Simülasyon ıraksadı (t={t:.3f})")
        ts.append(t + dt)
        S.append(list(s))
        U.append(control(s))
        if abs(s[0]) > rail_limit:
            reason = "rail"
            break
        if abs(s[2]) >= theta_limit:
            reason = "angle"
            break
    return MatlabSimResult(np.array(ts), np.array(S), np.array(U), reason)
