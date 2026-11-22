"""Simülasyon — Matlab `ode45` karşılığı ve ESP32'yi taklit eden ayrık kontrol simülasyonu.

simulate_open_loop   : scipy solve_ivp (RK45 = ode45) ile sürekli zamanlı çözüm.
simulate_closed_loop : kontrolcü sabit Ts'de (ZOH) çalışır, tesis RK4 alt adımlarla integrallenir;
                       AS5600 12-bit kuantalama, enkoder çözünürlüğü, gürültü, gecikme ve
                       türevle hız kestirimi eklenebilir → gerçek donanıma en yakın sonuç.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from scipy.integrate import solve_ivp

from ..filters import KALMAN_ENC_RES, KALMAN_TH_RES, KalmanFilter, VelocityEstimator, encoder_resolution
from ..model.dynamics import accelerations, dynamics, wrap_angle
from ..model.params import PendulumParams


@dataclass
class SensorModel:
    """Sensör kusurları. Varsayılan: AS5600 (12 bit) + 48 CPR motor enkoderi, 10:1, HTD3M 20 diş (~0.125 mm/sayım).
    Parametrelerden (gerçek redüktör ve kasnakla, firmware gibi) kurmak için SensorModel.for_params(p)."""
    th_resolution: float = KALMAN_TH_RES
    x_resolution: float = KALMAN_ENC_RES
    th_noise_std: float = 0.0
    x_noise_std: float = 0.0
    seed: int | None = 0
    _rng: np.random.Generator = field(init=False, repr=False)

    def __post_init__(self):
        self._rng = np.random.default_rng(self.seed)

    @classmethod
    def for_params(cls, p: PendulumParams, **kw) -> "SensorModel":
        """Enkoder çözünürlüğü p'nin redüktör ve kasnağından (firmware'deki X_PER_COUNT ile aynı)."""
        kw.setdefault("x_resolution", encoder_resolution(p))
        return cls(**kw)

    @staticmethod
    def _q(v: float, res: float) -> float:
        return round(v / res) * res if res > 0 else v

    def measure(self, s) -> tuple[float, float]:
        x = s[0] + (self._rng.normal(0, self.x_noise_std) if self.x_noise_std else 0.0)
        th = wrap_angle(s[2]) + (self._rng.normal(0, self.th_noise_std) if self.th_noise_std else 0.0)
        return self._q(x, self.x_resolution), self._q(th, self.th_resolution)


def rk4_step(p: PendulumParams, s, u: float, h: float, f_cart: float = 0.0, f_pole: float = 0.0):
    """Tek RK4 adımı (skaler, hızlı). f_cart / f_pole: dış itme kuvvetleri [N] (dynamics.accelerations)."""
    def f(z):
        ddx, ddth = accelerations(p, z, u, f_cart, f_pole)
        return (z[1], ddx, z[3], ddth)
    k1 = f(s)
    k2 = f([s[i] + 0.5 * h * k1[i] for i in range(4)])
    k3 = f([s[i] + 0.5 * h * k2[i] for i in range(4)])
    k4 = f([s[i] + h * k3[i] for i in range(4)])
    return [s[i] + h / 6.0 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) for i in range(4)]


@dataclass
class SimResult:
    t: np.ndarray          # (n+1,)
    s: np.ndarray          # (n+1, 4) gerçek durum
    u: np.ndarray          # (n+1,) uygulanan giriş (son eleman tekrar)
    mode: list             # (n+1,) mod adları

    @property
    def x(self): return self.s[:, 0]

    @property
    def th(self): return self.s[:, 2]


def simulate_open_loop(p: PendulumParams, s0, T: float, u_func=None, max_step: float = 0.005, **kw):
    """ode45 benzeri: u_func(t, s) -> u (verilmezse 0). scipy OdeResult döner (t, y)."""
    u_func = u_func or (lambda t, s: 0.0)
    return solve_ivp(lambda t, s: dynamics(s, u_func(t, s), p), (0, T), np.asarray(s0, float),
                     method="RK45", max_step=max_step, rtol=1e-8, atol=1e-10, **kw)


def simulate_closed_loop(p: PendulumParams, controller, s0, T: float, dt: float = 0.002,
                         substeps: int = 4, sensor: SensorModel | None = None,
                         delay_steps: int = 0, vel_fc_hz: float = 25.0,
                         disturbance=None, end_stops: bool = True,
                         use_kalman: bool = True, disturbance_target: str = "pole",
                         kalman_Rn=None) -> SimResult:
    """controller.update(s_hat) -> (u, mode)  veya  controller(s_hat) -> u.

    sensor=None ise kontrolcü gerçek durumu görür (ideal). disturbance(t) -> ek yatay kuvvet [N]
    (dürtme testi); input_mode'dan bağımsız olarak kuvvet eklenir. disturbance_target: "pole" ise
    sarkacın ağırlık merkezine, "cart" ise arabaya (Matlab'daki "Apply push" gibi) uygulanır.
    end_stops: araba ±rail_half'teki mekanik uçlarda durur (hız sıfırlanır), rayı aşamaz.
    use_kalman: True ise (firmware gibi) dik konum bölgesinde (|θ| < release) hız Kalman filtresiyle
    kestirilir; geniş açıda vel_fc_hz'lik türev filtresi kullanılır. Eski davranış için False verin.
    """
    if disturbance_target not in ("pole", "cart"):
        raise ValueError(f"disturbance_target 'pole' veya 'cart' olmalı, verilen {disturbance_target!r}")
    n = int(round(T / dt))
    t = np.arange(n + 1) * dt
    S = np.empty((n + 1, 4))
    U = np.zeros(n + 1)
    modes: list = [None] * (n + 1)
    s = [float(v) for v in s0]
    vx = VelocityEstimator(dt, vel_fc_hz)
    vth = VelocityEstimator(dt, vel_fc_hz, angular=True)
    ctrl_p = getattr(controller, "p", p)
    # kalman_Rn: Kalman ölçüm gürültüsü [x, θ] (None: firmware gibi ENC_CPR_MOTOR ve 12-bit AS5600'dan)
    kf = KalmanFilter.from_params(ctrl_p, dt, Rn=kalman_Rn) if (use_kalman and sensor is not None) else None
    in_kf = False
    u_prev = 0.0
    pending = [0.0] * delay_steps
    call = controller.update if hasattr(controller, "update") else (lambda z: (controller(z), "CTRL"))
    h = dt / substeps

    for k in range(n):
        S[k] = s
        if sensor is None:
            s_hat = s
        else:
            xm, thm = sensor.measure(s)
            if k == 0:                       # başlangıç hızı ilk adımda kaybolmasın (kayıttan oynatma)
                vx.prime(xm, float(s0[1]))
                vth.prime(thm, float(s0[3]))
            dx_lp = vx.update(xm)
            dth_lp = vth.update(thm)
            if kf is not None:
                cur_mode = getattr(controller, "mode", "BALANCE")
                mode_name = getattr(cur_mode, "value", cur_mode)
                release = getattr(controller, "release", math.radians(30.0))
                if mode_name in ("SWING_UP", "WAIT", "BALANCE", "CTRL") and abs(wrap_angle(thm)) < release:
                    if not in_kf:
                        kf.reset([xm, dx_lp, wrap_angle(thm), dth_lp])
                        in_kf = True
                    else:
                        kf.step([xm, thm], u_prev)
                    s_hat = [xm, float(kf.s[1]), thm, float(kf.s[3])]
                else:
                    in_kf = False
                    s_hat = [xm, dx_lp, thm, dth_lp]
            else:
                s_hat = [xm, dx_lp, thm, dth_lp]
        u, mode = call(s_hat)
        modes[k] = getattr(mode, "value", mode)
        if delay_steps:
            pending.append(u)
            u = pending.pop(0)
        U[k] = u
        u_prev = 0.0 if modes[k] in ("IDLE", "FAULT") else u
        d = disturbance(t[k]) if disturbance else 0.0
        f_cart, f_pole = (d, 0.0) if disturbance_target == "cart" else (0.0, d)
        # firmware IDLE/FAULT'ta sürücüyü kapatır → araba serbest (0 V gibi frenlemez)
        coast = modes[k] in ("IDLE", "FAULT") and p.input_mode == "voltage"
        for _ in range(substeps):
            u_phys = p.motor.beta * s[1] / p.motor.alpha if coast else u
            s = rk4_step(p, s, u_phys, h, f_cart, f_pole)
            if end_stops and abs(s[0]) > p.rail_half:
                s[0], s[1] = math.copysign(p.rail_half, s[0]), 0.0
        if not all(math.isfinite(v) for v in s):
            raise FloatingPointError(f"Simülasyon ıraksadı (t={t[k]:.3f})")
    S[n], U[n], modes[n] = s, U[n - 1] if n else 0.0, modes[n - 1] if n else None
    return SimResult(t, S, U, modes)
