"""Araba-sarkaç sisteminin lineer OLMAYAN hareket denklemleri (Lagrange).

    (M+m) ẍ + m l cosθ θ̈ − m l sinθ θ̇² = F − b ẋ − F_c sgn(ẋ)
    m l cosθ ẍ + J θ̈ − m g l sinθ        = − c θ̇          (J = I + m l²)

θ = 0 dik konumdur. Kodda M yerine M_eff kullanılır (DC motorda rotor ataleti dahil), b yerine b_cart
(DC motorda motor mili viskoz sürtünmesi B_m dahil). F_c = T_f·N/r motorun Coulomb sürtünmesidir;
doğrusallaştırmada ihmal edilir. sgn(ẋ) yerine tanh(ẋ / v_s) kullanılır: sıfır hız çevresindeki
eğim FRICTION_RATE ile sınırlanır, böylece sabit adımlı RK4 sürtünme yüzünden titremez/ıraksamaz.
accel_to_input (swing-up ve homing) F_c'yi telafi eder; firmware'deki accelToVolt da aynısını yapar.
LQR (dengede) telafi kullanmaz.
Bu fonksiyonlar simülasyonda (ode45 / solve_ivp) "gerçek dünya" olarak kullanılır; LQR tasarımı ise linearize.py'deki doğrusallaştırılmış modelle yapılır.
Skaler `math` kullanılır: 500 Hz × RK4 alt adımlarda NumPy'den ~10 kat hızlıdır.
"""

from __future__ import annotations

import math

import numpy as np

from .params import PendulumParams

# Coulomb sürtünmesinin sıfır hız çevresindeki en büyük "sönüm / araba kütlesi" oranı [1/s].
# 200 1/s: 10 ms'lik RK4 adımında bile kararlı (h·200 = 2 < 2.78); gerçekçi T_f'de v_s ~1 cm/s.
FRICTION_RATE = 200.0
# Telafide bu hızın altında araba duruyor sayılır; yön istenen ivmeden alınır (firmware: FC_V_COMP)
V_COMP = 0.01


def coulomb_friction(p: PendulumParams, dx: float) -> float:
    """Arabaya etkiyen Coulomb sürtünme kuvveti ≈ F_c·sgn(ẋ) [N] (yumuşatılmış)."""
    Fc = p.F_coulomb
    if Fc <= 0.0:
        return 0.0
    v_s = Fc / (p.M_eff * FRICTION_RATE)
    return Fc * math.tanh(dx / v_s)


def coulomb_compensation(p: PendulumParams, dx: float, a: float) -> float:
    """Coulomb sürtünmesini yenmek için eklenen kuvvet [N]: hareket yönünde F_c; araba duruyorsa
    istenen ivme a yönünde (ilk kalkışta statik sürtünmeyi aşmak için)."""
    Fc = p.F_coulomb
    d = dx if abs(dx) > V_COMP else a
    if Fc <= 0.0 or d == 0.0:
        return 0.0
    return math.copysign(Fc, d)


def cart_force(p: PendulumParams, dx: float, u: float) -> float:
    """Giriş u'yu arabaya etkiyen kuvvete çevirir (force / voltage modları)."""
    if p.input_mode == "force":
        return u
    if p.input_mode == "voltage":
        return p.motor.alpha * u - p.motor.beta * dx
    raise ValueError("acceleration modunda kuvvet tanımlı değil")


def accelerations(p: PendulumParams, s, u: float) -> tuple[float, float]:
    """(ẍ, θ̈) döndürür."""
    _, dx, th, dth = s
    sin, cos = math.sin(th), math.cos(th)
    ml = p.m * p.l
    if p.input_mode == "acceleration":
        ddx = u
        ddth = (ml * p.g * sin - p.c * dth - ml * cos * ddx) / p.J
        return ddx, ddth
    F = cart_force(p, dx, u)
    # 2x2 kütle matrisini açık formülle çöz
    a11, a12, a22 = p.M_eff + p.m, ml * cos, p.J
    r1 = F - p.b_cart * dx - coulomb_friction(p, dx) + ml * sin * dth * dth
    r2 = ml * p.g * sin - p.c * dth
    det = a11 * a22 - a12 * a12
    ddx = (a22 * r1 - a12 * r2) / det
    ddth = (a11 * r2 - a12 * r1) / det
    return ddx, ddth


def dynamics(s, u: float, p: PendulumParams) -> np.ndarray:
    """ṡ = f(s, u). s = [x, dx, th, dth]."""
    ddx, ddth = accelerations(p, s, u)
    return np.array([s[1], ddx, s[3], ddth])


def accel_to_input(p: PendulumParams, s, a: float) -> float:
    """İstenen araba ivmesi a'yı sağlayacak girişi hesaplar (geri besleme doğrusallaştırma).

    Swing-up kontrolcüsü ivme cinsinden çalışır; bu fonksiyon onu motor girişine çevirir.
    """
    if p.input_mode == "acceleration":
        return a
    _, dx, th, dth = s
    sin, cos = math.sin(th), math.cos(th)
    ml = p.m * p.l
    ddth = (ml * p.g * sin - p.c * dth - ml * cos * a) / p.J
    F = (p.M_eff + p.m) * a + ml * cos * ddth - ml * sin * dth * dth + p.b_cart * dx \
        + coulomb_compensation(p, dx, a)
    if p.input_mode == "force":
        return F
    return (F + p.motor.beta * dx) / p.motor.alpha


def energy(p: PendulumParams, th: float, dth: float) -> float:
    """Sarkacın enerjisi; dik ve durağan konumda 0, aşağıda durağan iken −2mgl."""
    return 0.5 * p.J * dth * dth + p.m * p.g * p.l * (math.cos(th) - 1.0)


def wrap_angle(th: float) -> float:
    """Açıyı (−π, π] aralığına sarar."""
    return math.atan2(math.sin(th), math.cos(th))
