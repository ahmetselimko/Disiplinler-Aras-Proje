"""Araba-sarkaç sisteminin lineer OLMAYAN hareket denklemleri (Lagrange).

    (M+m) ẍ + m l cosθ θ̈ − m l sinθ θ̇² = F − b ẋ
    m l cosθ ẍ + J θ̈ − m g l sinθ        = − c θ̇          (J = I + m l²)

θ = 0 dik konumdur. Kodda M yerine M_eff kullanılır (DC motorda rotor ataleti dahil).
Bu fonksiyonlar simülasyonda (ode45 / solve_ivp) "gerçek dünya" olarak kullanılır; LQR tasarımı ise linearize.py'deki doğrusallaştırılmış modelle yapılır.
Skaler `math` kullanılır: 500 Hz × RK4 alt adımlarda NumPy'den ~10 kat hızlıdır.
"""

from __future__ import annotations

import math

import numpy as np

from .params import PendulumParams


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
    r1 = F - p.b * dx + ml * sin * dth * dth
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
    F = (p.M_eff + p.m) * a + ml * cos * ddth - ml * sin * dth * dth + p.b * dx
    if p.input_mode == "force":
        return F
    return (F + p.motor.beta * dx) / p.motor.alpha


def energy(p: PendulumParams, th: float, dth: float) -> float:
    """Sarkacın enerjisi; dik ve durağan konumda 0, aşağıda durağan iken −2mgl."""
    return 0.5 * p.J * dth * dth + p.m * p.g * p.l * (math.cos(th) - 1.0)


def wrap_angle(th: float) -> float:
    """Açıyı (−π, π] aralığına sarar."""
    return math.atan2(math.sin(th), math.cos(th))
