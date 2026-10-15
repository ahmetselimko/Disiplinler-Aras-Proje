"""Dik konum (θ = 0) etrafında doğrusallaştırma — Matlab `jacobian` karşılığı.

sinθ ≈ θ, cosθ ≈ 1, θ̇² ≈ 0 varsayımıyla  ṡ = A s + B u.
Analitik sonuç, sayısal jacobian ile testte karşılaştırılır.
"""

from __future__ import annotations

import numpy as np

from .dynamics import dynamics
from .params import PendulumParams


def linearize(p: PendulumParams) -> tuple[np.ndarray, np.ndarray]:
    """Analitik (A, B). Durum sırası [x, dx, th, dth]."""
    ml, J, g = p.m * p.l, p.J, p.g

    if p.input_mode == "acceleration":
        A = np.array([[0, 1, 0, 0],
                      [0, 0, 0, 0],
                      [0, 0, 0, 1],
                      [0, 0, ml * g / J, -p.c / J]], dtype=float)
        B = np.array([[0], [1], [0], [-ml / J]], dtype=float)
        return A, B

    if p.input_mode == "voltage":
        b_eff, gain = p.b + p.motor.beta, p.motor.alpha
    else:
        b_eff, gain = p.b, 1.0

    D = (p.M_eff + p.m) * J - ml ** 2
    A = np.array([
        [0, 1, 0, 0],
        [0, -J * b_eff / D, -ml ** 2 * g / D, ml * p.c / D],
        [0, 0, 0, 1],
        [0, ml * b_eff / D, (p.M_eff + p.m) * ml * g / D, -(p.M_eff + p.m) * p.c / D],
    ], dtype=float)
    B = gain * np.array([[0], [J / D], [0], [-ml / D]], dtype=float)
    return A, B


def numerical_jacobian(f, s0, u0: float, eps: float = 1e-6) -> tuple[np.ndarray, np.ndarray]:
    """Merkezi farkla ∂f/∂s ve ∂f/∂u. f(s, u) -> ṡ. Herhangi bir model için kullanılabilir."""
    s0 = np.asarray(s0, dtype=float)
    n = s0.size
    A = np.zeros((n, n))
    for i in range(n):
        d = np.zeros(n)
        d[i] = eps
        A[:, i] = (f(s0 + d, u0) - f(s0 - d, u0)) / (2 * eps)
    B = ((f(s0, u0 + eps) - f(s0, u0 - eps)) / (2 * eps)).reshape(n, 1)
    return A, B


def numerical_linearize(p: PendulumParams, s0=(0, 0, 0, 0), u0: float = 0.0):
    """Lineer olmayan modeli herhangi bir çalışma noktasında sayısal doğrusallaştırır."""
    return numerical_jacobian(lambda s, u: dynamics(s, u, p), s0, u0)
