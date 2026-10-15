"""LQR tasarımı — Matlab `lqr`, `dlqr`, `c2d` karşılıkları.

Sürekli:  A'P + PA − PBR⁻¹B'P + Q = 0  (CARE),  K = R⁻¹B'P
Ayrık:    P = A'PA − A'PB(R+B'PB)⁻¹B'PA + Q (DARE), K = (R+B'PB)⁻¹B'PA

ESP32 kontrol döngüsü sabit Ts ile çalıştığı için gönderilecek K, `dlqr(c2d(A,B,Ts))` ile
hesaplanmalıdır; Ts küçükken (2 ms) sürekli K ile neredeyse aynıdır.
Kontrol kanunu her iki durumda:  u = −K (s − s_ref)
"""

from __future__ import annotations

import numpy as np
from scipy.linalg import expm, solve_continuous_are, solve_discrete_are


def _as_matrices(Q, R, n, m):
    Q = np.atleast_2d(np.asarray(Q, dtype=float))
    if Q.shape == (1, n):
        Q = np.diag(Q.ravel())
    R = np.atleast_2d(np.asarray(R, dtype=float))
    if Q.shape != (n, n) or R.shape != (m, m):
        raise ValueError(f"Q {n}x{n} (veya {n} elemanlı köşegen), R {m}x{m} olmalı")
    return Q, R


def lqr(A, B, Q, R):
    """(K, P, kapalı çevrim özdeğerleri). Q köşegen vektör olarak da verilebilir."""
    A, B = np.asarray(A, float), np.asarray(B, float)
    Q, R = _as_matrices(Q, R, A.shape[0], B.shape[1])
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.solve(R, B.T @ P)
    return K, P, np.linalg.eigvals(A - B @ K)


def c2d(A, B, Ts: float):
    """Sıfırıncı derece tutucu (ZOH) ile ayrıklaştırma: Ad = e^{A Ts}, Bd = ∫ e^{Aτ} dτ B."""
    A, B = np.asarray(A, float), np.asarray(B, float)
    n, m = B.shape
    M = np.zeros((n + m, n + m))
    M[:n, :n], M[:n, n:] = A, B
    E = expm(M * Ts)
    return E[:n, :n], E[:n, n:]


def dlqr(Ad, Bd, Q, R):
    """Ayrık LQR: (K, P, kapalı çevrim özdeğerleri — birim çember içinde olmalı)."""
    Ad, Bd = np.asarray(Ad, float), np.asarray(Bd, float)
    Q, R = _as_matrices(Q, R, Ad.shape[0], Bd.shape[1])
    P = solve_discrete_are(Ad, Bd, Q, R)
    K = np.linalg.solve(R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)
    return K, P, np.linalg.eigvals(Ad - Bd @ K)


def design_lqr(p, Q, R, Ts: float | None = None):
    """Parametrelerden doğrudan K. Ts verilirse ayrık (ESP32 için), verilmezse sürekli."""
    from ..model.linearize import linearize
    A, B = linearize(p)
    if Ts is None:
        return lqr(A, B, Q, R)
    return dlqr(*c2d(A, B, Ts), Q, R)
