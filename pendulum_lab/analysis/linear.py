"""Lineer sistem analizi — Matlab `ss`, `tf`, `eig`, `ctrb`, `step`, `rlocus` karşılıkları."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy import signal

from ..control.lqr import c2d
from ..model.linearize import linearize

STATE_NAMES = ("x", "dx", "th", "dth")


@dataclass
class StateSpace:
    A: np.ndarray
    B: np.ndarray
    C: np.ndarray
    D: np.ndarray

    @classmethod
    def from_params(cls, p, outputs=("x", "th")) -> "StateSpace":
        """Doğrusallaştırılmış modeli; çıkışlar sensörlerin ölçtüğü x ve θ."""
        A, B = linearize(p)
        C = np.zeros((len(outputs), 4))
        for i, name in enumerate(outputs):
            C[i, STATE_NAMES.index(name)] = 1.0
        return cls(A, B, C, np.zeros((len(outputs), B.shape[1])))

    # --- yapısal özellikler ---
    def poles(self) -> np.ndarray:
        return np.linalg.eigvals(self.A)

    def is_stable(self) -> bool:
        return bool(np.all(self.poles().real < 0))

    def ctrb(self) -> np.ndarray:
        n = self.A.shape[0]
        return np.hstack([np.linalg.matrix_power(self.A, i) @ self.B for i in range(n)])

    def obsv(self) -> np.ndarray:
        n = self.A.shape[0]
        return np.vstack([self.C @ np.linalg.matrix_power(self.A, i) for i in range(n)])

    def is_controllable(self) -> bool:
        return np.linalg.matrix_rank(self.ctrb()) == self.A.shape[0]

    def is_observable(self) -> bool:
        return np.linalg.matrix_rank(self.obsv()) == self.A.shape[0]

    # --- dönüşümler ---
    def tf(self, output: int = 0, inp: int = 0):
        """SISO transfer fonksiyonu (pay, payda). Sayısal sıfıra çok yakın katsayılar temizlenir."""
        num, den = signal.ss2tf(self.A, self.B, self.C, self.D, input=inp)
        num = np.where(np.abs(num[output]) < 1e-9 * max(1.0, np.abs(num).max()), 0.0, num[output])
        return np.trim_zeros(num, "f") if np.any(num) else np.array([0.0]), den

    def closed_loop(self, K) -> "StateSpace":
        """u = −K(s − r·e₁) kanunuyla kapalı çevrim; yeni giriş r = araba konum referansı."""
        K = np.atleast_2d(K)
        return StateSpace(self.A - self.B @ K, self.B @ K[:, :1], self.C, np.zeros((self.C.shape[0], 1)))

    def c2d(self, Ts: float) -> "StateSpace":
        Ad, Bd = c2d(self.A, self.B, Ts)
        return StateSpace(Ad, Bd, self.C, self.D)

    # --- zaman yanıtları ---
    def step(self, T: float = 5.0, n: int = 1000, amplitude: float = 1.0, x0=None):
        """Birim basamak yanıtı. (t, y) — y şekli (n, çıkış sayısı). Kararsız sistemlerde de çalışır."""
        t = np.linspace(0, T, n)
        Ad, Bd = c2d(self.A, self.B, t[1] - t[0])
        s = np.zeros(self.A.shape[0]) if x0 is None else np.asarray(x0, float)
        u = np.full(self.B.shape[1], amplitude)
        y = np.empty((n, self.C.shape[0]))
        for k in range(n):
            y[k] = self.C @ s + self.D @ u
            s = Ad @ s + Bd @ u
        return t, y

    def closed_loop_step_discrete(self, K, Ts: float, T: float = 5.0, amplitude: float = 1.0):
        """ESP32 gibi: tesis ZOH ile Ts'de ayrıklaştırılır, u = −K(s − r·e₁) her Ts'de bir hesaplanır.
        dlqr ile tasarlanan K için doğru karşılaştırma budur. (t, y) döndürür."""
        K = np.atleast_2d(K)
        Ad, Bd = c2d(self.A, self.B, Ts)
        n = int(round(T / Ts)) + 1
        r = np.zeros(self.A.shape[0])
        r[0] = amplitude
        s = np.zeros(self.A.shape[0])
        y = np.empty((n, self.C.shape[0]))
        for k in range(n):
            y[k] = self.C @ s
            s = Ad @ s + Bd @ (-K @ (s - r))
        return np.arange(n) * Ts, y

    def initial(self, x0, T: float = 5.0, n: int = 1000):
        """Başlangıç koşulu yanıtı (girişsiz)."""
        return self.step(T, n, amplitude=0.0, x0=x0)


def rlocus(num, den, gains=None):
    """Kök eğrisi: 1 + k·G(s) = 0 köklerini her k için döndürür → (gains, roots[len(gains), n])."""
    num, den = np.atleast_1d(num).astype(float), np.atleast_1d(den).astype(float)
    if gains is None:
        gains = np.concatenate([[0], np.logspace(-3, 3, 400)])
    num_p = np.concatenate([np.zeros(len(den) - len(num)), num])
    roots = np.array([np.roots(den + k * num_p) for k in gains])
    return np.asarray(gains), roots


def step_info(t, y, settle_band: float = 0.02) -> dict:
    """Aşım, ters yönde ilk sapma (sarkaç-arabada minimum faz dışı davranış), yükselme ve oturma zamanı."""
    t, y = np.asarray(t), np.asarray(y).ravel()
    yf = y[-1]
    if abs(yf) < 1e-12:
        return {"final": yf}
    yn = y / yf
    info = {
        "final": float(yf),
        "overshoot_pct": float(max(0.0, (yn.max() - 1) * 100)),
        "undershoot_pct": float(max(0.0, -yn.min() * 100)),
    }
    try:
        t10 = t[np.argmax(yn >= 0.1)]
        t90 = t[np.argmax(yn >= 0.9)]
        info["rise_time"] = float(t90 - t10)
    except ValueError:
        info["rise_time"] = float("nan")
    outside = np.nonzero(np.abs(yn - 1) > settle_band)[0]
    info["settling_time"] = float(t[outside[-1] + 1]) if outside.size and outside[-1] + 1 < len(t) else (
        0.0 if not outside.size else float("nan"))
    return info
