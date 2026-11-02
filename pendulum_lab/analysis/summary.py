"""Deneme metrikleri: toplu hesap ve sabit bellekli akış hesabı; açılar radyan."""
from __future__ import annotations

import math
import threading
from dataclasses import dataclass, replace

import numpy as np


def _arrays(t, *values):
    arrays = [np.asarray(a, dtype=float) for a in (t, *values)]
    if any(a.ndim != 1 or len(a) != len(arrays[0]) or not np.all(np.isfinite(a)) for a in arrays):
        raise ValueError("Veriler eşit uzunlukta, sonlu, tek boyutlu diziler olmalı")
    if np.any(np.diff(arrays[0]) <= 0):
        raise ValueError("Zaman kesin artan olmalı")
    return arrays


def find_settling_time(t, th, threshold_deg=2.0, min_duration=1.0, max_gap=None):
    """Banda ilk giriş; min_duration boyunca kesintisiz gözlemle doğrulanır.

    Bu ilk dengelemedir; sarkacın bir daha düşmeyeceği anlamına gelmez.
    max_gap, eksik telemetride sürekliliği keser.
    """
    t, th = _arrays(t, th)
    acc = TrialAccumulator(threshold_deg=threshold_deg, min_duration=min_duration, max_gap=max_gap)
    for ti, angle in zip(t, th):
        acc.add(ti, 0.0, angle, 0.0)
    return acc.summary().settling_time


def input_saturation_time(t, u, u_max, tol=1e-3):
    """Soldan tutulan giriş integrali: u[i], [t[i], t[i+1]) aralığında geçerlidir."""
    t, u = _arrays(t, u)
    if not math.isfinite(u_max) or u_max <= 0 or not 0 <= tol < u_max:
        raise ValueError("u_max pozitif, 0 <= tol < u_max olmalı")
    sat = float(np.sum(np.diff(t)[np.abs(u[:-1]) >= u_max - tol]))
    duration = float(t[-1] - t[0]) if len(t) else 0.0
    return sat, 100 * sat / duration if duration else 0.0


@dataclass
class TrialSummary:
    settling_time: float | None = None
    max_x: float = 0.0
    max_u: float = 0.0
    sat_time: float = 0.0
    sat_pct: float = 0.0
    final_th_deg: float = 0.0
    final_mode: str | None = None
    first_lqr_time: float | None = None
    threshold_deg: float = 2.0
    min_duration: float = 1.0
    observed_time: float = 0.0
    missing_time: float = 0.0
    samples: int = 0
    end_reason: str | None = None

    def format_lines(self, rail_half=0.4, u_max=12.0):
        lines = []
        if self.final_mode is not None:
            lines.append(f"Son mod: {self.final_mode}")
        if self.end_reason and self.end_reason != self.final_mode:     # ör. "Son mod: IDLE" + "Bitiş: IDLE" olmasın
            lines.append(f"Bitiş: {self.end_reason}")
        if self.first_lqr_time is not None:
            lines.append(f"LQR ilk devreye giriş: t = {self.first_lqr_time:.2f} s")
        rule = f"|θ|≤{self.threshold_deg:g}°, {self.min_duration:g} s"
        lines.append(f"İlk dengeleme zamanı ({rule}): " +
                     (f"t = {self.settling_time:.2f} s" if self.settling_time is not None else "Henüz sağlanmadı"))
        lines.extend([
            f"Maksimum araba hareketi: max |x| = {self.max_x:.3f} m (ray ±{rail_half:.2f} m)",
            f"Maksimum giriş: max |u| = {self.max_u:.2f} / {u_max:g}",
            f"Giriş sınırında geçen süre: {self.sat_time:.3f} s (%{self.sat_pct:.1f})",
            f"Son açı: |θ| = {abs(self.final_th_deg):.2f}°",
        ])
        if self.missing_time:
            lines.append(f"Hariç süre (veri boşluğu/duraklatma): {self.missing_time:.3f} s")
        return lines

    def to_dict(self):
        return dict(settling_time_s=self.settling_time, max_x_m=self.max_x, max_u=self.max_u,
                    saturation_time_s=self.sat_time, saturation_pct=self.sat_pct,
                    final_th_deg=self.final_th_deg, final_mode=self.final_mode,
                    first_lqr_time_s=self.first_lqr_time, threshold_deg=self.threshold_deg,
                    min_duration_s=self.min_duration, observed_time_s=self.observed_time,
                    missing_time_s=self.missing_time, samples=self.samples, end_reason=self.end_reason)


class TrialAccumulator:
    """O(1) bellek, örnek başına O(1) iş. Giriş bir sonraki örneğe kadar sabit tutulur."""
    def __init__(self, u_max=12.0, threshold_deg=2.0, min_duration=1.0, max_gap=None):
        if not math.isfinite(u_max) or u_max <= 0:
            raise ValueError("u_max pozitif olmalı")
        if not math.isfinite(threshold_deg) or threshold_deg <= 0 or not math.isfinite(min_duration) or min_duration <= 0:
            raise ValueError("Denge eşiği ve süresi pozitif olmalı")
        if max_gap is not None and (not math.isfinite(max_gap) or max_gap <= 0):
            raise ValueError("max_gap pozitif olmalı")
        self.u_max, self.max_gap = u_max, max_gap
        self._s = TrialSummary(threshold_deg=threshold_deg, min_duration=min_duration)
        self._last_t = self._last_u = self._band_start = None
        self._last_limit = u_max

    def add(self, t, x, th, u, mode=None, continuous=True):
        if not all(math.isfinite(v) for v in (t, x, th, u)):
            raise ValueError("Telemetri sonlu olmalı")
        s = self._s
        if self._last_t is not None:
            dt = t - self._last_t
            if dt <= 0:
                raise ValueError("Zaman kesin artan olmalı")
            if not continuous or (self.max_gap is not None and dt > self.max_gap):
                s.missing_time += dt
                self._band_start = None
            else:
                s.observed_time += dt
                if abs(self._last_u) >= self._last_limit - min(1e-3, self._last_limit * 1e-3):
                    s.sat_time += dt
        angle = math.degrees(math.atan2(math.sin(th), math.cos(th)))
        if abs(angle) <= s.threshold_deg:
            if self._band_start is None:
                self._band_start = t
            if s.settling_time is None and t - self._band_start >= s.min_duration - 1e-12:
                s.settling_time = self._band_start
        else:
            self._band_start = None
        s.max_x, s.max_u = max(s.max_x, abs(x)), max(s.max_u, abs(u))
        s.final_th_deg, s.final_mode = angle, getattr(mode, "value", mode)
        if s.first_lqr_time is None and s.final_mode in ("BALANCE", "LQR"):
            s.first_lqr_time = t
        s.samples += 1
        self._last_t, self._last_u = t, u
        self._last_limit = self.u_max

    def break_continuity(self):
        self._band_start = None

    def summary(self):
        s = replace(self._s)
        s.sat_pct = 100 * s.sat_time / s.observed_time if s.observed_time else 0.0
        return s


def compute_trial_summary(t, x, th, u, modes=None, u_max=12.0, rail_half=0.4,
                          threshold_deg=2.0, min_duration=1.0):
    t, x, th, u = _arrays(t, x, th, u)
    if modes is not None and len(modes) != len(t):
        raise ValueError("Mod ve örnek sayısı eşleşmeli")
    acc = TrialAccumulator(u_max, threshold_deg, min_duration)
    for i in range(len(t)):
        acc.add(t[i], x[i], th[i], u[i], modes[i] if modes is not None else None)
    return acc.summary()


class LiveTrialTracker:
    """Thread-güvenli deneme yaşam döngüsü; yalnız güncel/son denemeyi O(1) bellekte tutar.

    WAIT, BALANCE ile aynı denemeye aittir. Süreler ilk etkin telemetri örneğinde başlar.
    Eksik sıra numaraları metriklerin sürekliliğini keser.
    """
    ACTIVE = ("WAIT", "SWING_UP", "BALANCE")

    def __init__(self, u_max=12.0, max_gap=1.5):
        self.u_max, self.max_gap = u_max, max_gap
        self._lock = threading.Lock()
        self._acc = None
        self._active = False
        self._t0 = self._last_t = self._last_n = None
        self._paused = False
        self._break_next = False

    def update(self, msg):
        with self._lock:
            if not all(math.isfinite(v) for v in (msg.t, msg.x, msg.th, msg.u)):
                if self._acc:
                    self._acc.break_continuity()
                self._break_next = True
                return
            if self._last_t is not None and msg.t <= self._last_t:
                if msg.t == self._last_t:
                    return
                self._finish("Cihaz saati sıfırlandı")
            continuous = not self._paused and not self._break_next and not (
                self._last_n is not None and msg.n >= 0 and msg.n != self._last_n + 1)
            self._break_next = False
            self._last_t, self._last_n = msg.t, msg.n if msg.n >= 0 else None
            if not self._active and msg.mode in self.ACTIVE:
                self._acc = TrialAccumulator(self.u_max, max_gap=self.max_gap)
                self._t0, self._active = msg.t, True
            if self._active:
                self._acc.add(msg.t - self._t0, msg.x, msg.th, msg.u, msg.mode, continuous)
                if msg.mode not in self.ACTIVE:
                    self._finish(msg.mode)

    def _finish(self, reason):
        if self._active:
            self._acc._s.end_reason = reason
        self._active = False

    def finish(self, reason="Bağlantı kesildi"):
        with self._lock:
            self._finish(reason)
            self._last_t = self._last_n = None
            self._paused = self._break_next = False

    def set_paused(self, on):
        with self._lock:
            self._paused = on
            self._break_next = True
            if self._acc:
                self._acc.break_continuity()

    def set_u_max(self, value):
        if not math.isfinite(value) or value <= 0:
            return
        with self._lock:
            self.u_max = value
            if self._acc:
                self._acc.u_max = value

    def snapshot(self):
        with self._lock:
            return (self._acc.summary() if self._acc else None), self._active
