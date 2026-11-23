#!/usr/bin/env python3
"""Rapor şekillerini üretir (#25): aşağıdan kalkış, dürtmeye tepki, kutup haritası, model uyumsuzluğu.

Ayarlar arayüzün varsayılanlarıyla aynıdır (Q = diag(10, 1, 100, 1), R = 0.1, Ts = 2 ms, enerji bazlı
swing-up, yakalama 15°, bırakma 30°, 12-bit açı + 48 CPR enkoder kuantalaması, Kalman açık), yani şekiller
Analiz sekmesinde aynı ayarlarla görülenin aynısıdır. Parametreler değişince (ör. mekatronik gerçek ölçüleri
girince) tek komutla yeniden üretilir:

    python tools/rapor_sekilleri.py                    # config/params.json → docs/media/rapor/
    python tools/rapor_sekilleri.py --params baska.json --out baska_klasor/

Çıktı: dört PNG ve şekil altı yazıları ile sayıları içeren README.md.
"""
from __future__ import annotations

import argparse
import copy
import math
import sys
from dataclasses import dataclass
from datetime import date
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT_DIR))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from pendulum_lab import linearize  # noqa: E402
from pendulum_lab.analysis.sensitivity import evaluate_result  # noqa: E402
from pendulum_lab.control.lqr import design_lqr  # noqa: E402
from pendulum_lab.control.supervisor import Supervisor  # noqa: E402
from pendulum_lab.control.swingup import EnergySwingUp  # noqa: E402
from pendulum_lab.model.params import load_params  # noqa: E402
from pendulum_lab.sim.simulator import SensorModel, simulate_closed_loop  # noqa: E402

# Arayüzün varsayılan tasarım ayarları (gui/state.py)
Q = (10.0, 1.0, 100.0, 1.0)
R = 0.1
TS = 0.002
CATCH_DEG = 15.0
RELEASE_DEG = 30.0

# Deney ayarları
PUSH_START = 1.0             # dürtme anı [s]
PUSH_DUR = 0.05              # dürtme süresi [s] (arayüz varsayılanı 50 ms)
PUSH_FRACTIONS = (0.3, 0.6, 0.9)  # itme kuvvetleri: kurtarılabilen en büyük itmenin bu katları
PUSH_SEARCH_MAX = 10.0       # kurtarılabilen en büyük itmeyi ararken üst sınır [N]
MISMATCH_GRID = tuple(round(v, 2) for v in np.arange(0.70, 1.301, 0.05))   # gerçek m ve l çarpanı
MISMATCH_SHOWN = (0.90, 1.00, 1.10, 1.30)   # zaman grafiğinde gösterilenler
SWING_T = 20.0               # aşağıdan kalkış süresi [s]

# Renkler: dataviz kılavuzunun kategorik sırası (açık zemin), durum renkleri ayrı
CAT = ("#2a78d6", "#eb6834", "#1baf7a", "#eda100")
GOOD, SERIOUS, CRITICAL = "#0ca30c", "#ec835a", "#d03b3b"
INK, INK2, GRID = "#1f1f1e", "#5f5f5c", "#e4e4e0"
BAND = {"SWING_UP": "#f1efe8", "BALANCE": "#e6f0fb", "FAULT": "#fbe4e4"}
MODE_TR = {"SWING_UP": "swing-up", "BALANCE": "denge (LQR)", "FAULT": "hata"}

plt.rcParams.update({
    "font.size": 9, "axes.titlesize": 10, "axes.labelsize": 9, "axes.edgecolor": INK2,
    "axes.labelcolor": INK, "xtick.color": INK2, "ytick.color": INK2, "axes.grid": True,
    "grid.color": GRID, "grid.linewidth": 0.6, "axes.spines.top": False, "axes.spines.right": False,
    "lines.linewidth": 1.6, "legend.frameon": False, "figure.dpi": 100, "savefig.dpi": 200,
    "savefig.bbox": "tight", "axes.titleweight": "bold", "axes.titlelocation": "left",
})


@dataclass
class Setup:
    p: object
    K: np.ndarray
    e_ratio: float = 1.0

    def run(self, real, s0, T, dist=None, target="pole"):
        sup = Supervisor(self.p, self.K, swingup=EnergySwingUp(E_ratio=self.e_ratio), catch_deg=CATCH_DEG, release_deg=RELEASE_DEG)
        sup.start()
        return simulate_closed_loop(real, sup, s0, T, dt=TS, sensor=SensorModel.for_params(self.p, seed=0),
                                    disturbance=dist, disturbance_target=target)


def wrap_deg(th):
    """θ'yı (−180°, 180°] aralığına sarar; ±180° geçişlerinde çizgi kopsun diye NaN ekler."""
    d = np.degrees(np.arctan2(np.sin(th), np.cos(th)))
    d[1:][np.abs(np.diff(d)) > 180] = np.nan
    return d


def dist_deg(th):
    """Dikten uzaklık |θ| [°]: 0 = dik, 180 = aşağıda. Sarma kopukluğu olmaz."""
    return np.abs(np.degrees(np.arctan2(np.sin(th), np.cos(th))))


def legend_above(ax, *args, **kw):
    kw.setdefault("ncol", 4)
    return ax.legend(*args, loc="lower right", bbox_to_anchor=(1.0, 1.0), fontsize=8, borderaxespad=0.2, **kw)


def first_balance(r):
    m = np.asarray(r.mode)
    return float(r.t[np.argmax(m == "BALANCE")]) if (m == "BALANCE").any() else None


def shade_modes(ax, r):
    m = list(r.mode)
    i = 0
    while i < len(m):
        j = i
        while j + 1 < len(m) and m[j + 1] == m[i]:
            j += 1
        if m[i] in BAND:
            ax.axvspan(r.t[i], r.t[j], color=BAND[m[i]], lw=0, zorder=0)
        i = j + 1


def pulse(F):
    return lambda t: F if PUSH_START <= t < PUSH_START + PUSH_DUR else 0.0


# ---------------------------------------------------------------- şekiller

def fig_swing_up(S: Setup, out: Path):
    r = S.run(S.p, [0, 0, math.pi, 0], SWING_T)
    e = evaluate_result(r, S.p)
    tb = first_balance(r)
    fig, axs = plt.subplots(3, 1, figsize=(7.0, 6.2), sharex=True, gridspec_kw={"hspace": 0.18})
    unit = "V" if S.p.input_mode == "voltage" else "N"
    for ax, y, lab in zip(axs, (dist_deg(r.th), r.x, r.u),
                          ("Dikten uzaklık |θ| [°]", "Araba konumu x [m]", f"Motor girişi u [{unit}]")):
        shade_modes(ax, r)
        ax.plot(r.t, y, color=CAT[0])
        ax.set_ylabel(lab)
        if tb is not None:
            ax.axvline(tb, color=INK2, lw=0.8, ls="--")
    axs[0].set_yticks([0, 45, 90, 135, 180])
    axs[0].set_yticklabels(["0 (dik)", "45", "90", "135", "180\n(aşağıda)"])
    axs[1].axhline(S.p.rail_half, color=CRITICAL, lw=0.8, ls=":")
    axs[1].axhline(-S.p.rail_half, color=CRITICAL, lw=0.8, ls=":")
    axs[1].text(SWING_T * 0.99, S.p.rail_half * 0.93, f"ray ucu ±{S.p.rail_half:g} m", color=INK2, va="top",
                ha="right", fontsize=8)
    axs[2].axhline(S.p.u_max, color=INK2, lw=0.6, ls=":")
    axs[2].axhline(-S.p.u_max, color=INK2, lw=0.6, ls=":")
    if tb is not None:
        axs[0].annotate(f"LQR'a geçiş\nt = {tb:.2f} s", xy=(tb, 60), xytext=(tb + 1.0, 100), fontsize=8,
                        color=INK, arrowprops=dict(arrowstyle="-", color=INK2, lw=0.6))
    axs[2].set_xlabel("Zaman [s]")
    axs[0].set_title("Aşağıdan kalkış (θ₀ = 180°)")
    handles = [plt.Rectangle((0, 0), 1, 1, facecolor=BAND[k], edgecolor=INK2, lw=0.5) for k in ("SWING_UP", "BALANCE")]
    legend_above(axs[0], handles, [MODE_TR["SWING_UP"], MODE_TR["BALANCE"]], ncol=2)
    fig.savefig(out / "asagidan_kalkis.png")
    plt.close(fig)
    return dict(first_balance=tb, settle=e.settling_time, max_x=e.max_x, max_u=float(np.abs(r.u).max()),
                status=e.status)


def max_recoverable_push(S: Setup):
    lo, hi = 0.0, PUSH_SEARCH_MAX
    for _ in range(14):
        mid = 0.5 * (lo + hi)
        r = S.run(S.p, [0, 0, 0, 0], 6.0, dist=pulse(mid))
        ok = evaluate_result(r, S.p).status in ("BAŞARILI", "SINIRDA") and "SWING_UP" not in r.mode
        lo, hi = (mid, hi) if ok else (lo, mid)
    return lo


def fig_push(S: Setup, out: Path):
    T = 6.0
    rows = []
    push_max = max_recoverable_push(S)
    forces = [round(f * push_max, 1) for f in PUSH_FRACTIONS]
    fig, axs = plt.subplots(2, 1, figsize=(7.0, 4.8), sharex=True, gridspec_kw={"hspace": 0.15})
    for F, col in zip(forces, CAT):
        r = S.run(S.p, [0, 0, 0, 0], T, dist=pulse(F))
        e = evaluate_result(r, S.p)
        th = wrap_deg(r.th)
        axs[0].plot(r.t, th, color=col, label=f"{F:g} N")
        axs[1].plot(r.t, r.x, color=col, label=f"{F:g} N")
        k = int(np.nanargmax(np.abs(th)))
        axs[0].annotate(f"{F:g} N", xy=(r.t[k], th[k]), xytext=(4, 0), textcoords="offset points",
                        color=INK, fontsize=8, va="center")
        out_band = np.flatnonzero(np.abs(th) > 2.0)            # ±2° bandının dışı
        settle = float(r.t[out_band[-1] + 1] - PUSH_START) if len(out_band) and out_band[-1] + 1 < len(r.t) else None
        fell = "SWING_UP" in list(r.mode)                     # LQR bıraktı, swing-up yeniden kaldırdı
        rows.append(dict(F=F, max_th=float(np.nanmax(np.abs(th))), max_x=float(np.abs(r.x).max()),
                         settle=settle, status="DÜŞTÜ" if fell and e.status in ("BAŞARILI", "SINIRDA") else e.status))
    for ax in axs:
        ax.axvspan(PUSH_START, PUSH_START + PUSH_DUR, color=BAND["FAULT"], lw=0, zorder=0)
    axs[1].annotate(f"itme ({PUSH_DUR * 1000:g} ms)", xy=(PUSH_START + PUSH_DUR, axs[1].get_ylim()[1] * 0.9),
                    xytext=(PUSH_START - 0.15, axs[1].get_ylim()[1] * 0.9), ha="right", va="center", color=INK2,
                    fontsize=8, arrowprops=dict(arrowstyle="-", color=INK2, lw=0.6))
    axs[0].set_ylabel("θ [°]")
    axs[1].set_ylabel("x [m]")
    axs[1].set_xlabel("Zaman [s]")
    axs[0].set_title("Dengedeyken sarkaca itme")
    legend_above(axs[0], title="itme kuvveti", title_fontsize=8, ncol=3)
    fig.savefig(out / "durtme.png")
    plt.close(fig)
    return rows, push_max


def fig_poles(S: Setup, out: Path):
    from scipy.linalg import expm
    A, B = linearize(S.p)
    n = A.shape[0]
    M = expm(np.block([[A, B], [np.zeros((1, n + 1))]]) * TS)
    Ad, Bd = M[:n, :n], M[:n, n:]
    ol = np.linalg.eigvals(A)
    cld = np.linalg.eigvals(Ad - Bd @ S.K.reshape(1, -1))
    cls = np.log(cld.astype(complex)) / TS          # ayrık kutupların sürekli karşılığı
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(7.4, 3.6), gridspec_kw={"wspace": 0.28, "width_ratios": (1, 1.15)})
    zoom = max(8.0, 1.6 * float(max(abs(z.real) for z in np.concatenate([ol, cls]) if abs(z.real) < 20)))
    im = max(1.0, 1.3 * float(np.abs(np.concatenate([ol.imag, cls.imag])).max()))
    for ax, full in ((a1, True), (a2, False)):
        ax.axhline(0, color=INK2, lw=0.6)
        ax.axvline(0, color=INK2, lw=0.6)
        ax.axvspan(0, 1e3, color=BAND["FAULT"], lw=0, zorder=0)
        ax.scatter(ol.real, ol.imag, s=55, facecolors="white", edgecolors=INK2, linewidths=1.4, zorder=3,
                   label="açık çevrim")
        ax.scatter(cls.real, cls.imag, s=55, marker="x", color=CAT[0], linewidths=1.8, zorder=4,
                   label="kapalı çevrim (LQR)")
        ax.set_ylim(-im, im)
        ax.set_xlabel("Gerçel kısım [1/s]")
        if not full:
            ax.set_xlim(-zoom, zoom * 0.75)
            # değerleri yaz: açık çevrim üstte, kapalı çevrim altta (aynı noktada üst üste binmesin)
            for z in uniq_conj(ol):
                if -zoom < z.real < zoom:
                    ax.annotate(fmt_c(z), (z.real, z.imag), xytext=(0, 9), textcoords="offset points",
                                ha="center", fontsize=7, color=INK2, rotation=60)
            for z in uniq_conj(cls):
                if -zoom < z.real < zoom:
                    ax.annotate(fmt_c(z), (z.real, z.imag), xytext=(0, -10), textcoords="offset points",
                                ha="center", va="top", fontsize=7, color=CAT[0], rotation=60)
    a1.set_ylabel("Sanal kısım [rad/s]")
    a1.set_xlim(min(np.concatenate([ol.real, cls.real])) * 1.12, max(8.0, max(ol.real) * 1.5))
    a1.axvspan(-zoom, zoom * 0.75, facecolor="none", edgecolor=INK2, lw=0.8, ls="--")
    a1.text(0.98, 0.04, "kararsız\nbölge →", transform=a1.transAxes, ha="right", fontsize=7, color=INK2)
    a1.set_title("s-düzlemi")
    a2.set_title(f"Yakın görünüm (kesikli çerçeve)")
    h, lab = a2.get_legend_handles_labels()
    fig.legend(h, lab, loc="lower center", bbox_to_anchor=(0.5, -0.12), ncol=3, fontsize=8)
    fig.suptitle("Kutup haritası", x=0.06, ha="left", fontweight="bold", fontsize=10, y=1.0)
    fig.savefig(out / "kutup_haritasi.png")
    plt.close(fig)
    return dict(open_loop=ol, closed_s=cls, closed_z=cld, max_abs_z=float(np.abs(cld).max()))


def fig_mismatch(S: Setup, out: Path):
    T = 30.0
    res = {}
    for k in MISMATCH_GRID:
        real = copy.deepcopy(S.p)
        real.m *= k
        real.l *= k
        r = S.run(real, [0, 0, math.pi, 0], T)
        e = evaluate_result(r, S.p)
        res[k] = (r, e, first_balance(r))
    fig = plt.figure(figsize=(7.4, 5.8))
    gs = fig.add_gridspec(2, 1, height_ratios=(1.25, 1.0), hspace=0.42)
    a1 = fig.add_subplot(gs[0])
    for k, col in zip(MISMATCH_SHOWN, CAT):
        r = res[k][0]
        sel = r.t <= 15.0
        a1.plot(r.t[sel], dist_deg(r.th)[sel], color=col, lw=1.3, label=f"{(k - 1) * 100:+.0f}%")
    a1.set_xlim(0, 15)
    a1.set_ylim(-5, 185)
    a1.set_yticks([0, 45, 90, 135, 180])
    a1.set_yticklabels(["0 (dik)", "45", "90", "135", "180\n(aşağıda)"])
    a1.set_ylabel("Dikten uzaklık |θ| [°]")
    a1.set_xlabel("Zaman [s]")
    a1.set_title("Aşağıdan kalkış: gerçek m ve l modeldekinden farklı", pad=22)
    legend_above(a1, title="m ve l sapması", title_fontsize=8, ncol=4)

    a2 = fig.add_subplot(gs[1])
    pct = [(k - 1) * 100 for k in MISMATCH_GRID]
    ok_t = [tb for (_, e, tb) in res.values() if tb is not None and e.status in ("BAŞARILI", "SINIRDA")]
    top = max(12.5, math.ceil(1.25 * max(ok_t, default=0.0) / 2.5) * 2.5)
    for k, x in zip(MISMATCH_GRID, pct):
        r, e, tb = res[k]
        if e.status in ("BAŞARILI", "SINIRDA"):
            a2.scatter(x, tb, s=60, color=GOOD, zorder=3)
        elif e.status == "RAY SINIRI":
            a2.scatter(x, top, s=60, marker="^", color=SERIOUS, zorder=3)
        else:
            a2.scatter(x, top, s=60, marker="x", color=CRITICAL, linewidths=2, zorder=3)
    a2.axhline(top - 1.25, color=INK2, lw=0.6, ls=":")
    a2.set_ylim(0, top + 1.2)
    ticks = [v for v in np.arange(0, top - 1.9, 2.5 if top <= 15 else 5.0)]
    a2.set_yticks(ticks + [top])
    a2.set_yticklabels([f"{v:g}" for v in ticks] + ["gelemedi"])
    a2.set_xticks(pct)
    a2.set_xticklabels([f"{v:+.0f}%" if v else "0" for v in pct], fontsize=8)
    a2.set_xlabel("Gerçek m ve l'nin modele göre sapması")
    a2.set_ylabel("LQR'a geçiş [s]")
    a2.set_title(f"Her sapma için sonuç ({T:g} s)", pad=22)
    h = [plt.Line2D([], [], ls="", marker="o", color=GOOD, ms=7),
         plt.Line2D([], [], ls="", marker="^", color=SERIOUS, ms=7),
         plt.Line2D([], [], ls="", marker="x", color=CRITICAL, ms=7, mew=2)]
    legend_above(a2, h, ["dengede kaldı", "ray ucuna çarptı", "swing-up bitmedi"], ncol=3)
    fig.savefig(out / "model_uyumsuzlugu.png")
    plt.close(fig)
    # sarkacın dik konuma en çok yaklaştığı açı (ilk 3 s'lik sallanma hariç)
    closest = {k: float(dist_deg(r.th)[r.t > 3.0].min()) for k, (r, e, tb) in res.items()}
    return {k: (e.status, tb, e.max_x, closest[k]) for k, (r, e, tb) in res.items()}


# ---------------------------------------------------------------- README

def fmt_c(z, d=2):
    return f"{z.real:.{d}f}" if abs(z.imag) < 1e-9 else f"{z.real:.{d}f} ± {abs(z.imag):.{d}f}j"


def uniq_conj(zs):
    return [z for z in sorted(zs, key=lambda c: (c.real, -c.imag)) if z.imag >= -1e-9]


def write_readme(out: Path, S: Setup, params_path: Path, sw, push_rows, push_max, poles, mm):
    unit = "V" if S.p.input_mode == "voltage" else "N"
    st = lambda s: {"BAŞARILI": "dengede kaldı", "SINIRDA": "dengede kaldı (ray ucuna yakın)",
                    "RAY SINIRI": "ray ucuna çarptı", "BAŞARISIZ": "swing-up bitmedi", "FAULT": "hata",
                    "DÜŞTÜ": "düştü, swing-up yeniden kaldırdı"}.get(s, s)
    placeholder = getattr(S.p, "placeholder", None)
    L = [f"# Rapor şekilleri",
         "",
         f"`tools/rapor_sekilleri.py` ile {date.today():%d.%m.%Y} tarihinde üretildi. **Elle düzenlemeyin;** "
         "parametreler değişince script'i yeniden çalıştırın.",
         "",
         f"* Parametreler: `{params_path.relative_to(ROOT_DIR) if params_path.is_relative_to(ROOT_DIR) else params_path}`"
         f" (M = {S.p.M:g} kg, m = {S.p.m:g} kg, l = {S.p.l:g} m, ray ±{S.p.rail_half:g} m, "
         f"giriş: {'motor gerilimi' if unit == 'V' else 'kuvvet'}, |u| ≤ {S.p.u_max:g} {unit})",
         f"* Kontrolcü: arayüzün varsayılanları; Q = diag{Q}, R = {R:g}, Ts = {TS * 1000:g} ms, "
         f"yakalama {CATCH_DEG:g}°, bırakma {RELEASE_DEG:g}°, enerji bazlı swing-up (hedef enerji oranı "
         f"{S.e_ratio:g})",
         "* K = [" + ", ".join(f"{v:.2f}" for v in S.K) + "]",
         "* Sensör: 12-bit açı ve 48 CPR enkoder kuantalaması; dik bölgede Kalman (firmware gibi)",
         ""]
    if placeholder or "YER TUTUCU" in str(getattr(S.p, "note", "")):
        L += ["> ⚠ **Parametreler yer tutucu değerlerdir.** Mekatronik ekibi gerçek ölçüleri `config/params.json`'a "
              "girince şekiller yeniden üretilmelidir; sayılar değişecektir.", ""]
    if S.e_ratio != 1.0:
        L[-1:-1] = [f"> Swing-up hedef enerji oranı arayüz varsayılanından (1.0) farklı: **{S.e_ratio:g}**.", ""]
    L += ["## 1. Aşağıdan kalkış — `asagidan_kalkis.png`",
          "",
          "![Aşağıdan kalkış](asagidan_kalkis.png)",
          "",
          f"*Şekil 1. Sarkaç aşağıda (θ₀ = 180°) dururken enerji bazlı swing-up ile sallanır, |θ| < {CATCH_DEG:g}° "
          f"olunca LQR'a geçilir. Gri bölge swing-up, mavi bölge denge modudur.*",
          "",
          f"| LQR'a geçiş | Yerleşme (±2°) | Max \\|x\\| | Max \\|u\\| | Sonuç |",
          "|---|---|---|---|---|",
          f"| {sw['first_balance']:.2f} s | {sw['settle']:.2f} s | {sw['max_x']:.3f} m | {sw['max_u']:.2f} {unit} | "
          f"{st(sw['status'])} |",
          "",
          "## 2. Dürtmeye tepki — `durtme.png`",
          "",
          "![Dürtmeye tepki](durtme.png)",
          "",
          f"*Şekil 2. Sarkaç dengedeyken t = {PUSH_START:g} s'de ağırlık merkezine {PUSH_DUR * 1000:g} ms süren "
          "yatay itme uygulanır. Kuvvetler, LQR'ın kurtarabildiği en büyük itmenin "
          + ", ".join(f"%{f * 100:.0f}" for f in PUSH_FRACTIONS) + "'ıdır.*",
          "",
          "| İtme | Max \\|θ\\| | Max \\|x\\| | İtmeden sonra yerleşme (±2°) | Sonuç |",
          "|---|---|---|---|---|"]
    for row in push_rows:
        s_ = f"{row['settle']:.2f} s" if row["settle"] is not None else "—"
        L.append(f"| {row['F']:g} N | {row['max_th']:.1f}° | {row['max_x']:.3f} m | {s_} | {st(row['status'])} |")
    L += ["",
          f"LQR'ın swing-up'a düşmeden kurtarabildiği en büyük itme (aynı süreyle): **≈ {push_max:.1f} N**. "
          "Daha büyük itmelerde LQR sarkacı tutamıyor; sarkaç düşüyor ya da araba ray ucuna çarpıyor.",
          "",
          "## 3. Kutup haritası — `kutup_haritasi.png`",
          "",
          "![Kutup haritası](kutup_haritasi.png)",
          "",
          "*Şekil 3. Dik konum etrafında doğrusallaştırılmış modelin açık çevrim kutupları ve LQR ile kapalı çevrim "
          "kutupları. Sağdaki panel sıfır civarını büyütür; açık çevrimdeki pozitif kutup sarkacın dik konumda "
          "kararsız olduğunu gösterir. Ayrık zamanlı (Ts = 2 ms) karşılıklar tablodadır; hepsi birim çemberin "
          "içindedir.*",
          "",
          "| | Kutuplar |",
          "|---|---|",
          "| Açık çevrim (s) | " + ", ".join(fmt_c(z) for z in uniq_conj(poles["open_loop"])) + " |",
          "| Kapalı çevrim (s karşılığı) | " + ", ".join(fmt_c(z) for z in uniq_conj(poles["closed_s"])) + " |",
          "| Kapalı çevrim (z) | " + ", ".join(fmt_c(z, 4) for z in uniq_conj(poles["closed_z"])) +
          f" (en büyük \\|z\\| = {poles['max_abs_z']:.4f}) |",
          "",
          "## 4. Model uyumsuzluğu — `model_uyumsuzlugu.png`",
          "",
          "![Model uyumsuzluğu](model_uyumsuzlugu.png)",
          "",
          "*Şekil 4. Kontrolcü nominal modelle tasarlanır; simülasyondaki \"gerçek\" sarkacın kütlesi ve "
          "ağırlık merkezi uzaklığı birlikte −%30 … +%30 değiştirilir (arayüzdeki \"model uyumsuzluğu\" ayarı). "
          "Aşağıdan kalkış, 30 s.*",
          "",
          "| m ve l sapması | Sonuç | LQR'a geçiş | Max \\|x\\| | Dike en yakın (t > 3 s) |",
          "|---|---|---|---|---|"]
    for k, (s_, tb, mx, cl) in mm.items():
        L.append(f"| {(k - 1) * 100:+.0f}% | {st(s_)} | {f'{tb:.2f} s' if tb is not None else '—'} | {mx:.3f} m | "
                 f"{cl:.1f}° |")
    def pct_range(ks):
        ks = sorted(ks)
        return f"{(ks[0] - 1) * 100:+.0f}%" if len(ks) == 1 else f"{(ks[0] - 1) * 100:+.0f}% … {(ks[-1] - 1) * 100:+.0f}%"

    stall = {k: v for k, v in mm.items() if v[0] == "BAŞARISIZ"}
    rail = [k for k, v in mm.items() if v[0] == "RAY SINIRI"]
    for side, ks in (("hafif/kısa", [k for k in stall if k < 1]), ("ağır/uzun", [k for k in stall if k > 1])):
        if not ks:
            continue
        cl = [mm[k][3] for k in ks]
        L += ["",
              f"**Not:** Gerçek sarkaç modeldekinden {side} olduğunda ({pct_range(ks)}) swing-up tamamlanmıyor: sarkaç "
              f"her salınımda dik konuma ancak {min(cl):.0f}–{max(cl):.0f}°'ye kadar yaklaşıyor, {CATCH_DEG:g}°'lik "
              "yakalama bölgesine giremiyor. Olası sebep: swing-up hedef enerjiyi nominal modelle hesaplıyor; gerçek sarkaç "
              "farklıysa bu hedef de kayıyor."]
        if side == "hafif/kısa":
            L[-1] += (" Bu, [hassasiyet raporundaki](../../hassasiyet_raporu.md) `l` parametresinin \"KRİTİK\" "
                      "sonucuyla tutarlıdır.")
    if rail:
        L += ["", f"Araba ray ucuna çarpan noktalar: {', '.join(f'{(k - 1) * 100:+.0f}%' for k in rail)}."]
    (out / "README.md").write_text("\n".join(L) + "\n", encoding="utf-8")


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--params", type=Path, default=ROOT_DIR / "config/params.json")
    ap.add_argument("--out", type=Path, default=ROOT_DIR / "docs/media/rapor")
    ap.add_argument("--e-ratio", type=float, default=1.0,
                    help="swing-up hedef enerji oranı E_ref / (m·g·l) (arayüz varsayılanı 1.0)")
    args = ap.parse_args()
    p = load_params(args.params)
    K, _, _ = design_lqr(p, Q, [[R]], Ts=TS)
    S = Setup(p, K.ravel(), args.e_ratio)
    args.out.mkdir(parents=True, exist_ok=True)
    print("1/4 aşağıdan kalkış…", flush=True)
    sw = fig_swing_up(S, args.out)
    print("2/4 dürtme…", flush=True)
    push_rows, push_max = fig_push(S, args.out)
    print("3/4 kutup haritası…", flush=True)
    poles = fig_poles(S, args.out)
    print("4/4 model uyumsuzluğu…", flush=True)
    mm = fig_mismatch(S, args.out)
    write_readme(args.out, S, args.params.resolve(), sw, push_rows, push_max, poles, mm)
    print(f"Tamamlandı: {args.out}")


if __name__ == "__main__":
    main()
