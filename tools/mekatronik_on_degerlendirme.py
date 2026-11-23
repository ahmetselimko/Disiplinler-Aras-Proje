#!/usr/bin/env python3
"""Mekatronik tasarım ölçüleriyle swing-up taraması: uç kütlesi × hedef enerji oranı (E_ratio).

Uç kütlesi seçenekleri mekatronik ara raporu Bölüm 13.2, Tablo 15'ten. Diğer değerler
config/params_mekatronik_tasarim.json'dan (araba 1.10 kg + kayış + kasnaklar, 19:1 redüktör; motorun elektriksel
değerleri yer tutucu). Kontrolcü arayüzün varsayılanları (tools/rapor_sekilleri.py ile aynı).

    python tools/mekatronik_on_degerlendirme.py

Sonuç Markdown tablo olarak yazdırılır; docs/mekatronik_on_degerlendirme.md'deki tablo buradan gelir.
"""
from __future__ import annotations

import copy
import math
import sys
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT_DIR))

import numpy as np  # noqa: E402

from pendulum_lab.analysis.sensitivity import evaluate_result  # noqa: E402
from pendulum_lab.control.lqr import design_lqr  # noqa: E402
from pendulum_lab.control.supervisor import Supervisor  # noqa: E402
from pendulum_lab.control.swingup import EnergySwingUp  # noqa: E402
from pendulum_lab.model.params import load_params  # noqa: E402
from pendulum_lab.sim.simulator import SensorModel, simulate_closed_loop  # noqa: E402

CART_WITH_PENDULUM = 1.10      # kg, Bölüm 12.1 (sarkaç ve rulmanlar dahil)
BELT_AND_PULLEYS = 0.26 + 0.025  # kg, Bölüm 14.1
# ad: (m [kg], l [m], I_pivot [kg·m²])  — Tablo 15
TIP_OPTIONS = {
    "uç kütlesiz": (0.085, 0.0637, 0.000986),
    "+20 g": (0.105, 0.0992, 0.002236),
    "+50 g": (0.135, 0.1327, 0.004111),
}
E_RATIOS = (1.00, 1.02, 1.05, 1.10)
Q, R, TS, T = (10.0, 1.0, 100.0, 1.0), 0.1, 0.002, 30.0


def params_for(base, m, l, I_pivot):
    p = copy.deepcopy(base)
    p.m, p.l, p.I = m, l, I_pivot - m * l * l
    p.M = CART_WITH_PENDULUM - m + BELT_AND_PULLEYS
    return p


def run(p, e_ratio):
    K, _, _ = design_lqr(p, Q, [[R]], Ts=TS)
    sup = Supervisor(p, K.ravel(), swingup=EnergySwingUp(E_ratio=e_ratio))
    sup.start()
    r = simulate_closed_loop(p, sup, [0, 0, math.pi, 0], T, dt=TS, sensor=SensorModel.for_params(p, seed=0))
    e = evaluate_result(r, p)
    m = np.asarray(r.mode)
    tb = float(r.t[np.argmax(m == "BALANCE")]) if (m == "BALANCE").any() else None
    closest = float(np.abs(np.degrees(np.arctan2(np.sin(r.th), np.cos(r.th))))[r.t > 3.0].min())
    return e.status, tb, float(np.abs(r.x).max()), closest


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    base = load_params(ROOT_DIR / "config/params_mekatronik_tasarim.json")
    print("| Hedef enerji oranı | " + " | ".join(TIP_OPTIONS) + " |")
    print("|---|" + "---|" * len(TIP_OPTIONS))
    for er in E_RATIOS:
        cells = []
        for m, l, Ip in TIP_OPTIONS.values():
            status, tb, mx, cl = run(params_for(base, m, l, Ip), er)
            if status in ("BAŞARILI", "SINIRDA") and tb is not None:
                cells.append(f"✔ {tb:.1f} s (max \\|x\\| {mx:.2f} m)")
            elif status == "RAY SINIRI":
                cells.append("✘ ray ucuna çarptı")
            else:
                cells.append(f"✘ dike en fazla {cl:.0f}°")
        print(f"| {er:.2f} | " + " | ".join(cells) + " |", flush=True)


if __name__ == "__main__":
    main()
