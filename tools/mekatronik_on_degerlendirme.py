#!/usr/bin/env python3
"""Mekatronik tasarım ölçüleriyle swing-up taraması: uç kütlesi × hedef enerji oranı (E_ratio).

Uç kütlesi seçenekleri mekatronik ara raporu (23.11.2026 sürümü) Bölüm 13.2, Tablo 15'ten. Diğer değerler
config/params_mekatronik_tasarim.json'dan: araba 1.10 kg (sarkaç dahil) + kayış + kasnaklar, JGB37-520 19:1; motor
Tablo 16'daki α_V = 2.26 N/V ve b_elec = 48.41 N·s/m'ye göre, rotorun arabaya yansıyan 1.346 kg'ı ve 3.0 N sürtünme
dahil. Kontrolcü arayüzün varsayılanları (tools/rapor_sekilleri.py ile aynı).

E_ratio bizim tanımımızdır: hedef enerji aşağı-durağan konumdan ölçülür ve dik konum enerjisine (2mgl) oranlanır.
Mekatroniğin E_ref = 1.02·mgl'si (enerji dikte +mgl, aşağıda −mgl) bizim tanımımızda E_ratio = 1.01'dir.

İki tablo yazdırılır: sürtünme telafisi AÇIK (kontrolcü 3 N'u biliyor, firmware'de ölçülen fc_volt gönderilmiş gibi)
ve KAPALI (fc_volt = 0, şu anki varsayılan).

    python tools/mekatronik_on_degerlendirme.py

Sonuç Markdown tablo olarak yazdırılır; docs/mekatronik_on_degerlendirme.md'deki tablolar buradan gelir.
"""
from __future__ import annotations

import copy
import math
import sys
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT_DIR))
sys.path.insert(0, str(ROOT_DIR / "tools"))

import numpy as np  # noqa: E402

from pendulum_lab.analysis.sensitivity import evaluate_result  # noqa: E402
from pendulum_lab.control.lqr import design_lqr  # noqa: E402
from pendulum_lab.control.supervisor import Supervisor  # noqa: E402
from pendulum_lab.control.swingup import EnergySwingUp  # noqa: E402
from pendulum_lab.model.params import load_params  # noqa: E402
from pendulum_lab.sim.simulator import SensorModel, simulate_closed_loop  # noqa: E402
from rapor_sekilleri import classify  # noqa: E402  (şekillerle aynı sonuç sınıflandırması)

PARAMS_FILE = ROOT_DIR / "config/params_mekatronik_tasarim.json"
CART_WITH_PENDULUM = 1.10      # kg, Bölüm 12.1 (sarkaç ve rulmanlar dahil)
BELT_AND_PULLEYS = 0.26 + 0.025  # kg, Bölüm 14.1
# ad: (m [kg], l [m], I_pivot [kg·m²])  — Tablo 15
TIP_OPTIONS = {
    "uç kütlesiz": (0.085, 0.0637, 0.000986),
    "+20 g": (0.105, 0.0992, 0.002236),
    "+50 g": (0.135, 0.1327, 0.004111),
}
E_RATIOS = (1.00, 1.01, 1.02, 1.05, 1.10)   # 1.01 = mekatroniğin E_ref = 1.02·mgl'si
Q, R, TS, T = (10.0, 1.0, 100.0, 1.0), 0.1, 0.002, 30.0


def params_for(base, m, l, I_pivot):
    p = copy.deepcopy(base)
    p.m, p.l, p.I = m, l, I_pivot - m * l * l
    p.M = CART_WITH_PENDULUM - m + BELT_AND_PULLEYS
    return p


def run(p, e_ratio, friction_comp=True, T=T):
    """Aşağıdan kalkış. friction_comp=False: kontrolcü sürtünmeyi bilmiyor (fc_volt = 0), fizikte 3 N var."""
    K, _, _ = design_lqr(p, Q, [[R]], Ts=TS)
    p_ctrl = p
    if not friction_comp:
        p_ctrl = copy.deepcopy(p)
        p_ctrl.motor.T_f = 0.0
    sup = Supervisor(p_ctrl, K.ravel(), swingup=EnergySwingUp(E_ratio=e_ratio))
    sup.start()
    r = simulate_closed_loop(p, sup, [0, 0, math.pi, 0], T, dt=TS, sensor=SensorModel.for_params(p, seed=0))
    e = evaluate_result(r, p)
    m = np.asarray(r.mode)
    tb = float(r.t[np.argmax(m == "BALANCE")]) if (m == "BALANCE").any() else None
    status = classify(r, e)
    # dike en yakın açı, ilk sallanma (3 s; kısa koşuda sürenin yarısı) hariç
    closest = float(np.abs(np.degrees(np.arctan2(np.sin(r.th), np.cos(r.th))))[r.t > min(3.0, 0.5 * T)].min())
    return status, tb, float(np.abs(r.x).max()), closest


def cell(status, tb, mx, cl):
    if status in ("BAŞARILI", "SINIRDA") and tb is not None:
        return f"✔ {tb:.1f} s (max \\|x\\| {mx:.2f} m)"
    if status == "DÜŞTÜ":
        return f"✔ ilk yakalama {tb:.1f} s; düştü, yeniden yakalandı (max \\|x\\| {mx:.2f} m)"
    if status == "TUTAMADI":
        return f"✘ {tb:.1f} s'de yakalandı ama tutulamadı"
    if status == "OTURMADI":
        return f"✘ {tb:.1f} s'de yakalandı, süre sonunda henüz oturmadı"
    if status == "RAY SINIRI":
        return "✘ ray ucuna çarptı"
    if status == "FAULT":
        return "✘ hata (yazılımsal ray sınırı)"
    return f"✘ dike en fazla {cl:.0f}°"


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    base = load_params(PARAMS_FILE)
    for comp in (True, False):
        print(f"\nSürtünme telafisi {'AÇIK' if comp else 'KAPALI'}\n")
        print("| Hedef enerji oranı | " + " | ".join(TIP_OPTIONS) + " |")
        print("|---|" + "---|" * len(TIP_OPTIONS))
        for er in E_RATIOS:
            cells = [cell(*run(params_for(base, m, l, Ip), er, comp)) for m, l, Ip in TIP_OPTIONS.values()]
            print(f"| {er:.2f} | " + " | ".join(cells) + " |", flush=True)


if __name__ == "__main__":
    main()
