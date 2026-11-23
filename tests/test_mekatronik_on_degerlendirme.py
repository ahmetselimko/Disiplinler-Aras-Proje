"""Mekatronik tasarım parametre dosyası ve ön değerlendirme araçları (PR #45 incelemesi)."""

import importlib.util
import math
import sys
from pathlib import Path

import pytest

from pendulum_lab import load_params

ROOT = Path(__file__).resolve().parents[1]


def _load_tool(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / "tools" / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod                      # dataclass'lar modülü sys.modules'ta arar
    spec.loader.exec_module(mod)
    return mod


def test_design_params_match_mechatronics_report():
    p = load_params(ROOT / "config" / "params_mekatronik_tasarim.json")
    m = p.motor
    assert p.placeholder is True                                 # tasarım değeri, ölçüm değil
    assert m.gear == 19.0 and m.r_pulley == 0.009549
    assert m.alpha == pytest.approx(2.26, rel=1e-3)              # Tablo 16: α_V
    assert m.beta == pytest.approx(48.41, rel=1e-3)              # Tablo 16: b_elec
    assert m.reflected_mass == pytest.approx(1.346, rel=1e-3)    # Bölüm 14.1: rotorun arabaya yansıması
    assert p.F_coulomb == pytest.approx(3.0, rel=1e-3)           # Bölüm 14.1: sürtünme
    assert 12 * m.alpha / m.beta == pytest.approx(0.56, abs=0.01)   # yüksüz hız ≈ 0.53–0.56 m/s
    assert p.M + p.m == pytest.approx(1.10 + 0.26 + 0.025)        # 1.10 kg sarkaç dahil + kayış + kasnak
    assert p.I + p.m * p.l ** 2 == pytest.approx(0.002236, rel=1e-3)   # Tablo 15, +20 g, pivota göre


def test_evaluation_tool_runs_and_uses_report_tip_options():
    t = _load_tool("mekatronik_on_degerlendirme")
    assert 1.01 in t.E_RATIOS                                    # mekatroniğin E_ref = 1.02·mgl'si
    base = load_params(t.PARAMS_FILE)
    for m, l, Ip in t.TIP_OPTIONS.values():
        p = t.params_for(base, m, l, Ip)
        assert p.I > 0 and p.M + m == pytest.approx(1.385)
        assert math.sqrt(m * p.g * l / (p.I + m * l * l)) == pytest.approx(
            {0.085: 7.34, 0.105: 6.76, 0.135: 6.54}[m], abs=0.01)   # Tablo 15: ω_n
    status, tb, mx, cl = t.run(t.params_for(base, *t.TIP_OPTIONS["+20 g"]), 1.05, friction_comp=False, T=2.0)
    assert status and mx >= 0 and 0 <= cl <= 180


def test_report_figures_tool_helpers():
    t = _load_tool("rapor_sekilleri")
    assert t.fmt_c(complex(-1.5, 0)) == "-1.50"
    assert t.fmt_c(complex(-1, 2)) == "-1.00 ± 2.00j"
    assert [z.imag >= 0 for z in t.uniq_conj([complex(-1, 2), complex(-1, -2), -3])] == [True, True]
