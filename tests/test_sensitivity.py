"""Parametre hassasiyet analizi birim testleri (Issue #13)."""

from pendulum_lab.analysis.sensitivity import (
    CaseResult,
    DEFAULT_PARAMS_TO_TEST,
    evaluate_result,
    run_sensitivity_sweep,
    ParamSensitivity,
    VariationResult,
    generate_markdown_report,
    get_nested_attr,
    run_single_simulation,
    set_nested_attr,
)
from pendulum_lab.control.lqr import design_lqr
from pendulum_lab.model.params import PendulumParams
from pendulum_lab.sim.simulator import SimResult
import numpy as np
import pytest


def result(angles, modes=None):
    t = np.arange(len(angles))*.01
    s = np.zeros((len(t), 4))
    s[:, 2] = np.radians(angles)
    return SimResult(t, s, np.zeros_like(t), modes or ["BALANCE"]*len(t))


def test_late_capture_is_not_success():
    r = result([180.]*181 + [0.]*20, ["SWING_UP"]*181 + ["BALANCE"]*20)
    case = evaluate_result(r, PendulumParams())
    assert not case.balanced and case.settling_time is None
    assert case.final_hold_s == pytest.approx(.19)


def test_early_settle_followed_by_fall_is_not_success():
    case = evaluate_result(result([0.]*150 + [30.]*51), PendulumParams())
    assert case.settling_time == 0.
    assert not case.balanced


def test_angle_band_without_balance_mode_is_not_success():
    assert not evaluate_result(result([0.]*201, ["WAIT"]*201), PendulumParams()).balanced


def test_fault_and_rail_touch_cannot_be_success():
    r = result([0.]*201)
    r.mode[10] = "FAULT"
    assert not evaluate_result(r, PendulumParams()).balanced
    r.mode[10] = "BALANCE"
    r.s[10, 0] = .4
    assert evaluate_result(r, PendulumParams()).status == "RAY SINIRI"


def test_sweep_includes_inertia_and_new_motor_losses():
    assert {"I", "motor.T_f", "motor.B_m", "motor.J_rotor", "motor.gear"} <= {p[0] for p in DEFAULT_PARAMS_TO_TEST}


def test_zero_nominal_uses_positive_absolute_values_and_keeps_nominal(monkeypatch):
    import pendulum_lab.analysis.sensitivity as module
    seen = []
    def run(nominal, real, K, *args):
        seen.append((nominal.I, real.I, tuple(K)))
        return CaseResult(True, 1., .1, 0., "BALANCE", "BAŞARILI", 2.)
    monkeypatch.setattr(module, "_run_case", run)
    p = PendulumParams()
    sweep = run_sensitivity_sweep(p, K=[1, 2, 3, 4], params_to_test=[("I", "Atalet", "kg·m²")], variations=[-.1, .1])
    assert p.I == 0 and all(n == 0 for n, _, _ in seen)
    assert sweep[0].results[0].status == "UYGULANAMAZ"
    assert sweep[0].results[1].perturbed_val == pytest.approx(.1*p.m*p.l**2)
    assert len(seen) == 4  # nominal pair + positive perturbation pair
    assert all(k == (1, 2, 3, 4) for _, _, k in seen)


def test_report_uses_actual_variations_and_does_not_invent_failure():
    ps = ParamSensitivity("l", "Boy", "m", .3)
    ps.results = [VariationResult(-.1, .3, .27, True, 1., .1, 0., "BALANCE", "BAŞARILI")]
    ps.evaluate_criticality()
    report = generate_markdown_report([ps])
    assert "kesinlikle" not in report
    assert "| Parametre | Nominal | -10% |" in report
    assert "+30%" not in report


def test_nominal_failure_prevents_misleading_sensitivity_ranking(monkeypatch):
    import pendulum_lab.analysis.sensitivity as module
    monkeypatch.setattr(module, "_run_case", lambda *a: CaseResult(False, None, .1, 0., "SWING_UP", "BAŞARISIZ"))
    sweep = run_sensitivity_sweep(K=[1, 2, 3, 4], params_to_test=[("m", "Kütle", "kg")], variations=[.1])
    assert "NOMİNAL BAŞARISIZ" in sweep[0].criticality


def test_unexpected_programming_errors_are_not_swallowed(monkeypatch):
    import pendulum_lab.analysis.sensitivity as module
    def broken(*a, **kw):
        raise AttributeError("bug")
    monkeypatch.setattr(module, "simulate_closed_loop", broken)
    with pytest.raises(AttributeError):
        run_single_simulation(PendulumParams(), PendulumParams(), [1, 2, 3, 4])


def test_get_and_set_nested_attr():
    p = PendulumParams()
    assert get_nested_attr(p, "m") == 0.2
    assert get_nested_attr(p, "motor.R") == 2.2

    set_nested_attr(p, "m", 0.25)
    assert p.m == 0.25

    set_nested_attr(p, "motor.R", 3.0)
    assert p.motor.R == 3.0


def test_criticality_evaluation():
    # ±%20'de başarısız oluyorsa KRİTİK olarak sınıflanmalı
    ps = ParamSensitivity("l", "Sarkaç boyu", "[m]", 0.3)
    ps.results = [
        VariationResult(-0.2, 0.3, 0.24, False, None, 0.45, 0.5, "SWING_UP", "BAŞARISIZ"),
        VariationResult(0.2, 0.3, 0.36, True, 2.5, 0.15, 0.1, "BALANCE", "BAŞARILI"),
    ]
    ps.evaluate_criticality()
    assert "KRİTİK" in ps.criticality

    # Hepsi başarılıysa TOLERANSLI
    ps_safe = ParamSensitivity("M", "Araba kütlesi", "[kg]", 0.5)
    ps_safe.results = [
        VariationResult(-0.3, 0.5, 0.35, True, 2.0, 0.1, 0.1, "BALANCE", "BAŞARILI"),
        VariationResult(0.3, 0.5, 0.65, True, 2.1, 0.1, 0.1, "BALANCE", "BAŞARILI"),
    ]
    ps_safe.evaluate_criticality()
    assert "TOLERANSLI" in ps_safe.criticality


def test_run_single_simulation_nominal():
    p = PendulumParams()
    K, _, _ = design_lqr(p, [10, 1, 100, 1], 0.1, Ts=0.002)
    bal, settle, max_x, sat_time, mode, status = run_single_simulation(p, p, K.ravel(), sim_time=6.0)

    assert bal
    assert status == "BAŞARILI"
    assert mode == "BALANCE"
    assert max_x < p.rail_half


def test_generate_markdown_report():
    ps = ParamSensitivity("l", "Sarkaç boyu", "[m]", 0.3)
    ps.results = [
        VariationResult(-0.1, 0.3, 0.27, True, 2.5, 0.15, 0.1, "BALANCE", "BAŞARILI"),
        VariationResult(0.1, 0.3, 0.33, True, 2.4, 0.14, 0.1, "BALANCE", "BAŞARILI"),
    ]
    ps.evaluate_criticality()
    report = generate_markdown_report([ps])
    assert "# Parametre Hassasiyet Analizi Raporu" in report
    assert "Sarkaç boyu" in report
