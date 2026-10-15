import math

import numpy as np
import pytest

from pendulum_lab import (PendulumParams, Supervisor, Mode, EnergySwingUp, PID, StateSpace,
                          linearize, lqr, dlqr, c2d, rlocus, step_info,
                          simulate_closed_loop, SensorModel)
from pendulum_lab.control.lqr import design_lqr

Q = [10, 1, 100, 1]
R = [[0.1]]


@pytest.fixture
def p():
    return PendulumParams()


def test_lqr_matches_python_control(p):
    control = pytest.importorskip("control")
    A, B = linearize(p)
    K, _, _ = lqr(A, B, Q, R)
    Kc, _, _ = control.lqr(A, B, np.diag(Q), R)
    np.testing.assert_allclose(K, Kc, rtol=1e-6)


def test_dlqr_matches_python_control(p):
    control = pytest.importorskip("control")
    Ad, Bd = c2d(*linearize(p), 0.002)
    K, _, _ = dlqr(Ad, Bd, Q, R)
    Kc, _, _ = control.dlqr(Ad, Bd, np.diag(Q), R)
    np.testing.assert_allclose(K, Kc, rtol=1e-6)


def test_closed_loop_poles_stable(p):
    _, _, ev = design_lqr(p, Q, R)
    assert np.all(ev.real < 0)
    _, _, evd = design_lqr(p, Q, R, Ts=0.002)
    assert np.all(np.abs(evd) < 1)


def test_c2d_matches_scipy(p):
    from scipy.signal import cont2discrete
    A, B = linearize(p)
    Ad, Bd = c2d(A, B, 0.002)
    Ad2, Bd2, *_ = cont2discrete((A, B, np.eye(4), np.zeros((4, 1))), 0.002)
    np.testing.assert_allclose(Ad, Ad2, atol=1e-12)
    np.testing.assert_allclose(Bd, Bd2, atol=1e-12)


@pytest.mark.parametrize("th0_deg", [5, 10, 15, -15])
def test_lqr_balances_nonlinear_plant_at_esp32_rate(p, th0_deg):
    K, _, _ = design_lqr(p, Q, R, Ts=0.002)
    sup = Supervisor(p, K)
    sup.mode = Mode.BALANCE
    r = simulate_closed_loop(p, sup, [0, 0, math.radians(th0_deg), 0], 5.0, sensor=SensorModel())
    assert set(r.mode) == {"BALANCE"}
    assert abs(r.s[-1, 2]) < math.radians(0.5)
    assert abs(r.s[-1, 0]) < 0.01
    assert np.abs(r.x).max() < p.rail_half


def test_lqr_survives_sensor_noise_and_delay(p):
    K, _, _ = design_lqr(p, Q, R, Ts=0.002)
    sup = Supervisor(p, K)
    sup.mode = Mode.BALANCE
    sensor = SensorModel(th_noise_std=math.radians(0.1), x_noise_std=2e-4)
    r = simulate_closed_loop(p, sup, [0, 0, math.radians(8), 0], 5.0, sensor=sensor, delay_steps=1)
    assert set(r.mode) == {"BALANCE"}
    assert abs(r.s[-1, 2]) < math.radians(1.0)


@pytest.mark.parametrize("th0", [math.pi, math.pi - 0.05, -math.pi + 0.3])
def test_swing_up_then_catch_with_lqr(p, th0):
    K, _, _ = design_lqr(p, Q, R, Ts=0.002)
    sup = Supervisor(p, K)
    sup.start()
    r = simulate_closed_loop(p, sup, [0, 0, th0, 0], 15.0, sensor=SensorModel())
    assert "SWING_UP" in r.mode and r.mode[-1] == "BALANCE"
    assert abs(r.s[-1, 2]) < math.radians(1.0)
    assert np.abs(r.x).max() < p.rail_half - 0.05


def test_swing_up_robust_to_model_mismatch(p):
    """Kazançlar yer tutucu modelle tasarlanır, gerçek sistem %20 farklı olsa da çalışmalı."""
    K, _, _ = design_lqr(p, Q, R, Ts=0.002)
    real = PendulumParams(M=p.M * 1.2, m=p.m * 0.85, l=p.l * 1.1, b=p.b * 2)
    sup = Supervisor(p, K)  # kontrolcü yanlış (nominal) modeli kullanıyor
    sup.start()
    r = simulate_closed_loop(real, sup, [0, 0, math.pi, 0], 20.0, sensor=SensorModel())
    assert r.mode[-1] == "BALANCE"


def test_supervisor_hysteresis_and_fault(p):
    sup = Supervisor(p, [0, 0, 0, 0])
    sup.start()
    assert sup.mode == Mode.SWING_UP
    sup.update([0, 0, math.radians(10), 0])
    assert sup.mode == Mode.BALANCE
    sup.update([0, 0, math.radians(20), 0])      # catch < 20° < release → BALANCE'ta kal
    assert sup.mode == Mode.BALANCE
    sup.update([0, 0, math.radians(35), 0])
    assert sup.mode == Mode.SWING_UP
    u, mode = sup.update([p.rail_half, 0, math.pi, 0])
    assert mode == Mode.FAULT and u == 0.0
    sup.stop()
    assert sup.mode == Mode.IDLE


def test_supervisor_rejects_bad_gain(p):
    sup = Supervisor(p, [0, 0, 0, 0])
    with pytest.raises(ValueError):
        sup.set_gain([1, 2, 3])
    with pytest.raises(ValueError):
        sup.set_gain([1, 2, float("nan"), 4])


def test_state_space_properties(p):
    ss = StateSpace.from_params(p)
    assert ss.is_controllable()
    assert ss.is_observable()
    assert not ss.is_stable()
    K, _, _ = design_lqr(p, Q, R)
    assert ss.closed_loop(K).is_stable()


def test_closed_loop_step_tracks_cart_reference(p):
    K, _, _ = design_lqr(p, Q, R)
    t, y = StateSpace.from_params(p).closed_loop(K).step(T=8.0, n=2000)
    info = step_info(t, y[:, 0])
    assert info["final"] == pytest.approx(1.0, abs=1e-3)
    assert info["undershoot_pct"] > 0      # araba önce ters yöne gider (minimum faz dışı)
    assert info["settling_time"] < 8.0


def test_transfer_function_theta_over_input(p):
    """tf'nin payı ve paydası, G(s) = C (sI − A)⁻¹ B'nin doğrudan hesabıyla birkaç frekansta eşleşmeli."""
    ss = StateSpace.from_params(p)
    num, den = ss.tf(output=1)
    for s in (0.7j, 3.0 + 2.0j, -1.5 + 10j):
        direct = (ss.C[1] @ np.linalg.solve(s * np.eye(4) - ss.A, ss.B)).item()
        assert np.polyval(num, s) / np.polyval(den, s) == pytest.approx(direct, rel=1e-8)


def test_rlocus_k0_gives_open_loop_poles():
    gains, roots = rlocus([1], [1, 3, 2])
    np.testing.assert_allclose(np.sort(roots[0].real), [-2, -1])
    assert roots.shape == (len(gains), 2)


def test_pid_converges_on_first_order_plant():
    pid = PID(Kp=2.0, Ki=5.0, Kd=0.0, dt=0.01, u_min=-10, u_max=10)
    y = 0.0
    for _ in range(1000):
        u = pid.update(1.0, y)
        y += 0.01 * (-y + u)
    assert y == pytest.approx(1.0, abs=1e-3)


def test_swingup_accel_saturates(p):
    su = EnergySwingUp()
    for s in ([0, 0, math.pi, 0], [0.1, 1.0, 2.0, 5.0], [-0.3, -1, -2.5, -6]):
        assert abs(su.accel(p, s)) <= su.a_max + 1e-12


def test_wait_mode_when_swingup_disabled(p):
    """Swing-up kapalıyken sarkaç elle kaldırılana kadar motor çalışmaz; dik konuma gelince LQR yakalar."""
    K, _, _ = design_lqr(p, Q, R, Ts=0.002)
    sup = Supervisor(p, K, swingup_enabled=False)
    sup.start()
    assert sup.mode == Mode.WAIT
    u, mode = sup.update([0, 0, math.pi, 0])
    assert (u, mode) == (0.0, Mode.WAIT)
    _, mode = sup.update([0, 0, math.radians(5), 0])
    assert mode == Mode.BALANCE
    _, mode = sup.update([0, 0, math.radians(40), 0])
    assert mode == Mode.WAIT                     # bırakınca swing-up'a değil WAIT'e döner


def test_start_rejected_from_fault_and_running(p):
    sup = Supervisor(p, [0, 0, 0, 0])
    sup.mode = Mode.FAULT
    ok, msg = sup.start()
    assert not ok and sup.mode == Mode.FAULT and "stop" in msg
    sup.stop()
    assert sup.start() == (True, "")
    assert sup.start()[0] is False               # zaten çalışıyor


def test_rotor_inertia_increases_effective_mass_consistently():
    from pendulum_lab.model.linearize import numerical_linearize
    from pendulum_lab.model.params import MotorParams
    p = PendulumParams(motor=MotorParams(J_rotor=2e-6))
    assert p.M_eff == pytest.approx(p.M + 2e-6 * 10 ** 2 / 0.0127 ** 2)
    A, B = linearize(p)
    An, Bn = numerical_linearize(p)
    np.testing.assert_allclose(A, An, rtol=1e-5, atol=1e-6)
    np.testing.assert_allclose(B, Bn, rtol=1e-5, atol=1e-6)


def test_u_max_cannot_exceed_supply_in_voltage_mode():
    with pytest.raises(ValueError):
        PendulumParams(u_max=24.0)               # motor V_max = 12 V
    PendulumParams(u_max=24.0, input_mode="force")


def test_simulator_end_stops_keep_cart_on_rail(p):
    """FAULT sonrası serbest kalan araba rayı aşamaz (eskiden 0.434 m'ye gidiyordu)."""
    K, _, _ = design_lqr(p, Q, R, Ts=0.002)
    sup = Supervisor(p, K)
    sup.mode = Mode.BALANCE
    r = simulate_closed_loop(p, sup, [0.3, 1.5, 0, 0], 3.0)
    assert "FAULT" in r.mode
    assert np.abs(r.x).max() <= p.rail_half + 1e-12


def test_discrete_step_matches_continuous_for_small_Ts(p):
    K, _, _ = design_lqr(p, Q, R, Ts=0.002)
    ss = StateSpace.from_params(p)
    t, y = ss.closed_loop_step_discrete(K, 0.002, T=8.0)
    info = step_info(t, y[:, 0])
    assert info["final"] == pytest.approx(1.0, abs=1e-3)
    assert info["undershoot_pct"] > 0
