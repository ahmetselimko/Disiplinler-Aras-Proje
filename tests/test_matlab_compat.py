"""Matlab/Simulink modeli ile Python kütüphanesinin karşılaştırması (issue #10).

Matlab'ın A, B matrisleri burada kütüphaneden BAĞIMSIZ olarak, cart_pendulum_lqr.m'deki
kapalı formüllerle kurulur. Matlab'da θ, sarkaç SOLA yatınca pozitiftir; projede SAĞA.
Bkz. pendulum_lab/sim/matlab_compat.py
"""

import math

import numpy as np
import pytest

from pendulum_lab import linearize, lqr
from pendulum_lab.control.lqr import design_lqr
from pendulum_lab.sim.matlab_compat import (
    MATLAB_F_MAX, MATLAB_K_REFERENCE, MATLAB_Q, MATLAB_R, MATLAB_SIGN, gain_from_matlab, gain_to_matlab,
    matlab_reference_params, simulate_matlab_compatible, state_from_matlab, state_to_matlab,
)
from pendulum_lab.sim.simulator import simulate_closed_loop


def matlab_AB(M, m, l, I, b, g):
    """cart_pendulum_lqr.m'deki durum uzayı modeli (Matlab'ın θ işaretinde: sola +)."""
    p = I * (M + m) + M * m * l ** 2
    A = np.array([[0, 1, 0, 0],
                  [0, -(I + m * l ** 2) * b / p, (m ** 2 * g * l ** 2) / p, 0],
                  [0, 0, 0, 1],
                  [0, -(m * l * b) / p, m * g * l * (M + m) / p, 0]])
    B = np.array([[0], [(I + m * l ** 2) / p], [0], [m * l / p]])
    return A, B


@pytest.fixture
def p():
    return matlab_reference_params()


def test_matlab_K_is_reproduced_from_independent_model(p):
    """Matlab'ın formülleriyle kurulan modelden çıkan K, Matlab'ın verdiği K ile aynı."""
    A_m, B_m = matlab_AB(p.M, p.m, p.l, p.I, p.b, p.g)
    K_m, _, _ = lqr(A_m, B_m, MATLAB_Q, MATLAB_R)
    np.testing.assert_allclose(K_m.ravel(), MATLAB_K_REFERENCE, atol=0.01)


def test_python_model_equals_matlab_model_up_to_theta_sign(p):
    """Kütüphanenin A, B'si Matlab'ınkiyle aynı; tek fark θ işareti (A_m = T·A·T, B_m = T·B)."""
    A, B = linearize(p)
    A_m, B_m = matlab_AB(p.M, p.m, p.l, p.I, p.b, p.g)
    T = MATLAB_SIGN
    np.testing.assert_allclose(T @ A @ T, A_m, rtol=1e-12, atol=1e-12)
    np.testing.assert_allclose(T @ B, B_m, rtol=1e-12, atol=1e-12)


def test_python_K_matches_matlab_K_with_theta_gains_flipped(p):
    """Projenin K'sı = Matlab K'sı, θ ve θ̇ kazançlarının işareti ters (issue #1'deki fark)."""
    K, _, _ = design_lqr(p, MATLAB_Q, MATLAB_R)
    K = K.ravel()
    np.testing.assert_allclose(K, [-50.0, -45.37, -168.66, -33.25], atol=0.01)
    np.testing.assert_allclose(gain_to_matlab(K), MATLAB_K_REFERENCE, atol=0.01)
    np.testing.assert_allclose(gain_from_matlab(MATLAB_K_REFERENCE), K, atol=0.01)


def test_using_matlab_K_without_conversion_drops_the_pendulum(p):
    """Riskin kendisi: Matlab K'sı dönüştürülmeden projede kullanılırsa kapalı çevrim kararsız."""
    A, B = linearize(p)
    K_raw = np.array(MATLAB_K_REFERENCE).reshape(1, 4)
    assert np.max(np.linalg.eigvals(A - B @ K_raw).real) > 0
    K_ok = gain_from_matlab(MATLAB_K_REFERENCE).reshape(1, 4)
    assert np.max(np.linalg.eigvals(A - B @ K_ok).real) < 0
    r = simulate_matlab_compatible(p, MATLAB_K_REFERENCE, [0, 0, math.radians(5), 0], T=5)
    assert r.stop_reason in ("angle", "rail")


def test_sign_conversions_are_their_own_inverse():
    s = np.array([0.1, -0.2, 0.3, -0.4])
    np.testing.assert_allclose(state_to_matlab(s), [0.1, -0.2, -0.3, 0.4])
    np.testing.assert_allclose(state_from_matlab(state_to_matlab(s)), s)
    K = np.array([1.0, 2.0, 3.0, 4.0])
    np.testing.assert_allclose(gain_from_matlab(gain_to_matlab(K)), K)


def test_closed_loop_poles_match_in_both_conventions(p):
    """İki tarafın kendi modeli ve kendi K'sıyla kapalı çevrim kutupları aynı (fiziksel olarak aynı kontrolcü)."""
    _, _, cl = design_lqr(p, MATLAB_Q, MATLAB_R)
    A_m, B_m = matlab_AB(p.M, p.m, p.l, p.I, p.b, p.g)
    _, _, cl_m = lqr(A_m, B_m, MATLAB_Q, MATLAB_R)   # Matlab K'sının yuvarlanmamış hali
    np.testing.assert_allclose(np.sort_complex(cl), np.sort_complex(cl_m), rtol=1e-9)


# ---- Matlab uyumlu deneme (cart_pendulum_application.m) ----

def test_matlab_compatible_sim_balances_and_returns_to_center(p):
    K = gain_from_matlab(MATLAB_K_REFERENCE)
    r = simulate_matlab_compatible(p, K, [0.05, 0, math.radians(10), 0], T=8)
    assert r.stop_reason == "time"
    assert r.t[-1] == pytest.approx(8.0)
    assert np.diff(r.t) == pytest.approx(0.0025)
    assert abs(r.th[-1]) < math.radians(0.1) and abs(r.x[-1]) < 1e-3
    assert np.all(np.abs(r.x) <= 0.2)


def test_matlab_compatible_sim_stops_at_angle_limit(p):
    r = simulate_matlab_compatible(p, np.zeros(4), [0, 0, math.radians(5), 0], T=10)
    assert r.stop_reason == "angle"
    assert abs(r.th[-1]) >= math.pi / 2 and np.all(np.abs(r.th[:-1]) < math.pi / 2)
    assert r.t[-1] < 10
    assert r.mode[-1] == "FAULT" and r.mode[0] == "BALANCE"


def test_matlab_compatible_sim_stops_at_rail_limit(p):
    """Büyük konum hatasıyla başlatılan araba ±0.2 m'ye ulaşınca deneme durur (Matlab: abs(z(1)) >= railLimit)."""
    K = gain_from_matlab(MATLAB_K_REFERENCE)
    r = simulate_matlab_compatible(p, K, [0.0, 0, 0, 0], T=10, x_ref=0.5)
    assert r.stop_reason == "rail"
    assert abs(r.x[-1]) >= 0.2 and np.all(np.abs(r.x[:-1]) < 0.2)


def test_matlab_compatible_sim_agrees_with_library_simulator(p):
    """Aynı K ile kütüphanenin 500 Hz ayrık simülasyonu ve Matlab uyumlu sürekli simülasyon
    hemen hemen aynı yörüngeyi verir (Ts = 2 ms küçük olduğu için)."""
    K = gain_from_matlab(MATLAB_K_REFERENCE)
    s0 = [0.05, 0, math.radians(8), 0]
    r_m = simulate_matlab_compatible(p, K, s0, T=4, f_max=None)   # kütüphane simülasyonunda kuvvet sınırı yok
    r_p = simulate_closed_loop(p, lambda s: -float(K @ np.asarray(s)), s0, T=4, dt=0.002, end_stops=False)
    for t_check in (0.5, 1.0, 2.0, 4.0):
        i_m = int(np.argmin(np.abs(r_m.t - t_check)))
        i_p = int(np.argmin(np.abs(r_p.t - t_check)))
        assert r_m.th[i_m] == pytest.approx(r_p.th[i_p], abs=math.radians(0.3))
        assert r_m.x[i_m] == pytest.approx(r_p.x[i_p], abs=3e-3)


def test_matlab_compatible_sim_ignores_motor_mode(p):
    """Parametreler gerilim girişli olsa da deneme kuvvet girişiyle yapılır (Matlab gibi)."""
    import copy
    pv = copy.deepcopy(p)
    pv.input_mode = "voltage"
    pv.u_max = 12.0
    K = gain_from_matlab(MATLAB_K_REFERENCE)
    s0 = [0, 0, math.radians(5), 0]
    a = simulate_matlab_compatible(p, K, s0, T=2)
    b = simulate_matlab_compatible(pv, K, s0, T=2)
    np.testing.assert_allclose(a.s, b.s)


def test_matlab_compatible_sim_rejects_bad_K(p):
    with pytest.raises(ValueError):
        simulate_matlab_compatible(p, [1, 2, 3], [0, 0, 0, 0])
    with pytest.raises(ValueError):
        simulate_matlab_compatible(p, [1, 2, float("nan"), 4], [0, 0, 0, 0])


def matlab_nonlinear_accel(p, z, F):
    """cart_pendulum_application.m / Simulink "Nonlinear Dynamics" fonksiyonu (Matlab işaretinde: θ sola +)."""
    J = p.I + p.m * p.l ** 2
    h = p.m * p.l
    th, om = z[2], z[3]
    H = np.array([[p.M + p.m, -h * math.cos(th)], [-h * math.cos(th), J]])
    rhs = np.array([F - p.b * z[1] - h * math.sin(th) * om ** 2, h * p.g * math.sin(th)])
    return np.linalg.solve(H, rhs)


def test_python_nonlinear_model_equals_matlab_up_to_theta_sign(p):
    """Doğrusal olmayan model de Matlab'ınkiyle aynı (θ işareti dışında), geniş açılarda da."""
    from pendulum_lab.model.dynamics import accelerations
    rng = np.random.default_rng(0)
    for _ in range(500):
        s = rng.uniform([-0.2, -2, -1.5, -6], [0.2, 2, 1.5, 6])
        F = rng.uniform(-20, 20)
        ddx, ddth = accelerations(p, s, F)
        a_m = matlab_nonlinear_accel(p, state_to_matlab(s), F)
        assert ddx == pytest.approx(a_m[0], abs=1e-9)
        assert -ddth == pytest.approx(a_m[1], abs=1e-9)


def test_matlab_Q_matches_scales_and_force_is_limited(p):
    """Q = diag(1/[0.2 0.5 deg2rad(10) 1].^2), R = 1/Fmax^2; kontrol kuvveti ±Fmax ile sınırlı (Matlab gibi)."""
    np.testing.assert_allclose(MATLAB_Q, 1 / np.array([0.2, 0.5, math.radians(10), 1.0]) ** 2)
    assert MATLAB_R == pytest.approx(1 / MATLAB_F_MAX ** 2)
    K = gain_from_matlab(MATLAB_K_REFERENCE)
    s0 = [0, 0, math.radians(10), 0]                       # cart_pendulum_lqr.m'deki z0
    r = simulate_matlab_compatible(p, K, s0, T=10)
    assert np.max(np.abs(r.u)) == pytest.approx(MATLAB_F_MAX)
    assert r.stop_reason == "time" and abs(r.th[-1]) < math.radians(0.1)
    r_free = simulate_matlab_compatible(p, K, s0, T=10, f_max=None)
    assert np.max(np.abs(r_free.u)) > 2 * MATLAB_F_MAX     # sınırsız olsaydı ~29 N isterdi
