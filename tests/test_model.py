import math

import numpy as np
import pytest

from pendulum_lab import PendulumParams, linearize, numerical_jacobian, dynamics, accel_to_input
from pendulum_lab.model.dynamics import accelerations, energy
from pendulum_lab.model.params import pendulum_from_geometry, bob_inertia, load_params, save_params
from pendulum_lab.model.linearize import numerical_linearize
from pendulum_lab.sim.simulator import simulate_open_loop


@pytest.mark.parametrize("mode", ["force", "voltage", "acceleration"])
def test_analytic_linearization_matches_numeric_jacobian(mode):
    p = PendulumParams(input_mode=mode, I=0.002, c=0.001)
    A, B = linearize(p)
    An, Bn = numerical_linearize(p)
    np.testing.assert_allclose(A, An, rtol=1e-5, atol=1e-6)
    np.testing.assert_allclose(B, Bn, rtol=1e-5, atol=1e-6)


def test_upright_is_unstable_and_hanging_is_stable():
    p = PendulumParams(input_mode="force")
    A, _ = linearize(p)
    assert np.max(np.linalg.eigvals(A).real) > 0
    Ah, _ = numerical_linearize(p, s0=(0, 0, math.pi, 0))
    assert np.max(np.linalg.eigvals(Ah).real) <= 1e-9


def test_sign_convention_push_right_tips_pendulum_back():
    p = PendulumParams(input_mode="force")
    ddx, ddth = accelerations(p, [0, 0, 0, 0], 1.0)
    assert ddx > 0 and ddth < 0
    _, ddth_g = accelerations(p, [0, 0, 0.1, 0], 0.0)
    assert ddth_g > 0  # θ>0 iken yerçekimi daha da yatırır


def test_energy_conserved_without_friction():
    p = PendulumParams(input_mode="force", b=0.0, c=0.0)
    r = simulate_open_loop(p, [0, 0, 2.0, 0], 3.0)
    E = [energy(p, th, dth) for th, dth in zip(r.y[2], r.y[3])]
    # araba serbest olduğu için sarkaç enerjisi tek başına korunmaz; toplam mekanik enerjiye bak
    ml = p.m * p.l
    total = [0.5 * (p.M + p.m) * dx ** 2 + ml * dx * dth * math.cos(th) + e
             for dx, th, dth, e in zip(r.y[1], r.y[2], r.y[3], E)]
    assert np.ptp(total) < 1e-5


@pytest.mark.parametrize("mode", ["force", "voltage", "acceleration"])
def test_accel_to_input_inverts_dynamics(mode):
    p = PendulumParams(input_mode=mode)
    s = [0.1, 0.3, 1.2, -2.0]
    u = accel_to_input(p, s, 2.5)
    ddx, _ = accelerations(p, s, u)
    assert ddx == pytest.approx(2.5, rel=1e-9)


def test_geometry_sphere_vs_cube():
    # Aynı kütlede, küp kenarı = küre çapı ise küpün kendi ataleti daha büyüktür:
    # küre 2/5 m r^2 = 0.1 m d^2,  küp 1/6 m a^2 ≈ 0.167 m a^2
    assert bob_inertia("cube", 1.0, 0.05) > bob_inertia("sphere", 1.0, 0.025)
    m, l, I = pendulum_from_geometry(rod_mass=0.05, rod_length=0.4, bob_mass=0.15, bob_shape="sphere", bob_size=0.02)
    assert m == pytest.approx(0.2)
    assert 0.3 < l < 0.4 and I > 0


def test_params_roundtrip(tmp_path):
    p = PendulumParams(M=0.7, input_mode="force")
    save_params(p, tmp_path / "p.json")
    q = load_params(tmp_path / "p.json")
    assert q == p


def test_invalid_params_rejected():
    with pytest.raises(ValueError):
        PendulumParams(input_mode="torque")
    with pytest.raises(ValueError):
        PendulumParams(m=0)
