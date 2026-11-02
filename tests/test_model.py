import math

import numpy as np
import pytest

from pendulum_lab import PendulumParams, linearize, numerical_jacobian, dynamics, accel_to_input
from pendulum_lab.model.dynamics import accelerations, energy
from pendulum_lab.export import gains_header
from pendulum_lab.model.params import MotorParams, pendulum_from_geometry, bob_inertia, load_params, save_params
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


# ------------------------------------------------------------------ motor sürtünmesi (T_f, B_m)
def test_motor_friction_reflected_to_cart():
    mot = MotorParams(T_f=0.002, B_m=2e-6)               # redüktör 10:1, kasnak r = 0.0127 m
    p = PendulumParams(motor=mot)
    assert p.F_coulomb == pytest.approx(0.002 * 10 / 0.0127)
    assert p.b_cart == pytest.approx(p.b + 2e-6 * 10 ** 2 / 0.0127 ** 2)
    pf = PendulumParams(input_mode="force", motor=mot)   # motor modeli yok → etkisiz
    assert pf.F_coulomb == 0.0 and pf.b_cart == pf.b
    with pytest.raises(ValueError):
        MotorParams(T_f=-0.001)


def test_motor_viscous_friction_enters_linear_model():
    p0 = PendulumParams(I=0.002)
    p = PendulumParams(I=0.002, motor=MotorParams(B_m=5e-6))
    A0, _ = linearize(p0)
    A, B = linearize(p)
    assert A[1, 1] < A0[1, 1]                             # araba daha sönümlü
    An, Bn = numerical_linearize(p)
    np.testing.assert_allclose(A, An, rtol=1e-5, atol=1e-6)
    np.testing.assert_allclose(B, Bn, rtol=1e-5, atol=1e-6)


def test_coulomb_friction_stops_coasting_cart():
    """Sürücü boşta (zıt EMK etkisi kaldırılmış) ve b = 0: araba yalnız Coulomb sürtünmesiyle durur."""
    v0 = 0.3
    p = PendulumParams(b=0.0, c=0.0, motor=MotorParams(T_f=0.002))
    coast = lambda t, s: p.motor.beta * s[1] / p.motor.alpha    # noqa: E731
    r = simulate_open_loop(p, [0, v0, math.pi, 0], 1.0, u_func=coast)
    a = p.F_coulomb / (p.M_eff + p.m)
    assert r.y[0, -1] == pytest.approx(v0 ** 2 / (2 * a), rel=0.3)   # durma mesafesi v0²/2a
    assert abs(r.y[1, -1]) < 0.01
    free = PendulumParams(b=0.0, c=0.0)                  # sürtünmesiz: araba gitmeye devam eder
    rf = simulate_open_loop(free, [0, v0, math.pi, 0], 1.0, u_func=coast)
    assert rf.y[0, -1] > 5 * r.y[0, -1]


def test_gains_header_cart_friction_includes_motor_viscous():
    p = PendulumParams(motor=MotorParams(B_m=5e-6))
    h = gains_header(p, [0, 0, 0, 0], 0.002)
    assert f"#define P_B                {p.b_cart:.6f}f" in h and p.b_cart > p.b


def test_coulomb_compensation_direction():
    from pendulum_lab.model.dynamics import V_COMP, coulomb_compensation
    p = PendulumParams(motor=MotorParams(T_f=0.002))
    Fc = p.F_coulomb
    assert coulomb_compensation(p, 0.2, -3.0) == Fc               # hareket ediyor: hız yönünde
    assert coulomb_compensation(p, -0.2, 3.0) == -Fc
    assert coulomb_compensation(p, V_COMP / 2, -3.0) == -Fc       # duruyor: istenen ivme yönünde
    assert coulomb_compensation(p, 0.0, 0.0) == 0.0
    assert coulomb_compensation(PendulumParams(), 0.2, 1.0) == 0.0   # T_f = 0 → telafi yok
    h = gains_header(p, [0, 0, 0, 0], 0.002)
    assert f"#define P_FC               {Fc:.6f}f" in h


def test_firmware_coulomb_compensation_matches_python():
    """Firmware'deki accelToVolt telafisi Python'daki coulomb_compensation ile aynı kalmalı (README'ye bakın)."""
    import re
    from pathlib import Path
    from pendulum_lab.model.dynamics import V_COMP
    src = (Path(__file__).resolve().parents[1] / "firmware" / "esp32_pendulum" / "esp32_pendulum.ino").read_text(
        encoding="utf-8")
    m = re.search(r"FC_V_COMP\s*=\s*([\d.]+)f", src)
    assert m and float(m.group(1)) == V_COMP
    body = src[src.index("static float accelToVolt"):src.index("static float swingUpAccel")]
    assert "fabsf(dx) > FC_V_COMP ? dx : a" in body
    assert "F += fc_N" in body and "F -= fc_N" in body
    assert "static float fc_N = P_FC;" in src                      # başta gains.h'deki telafi
    assert "fc_N = p.fc_volt * MOTOR_ALPHA;" in src                # sürtünme ölçümü / set_param fc_volt
    assert "#define P_FC 0.0f" in src                              # eski gains.h ile de derlenir
