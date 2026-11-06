"""Homing (KALİBRE ET) sürtünme tahmini yanlışken de bitmeli (#30): PI hız döngüsü, Coulomb telafisi yok."""

import copy
import math
import re
from pathlib import Path

import pytest

from pendulum_lab import PendulumParams, accel_to_input
from pendulum_lab.comm import mock_device as MD
from pendulum_lab.comm import protocol as P
from pendulum_lab.comm.mock_device import MockDevice
from pendulum_lab.control.supervisor import Mode

FIRMWARE = Path(__file__).resolve().parents[1] / "firmware" / "esp32_pendulum" / "esp32_pendulum.ino"
TF = 0.0018   # 37D tahmini T_f ≈ Kt·I₀ [N·m] → yer tutucu kasnakla arabada ≈ 1.4 N


def _home(tf_gercek, tf_tahmin, x0):
    """Fiziğinde tf_gercek, kontrolcüsünde tf_tahmin sürtünmesi olan sanal cihazda KALİBRE ET."""
    p = PendulumParams()
    p.motor.T_f = tf_gercek
    out = []
    dev = MockDevice(params=p, seed=4, realtime=False, x0=x0)
    dev.output = out.append
    dev.start()
    dev.sup.p = copy.deepcopy(p)
    dev.sup.p.motor.T_f = tf_tahmin
    dev.feed(P.encode_command("home"))
    for _ in range(int((MD.HOME_TIMEOUT + 1) / dev.Ts)):
        dev.step()
        if dev.homed or dev.sup.mode == Mode.FAULT:
            break
    return dev, [o for o in out if b'"evt":"fault"' in o]


@pytest.mark.parametrize("tf_gercek, tf_tahmin", [
    (TF, 0.0),          # sürtünme var, henüz ölçülmemiş (bugünkü varsayılan)
    (TF, TF / 2),       # eksik tahmin
    (TF / 2, TF),       # 2 kat fazla tahmin
    (0.0, TF),          # sürtünme yok ama telafi açık
    (3 * TF, 0.0),      # tahminden çok yüksek sürtünme
])
@pytest.mark.parametrize("x0", [-0.10, 0.15])
def test_homing_finishes_with_wrong_friction_estimate(tf_gercek, tf_tahmin, x0):
    dev, faults = _home(tf_gercek, tf_tahmin, x0)
    assert not faults, faults
    assert dev.homed and dev.sup.mode == Mode.IDLE
    assert abs(dev.true_state[0]) < 0.005                          # araba gerçekten ortada


def test_accel_to_input_without_coulomb_ignores_friction_estimate():
    s = [0.0, 0.0, math.pi, 0.0]
    p0, pf = PendulumParams(), PendulumParams()
    pf.motor.T_f = TF
    assert accel_to_input(pf, s, 1.0, coulomb=False) == accel_to_input(p0, s, 1.0)
    assert accel_to_input(pf, s, 1.0) > accel_to_input(p0, s, 1.0)  # swing-up'ta telafi sürüyor


def test_firmware_homing_matches_python():
    src = FIRMWARE.read_text(encoding="utf-8")
    for name in ("HOME_SPEED", "HOME_KP", "HOME_KI", "HOME_I_MAX"):
        m = re.search(rf"static const float {name} = ([\d.]+)f;", src)
        assert m, name
        assert float(m.group(1)) == getattr(MD, name), name
    assert "home_i = clampf(home_i + HOME_KI * e * dt, -HOME_I_MAX, HOME_I_MAX);" in src
    assert "accelToVolt(dx, th, dth, HOME_KP * e + home_i, false)" in src          # homing: telafi yok
    assert "accelToVolt(dx, th, dth, swingUpAccel(p, x, dx, th, dth), true)" in src  # swing-up: telafi var
