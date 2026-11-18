"""Parametre ekranındaki yeni alanlar (Issue #5): ẋ₀, θ̇₀, dürtme süresi ve g."""

import math
import os

import numpy as np
import pytest
from PyQt5 import QtWidgets

from gui.analysis_tab import SimulationPage, pulse_disturbance
from gui.design_tab import DesignTab
from gui.state import AppState


@pytest.fixture(scope="module")
def qapp():
    os.environ["QT_QPA_PLATFORM"] = "offscreen"
    app = QtWidgets.QApplication.instance()
    if app is None:
        app = QtWidgets.QApplication(["test"])
    yield app


def test_pulse_disturbance_duration():
    assert pulse_disturbance(1.0, 0.0, 0.05) is None          # kuvvet yoksa bozucu da yok
    d = pulse_disturbance(1.0, 5.0, 0.2)
    assert d(0.99) == 0.0
    assert d(1.0) == 5.0
    assert d(1.19) == 5.0                                      # 50 ms sınırı artık sabit değil
    assert d(1.2) == 0.0


def test_simulation_uses_initial_velocities(qapp):
    page = SimulationPage(AppState("config/params.json"))
    page.T.setValue(0.5)
    page.real_cb.setChecked(False)
    page.dx0.setValue(0.3)
    page.dth0.setValue(90.0)
    page.run()
    s0 = page.result.s[0]
    assert s0[1] == pytest.approx(0.3)
    assert s0[3] == pytest.approx(math.radians(90.0))


def test_longer_disturbance_pushes_cart_further(qapp):
    def final_x(duration_ms):
        page = SimulationPage(AppState("config/params.json"))
        page.th0.setValue(0.0)                                 # dik başla, LQR dengede tutsun
        page.T.setValue(1.0)
        page.real_cb.setChecked(False)
        page.dist_t.setValue(0.1)
        page.dist_F.setValue(2.0)
        page.dist_dur.setValue(duration_ms)
        page.run()
        return np.max(np.abs(page.result.x))

    assert final_x(200) > final_x(20)


def test_design_tab_g_roundtrip(qapp):
    st = AppState("config/params.json")
    tab = DesignTab(st)
    assert tab.w_par["g"].value() == pytest.approx(st.p.g)
    tab.w_par["g"].setValue(3.71)                              # Mars :)
    assert tab._params_from_widgets().g == pytest.approx(3.71)


# ------------------------------------------------------------------ 2. adım: E_ratio ve LQR'a giriş sınırları
from pathlib import Path  # noqa: E402
import re  # noqa: E402

from gui.live_tab import LiveTab  # noqa: E402
from pendulum_lab.comm import protocol as P  # noqa: E402
from pendulum_lab.comm.mock_device import MockDevice  # noqa: E402
from pendulum_lab.control.supervisor import Mode, Supervisor  # noqa: E402
from pendulum_lab.control.swingup import EnergySwingUp  # noqa: E402
from pendulum_lab.export import gains_header  # noqa: E402
from pendulum_lab.model.params import PendulumParams  # noqa: E402

FIRMWARE = Path(__file__).resolve().parents[1] / "firmware" / "esp32_pendulum" / "esp32_pendulum.ino"


def test_energy_ratio_sets_target_energy():
    p = PendulumParams()
    mgl = p.m * p.g * p.l
    assert EnergySwingUp().target_energy(p) == pytest.approx(0.0)          # varsayılan: dik konum
    assert EnergySwingUp(E_ratio=1.1).target_energy(p) == pytest.approx(0.2 * mgl)
    # dik ve yavaşça dönerken (E ≈ 0): oran 1 ise enerji terimi sıfır, oran > 1 ise sarkaç hâlâ itilir
    s = [0.0, 0.0, 0.0, 0.5]
    assert abs(EnergySwingUp(E_ratio=1.0, k_E=1.0).accel(p, s)) < 0.01
    assert abs(EnergySwingUp(E_ratio=1.2, k_E=1.0).accel(p, s)) > 0.01


def test_catch_requires_cart_position_and_speed_limits():
    p = PendulumParams()
    near_top = [0.0, 0.0, math.radians(5), 0.0]
    sup = Supervisor(p, [0, 0, 0, 0])
    assert sup.can_catch(near_top)                                         # varsayılanlar kısıtlamaz
    assert not Supervisor(p, [0, 0, 0, 0], catch_x=0.1).can_catch([0.2, 0.0, math.radians(5), 0.0])
    assert not Supervisor(p, [0, 0, 0, 0], catch_dx=0.5).can_catch([0.0, 1.0, math.radians(5), 0.0])
    assert not Supervisor(p, [0, 0, 0, 0], catch_dth=1.0).can_catch([0.0, 0.0, math.radians(5), 2.0])

    sup = Supervisor(p, [0, 0, 0, 0], catch_x=0.1)
    sup.start()
    sup.update([0.2, 0.0, math.radians(5), 0.0])
    assert sup.mode == Mode.SWING_UP                                       # araba uzakta: LQR'a geçmez
    sup.update([0.05, 0.0, math.radians(5), 0.0])
    assert sup.mode == Mode.BALANCE


def _device():
    out = []
    dev = MockDevice(seed=1, realtime=False)
    dev.output = out.append
    dev.start()
    return dev, out


def _acks(out, cmd):
    return [m for m in map(P.parse_line, out) if isinstance(m, P.Ack) and m.cmd == cmd]


def test_mock_device_applies_new_params():
    dev, out = _device()
    dev.feed(P.encode_command("set_param", catch_dth=3.0, catch_x=0.2, catch_dx=0.8, E_ratio=1.05))
    dev.step()
    assert _acks(out, "set_param")[-1].ok
    assert (dev.sup.catch_dth, dev.sup.catch_x, dev.sup.catch_dx) == (3.0, 0.2, 0.8)
    assert dev.sup.swingup.E_ratio == 1.05
    with pytest.raises(P.ProtocolError):
        P.encode_command("set_param", E_ratio=3.0)                         # aralık dışı gönderilmez


def test_app_state_pushes_new_params_to_device():
    st = AppState("config/params.json")
    st.catch_dth, st.catch_x, st.catch_dx = 4.0, 0.25, 1.0
    st.swingup.E_ratio = 1.02
    sent = []
    st.send = lambda cmd, **f: sent.append((cmd, f)) or True
    assert st.push_to_device()
    params = {k: v for cmd, f in sent if cmd == "set_param" for k, v in f.items()}
    assert params["catch_dth"] == 4.0 and params["catch_x"] == 0.25 and params["catch_dx"] == 1.0
    assert params["E_ratio"] == 1.02
    meta = st.get_metadata()
    assert meta["catch_dth"] == 4.0 and meta["swingup"]["E_ratio"] == 1.02


def test_design_tab_new_fields_reach_state(qapp):
    st = AppState("config/params.json")
    tab = DesignTab(st)
    tab.w_su["E_ratio"].setValue(1.1)
    tab.w_catch_dth.setValue(3.5)
    tab.w_catch_x.setValue(0.2)
    tab.w_catch_dx.setValue(0.9)
    tab.calculate()
    assert st.swingup.E_ratio == pytest.approx(1.1)
    assert (st.catch_dth, st.catch_x, st.catch_dx) == pytest.approx((3.5, 0.2, 0.9))


def test_gains_header_and_firmware_use_new_params():
    h = gains_header(PendulumParams(), [0, 0, 0, 0], 0.002, swingup=EnergySwingUp(E_ratio=1.1),
                     catch_dth=3.0, catch_x=0.25, catch_dx=1.5)
    for line in ("#define CATCH_DTH          3.0000f", "#define CATCH_X            0.2500f",
                 "#define CATCH_DX           1.5000f", "#define SU_E_RATIO         1.1000f"):
        assert line in h
    src = FIRMWARE.read_text(encoding="utf-8")
    assert "fabsf(dth) < 5.0f" not in src                                  # sabit sınır kaldırıldı
    assert re.search(r"fabsf\(dth\) < p\.catch_dth\s*&& fabsf\(x\) < p\.catch_x && fabsf\(dx\) < p\.catch_dx", src)
    assert "(E - E_target)" in src


# ------------------------------------------------------------------ 3. adım: kuvvet ve süreyle dürtme
def test_mock_kick_with_force_and_duration():
    dev, out = _device()
    p = dev.p
    dev.s = [dev.s[0], 0.0, math.pi, 0.0]                                  # aşağıda, hareketsiz
    F, dur = 2.0, 0.02
    dev.feed(P.encode_command("kick", F=F, dur=dur))
    for _ in range(int(dur / dev.Ts)):
        dev.step()
    assert _acks(out, "kick")[-1].ok
    expected = F * p.l * math.cos(math.pi) / p.J * dur                     # simulator ile aynı model
    assert dev.s[3] == pytest.approx(expected, rel=0.2)
    dev.feed(P.encode_command("kick", F=50.0, dur=0.1))
    dev.step()
    assert not _acks(out, "kick")[-1].ok                                   # kuvvet aralık dışı


def test_live_tab_kick_sends_force_and_duration(qapp):
    st = AppState("config/params.json")
    tab = LiveTab(st)
    sent = []
    st.send = lambda cmd, **f: sent.append((cmd, f)) or True
    tab.kick_F.setValue(-3.0)
    tab.kick_dur.setValue(200.0)
    tab.kick()
    assert sent == [("kick", {"F": -3.0, "dur": pytest.approx(0.2)})]
