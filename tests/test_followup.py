"""İkinci (doğrulama) incelemesinde bulunan sorunların testleri."""

import json
import math
from pathlib import Path

import numpy as np

from pendulum_lab import PendulumParams, Supervisor, Mode, simulate_closed_loop
from pendulum_lab.comm import protocol as P
from pendulum_lab.comm.mock_device import MockDevice
from pendulum_lab.control.lqr import design_lqr
from pendulum_lab.export import gains_header
from pendulum_lab.model.params import MotorParams


def _offline(**kw):
    out = []
    dev = MockDevice(seed=4, realtime=False, **kw)
    dev.output = out.append
    dev.start()
    return dev, out


def test_homing_recovers_when_cart_rests_on_right_switch():
    """Sağ limit hatasından sonra araba sağ switch üzerinde dururken FAULT → home kurtarması çalışmalı."""
    dev, out = _offline(x0=0.4)
    dev.feed(P.encode_command("home"))
    for _ in range(int(20 / dev.Ts)):
        dev.step()
        if dev.homed:
            break
    assert dev.homed and abs(dev.true_state[0]) < 0.01
    assert not [o for o in out if b"homing_wrong_side" in o]


def test_gains_header_exports_supply_voltage():
    p = PendulumParams(motor=MotorParams(V_max=24.0), u_max=24.0)
    assert "#define MOTOR_V_MAX        24.000f" in gains_header(p, [0, 0, 0, 0], 0.002)


def test_simulator_coasts_in_fault_like_firmware():
    """FAULT'ta sürücü kapalı: araba 0 V'taki zıt EMK freni olmadan serbest kayar (firmware ve mock gibi)."""
    p = PendulumParams(rail_half=5.0)                # uç durdurucuya çarpmasın, kayma mesafesini ölç
    K, _, _ = design_lqr(p, [10, 1, 100, 1], [[0.1]], Ts=0.002)
    sup = Supervisor(p, K)
    sup.mode = Mode.FAULT
    r = simulate_closed_loop(p, sup, [0, 1.0, math.pi, 0], 1.0)
    # yalnız b=0.1 N·s/m sürtünme: 1 s'de hız neredeyse hiç azalmaz (0 V fren olsaydı ~0'a inerdi)
    assert r.s[-1, 1] > 0.7
    assert r.u.max() == 0.0                          # bildirilen giriş 0


def test_app_starts_with_invalid_params_file(tmp_path):
    from gui.state import AppState
    bad = tmp_path / "config" / "params.json"
    bad.parent.mkdir()
    bad.write_text(json.dumps({"u_max": 99.0}), encoding="utf-8")   # u_max > V_max → geçersiz
    st = AppState(bad)
    assert st.startup_warning and st.p == PendulumParams()
    assert np.all(np.isfinite(st.K))


def test_xref_clamped_to_rail():
    from gui.state import AppState
    st = AppState("yok/params.json")
    st.p = PendulumParams(rail_half=0.35)
    assert st.clamp_xref(0.3) == 0.25 or abs(st.clamp_xref(0.3) - 0.25) < 1e-12


# ------------------------------------------------------------------ yer tutucu parametreler / kayıt kaynağı
class _FakeReader:
    """Gerçek karta (SimTransport olmayan) bağlı okuyucu yerine geçer; gönderilen komutları saklar."""
    def __init__(self, transport):
        self.transport = transport
        self.sent = []

    def send(self, cmd, **fields):
        self.sent.append(cmd)


def _state_on(transport, placeholder):
    from gui.state import AppState
    st = AppState("yok/params.json")
    st.p = PendulumParams(placeholder=placeholder)
    st.set_reader(_FakeReader(transport))
    return st


def test_placeholder_params_not_pushed_to_real_device_without_force():
    class Serial:
        name = "COM3 @ 921600"
    st = _state_on(Serial(), placeholder=True)
    assert st.placeholder_on_real_device
    st.on_device_info({"Ts": 0.002})
    assert st.reader.sent == []                                   # bağlanınca otomatik gönderilmez
    assert st.push_to_device() is False and st.reader.sent == []
    assert st.push_to_device(force=True) and "set_gain" in st.reader.sent   # kullanıcı onaylarsa gider


def test_measured_params_and_simulator_push_automatically():
    from pendulum_lab.comm.mock_device import MockDevice
    from pendulum_lab.comm.transport import SimTransport

    class Serial:
        name = "COM3 @ 921600"
    real = _state_on(Serial(), placeholder=False)
    real.on_device_info({"Ts": 0.002})
    assert "set_gain" in real.reader.sent
    sim = _state_on(SimTransport(MockDevice(realtime=False)), placeholder=True)
    assert not sim.real_device and not sim.placeholder_on_real_device
    sim.on_device_info({"Ts": 0.002})
    assert "set_gain" in sim.reader.sent                          # sanal ESP32'de davranış değişmedi


def test_config_params_marked_placeholder_and_gains_h_warns():
    from pendulum_lab.model.params import load_params
    root = Path(__file__).resolve().parents[1]
    assert load_params(root / "config" / "params.json").placeholder
    assert "#warning" in gains_header(PendulumParams(placeholder=True), [0, 0, 0, 0], 0.002)
    assert "#warning" not in gains_header(PendulumParams(placeholder=False), [0, 0, 0, 0], 0.002)
    assert "#warning" in (root / "firmware" / "esp32_pendulum" / "gains.h").read_text(encoding="utf-8")


def test_recorder_path_prefix_marks_source(tmp_path):
    from pendulum_lab.recorder import Recorder
    assert Recorder.default_path(tmp_path, prefix="sim").name.startswith("sim_")
    assert Recorder.default_path(tmp_path, prefix="gercek").name.startswith("gercek_")
    assert Recorder.default_path(tmp_path).name.startswith("oturum_")
