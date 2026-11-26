"""#50 Aşama 4: iki kademeli uç sensörleri (END_SENSORS 2) sanal ESP32'de ve firmware'de aynı kurallarla."""

import re
from pathlib import Path

import pytest

from pendulum_lab import load_params
from pendulum_lab.comm import mock_device as MD
from pendulum_lab.comm import protocol as P
from pendulum_lab.comm.mock_device import MockDevice
from pendulum_lab.control.supervisor import Mode

ROOT = Path(__file__).resolve().parents[1]
FW = (ROOT / "firmware" / "esp32_pendulum" / "esp32_pendulum.ino").read_text(encoding="utf-8")


def _dev(end_sensors=2, x0=0.1, **kw):
    out = []
    dev = MockDevice(load_params(ROOT / "config" / "params.json"), realtime=False, seed=1, x0=x0,
                     end_sensors=end_sensors, **kw)
    dev.output = out.append
    dev.start()
    return dev, out


def _msgs(out):
    return [P.parse_line(o) for o in out]


def _events(out, name):
    return [m.data for m in _msgs(out) if isinstance(m, P.Event) and m.name == name]


def _run(dev, seconds, until=None):
    for _ in range(int(seconds / dev.Ts)):
        dev.step()
        if until and until():
            break


def _home(dev):
    dev.feed(P.encode_command("home"))
    _run(dev, 25, until=lambda: dev.homed or dev.sup.mode == Mode.FAULT)


def test_homing_uses_optical_reference():
    dev, out = _dev()
    _home(dev)
    assert dev.homed and dev.sup.mode == Mode.IDLE
    lim = [e for e in _events(out, "limit")]
    assert lim == [{"side": "L", "stage": "opt"}]                       # mikro switch'e değmedi
    # optiğin konumu x = −rail_half + OPT_FROM_END: sıfırdan sonra gerçek konum ile ölçülen aynı
    assert dev.true_state[0] - dev.x_offset == pytest.approx(dev.true_state[0], abs=0.003)
    assert abs(dev.true_state[0]) < 0.01


def test_optical_in_run_mode_is_limit_fault_and_hard_switch_is_hard_limit():
    dev, out = _dev()
    _home(dev)
    dev.sup.mode = Mode.BALANCE                                         # çalışma modunda arabayı uca sür
    dev.s = [-dev.p.rail_half + MD.OPT_FROM_END - 0.001, -0.3, 0.0, 0.0]
    dev.sup.mode = Mode.WAIT
    dev.step()
    assert dev.sup.mode == Mode.FAULT and _events(out, "fault")[-1]["reason"] == "limit"
    dev, out = _dev()
    _home(dev)
    dev.sup.mode = Mode.WAIT
    dev.s = [-dev.p.rail_half, 0.0, 0.0, 0.0]                           # optiği atlayıp doğrudan uca (ör. hızlı)
    dev.step()
    assert _events(out, "fault")[-1]["reason"] == "hard_limit"
    assert {"side": "L", "stage": "hard"} in _events(out, "limit")


def test_hard_switch_during_homing_is_overrun_and_blocks_commands():
    dev, out = _dev()
    dev.stuck_hard = "L"
    for cmd in ("start", "home", "friction"):                           # basılıyken her modda ret
        dev.feed(P.encode_command(cmd))
        dev.step()
    acks = [m for m in _msgs(out) if isinstance(m, P.Ack)]
    assert [a.ok for a in acks[-3:]] == [False, False, False]
    assert all("sol mikro switch basılı" in a.msg for a in acks[-3:])
    dev.stuck_hard = None
    dev.feed(P.encode_command("home"))
    dev.step()
    dev.stuck_hard = "R"                                                # homing sırasında dış kademe
    dev.step()
    assert dev.sup.mode == Mode.FAULT and _events(out, "fault")[-1]["reason"] == "homing_overrun"


def test_optical_missed_during_homing_gives_overrun_and_mismatch():
    dev, out = _dev()
    dev._at = lambda side, from_end, _orig=dev._at: _orig(side, 0.0)  # optik kaymış: ancak ray sonunda görüyor
    real_ref = dev._ref
    dev._ref = lambda side: False                                       # optik hiç görmüyor (arızalı)
    _home(dev)
    assert dev.sup.mode == Mode.FAULT and _events(out, "fault")[-1]["reason"] == "homing_overrun"
    assert {"side": "L"} in _events(out, "sensor_mismatch")
    assert real_ref is not None


def test_single_sensor_mode_unchanged():
    dev, out = _dev(end_sensors=1)
    _home(dev)
    assert dev.homed and _events(out, "limit") == [{"side": "L"}]       # 1'de stage alanı yok
    with pytest.raises(ValueError):
        MockDevice(end_sensors=3)
    assert MockDevice(limit_switches=False, realtime=False).end_sensors == 0


def test_firmware_matches_mock():
    m = re.search(r"static const float OPT_FROM_END_M = ([\d.]+)f;", FW)
    assert m and float(m.group(1)) == MD.OPT_FROM_END
    assert re.search(r"#ifndef END_SENSORS\s*\n#define END_SENSORS 1", FW)
    assert '#define END_SENSORS LIMIT_SWITCHES' in FW and '#warning "LIMIT_SWITCHES eski ad' in FW
    for reason in ("hard_limit", "homing_overrun", "limit"):
        assert f'"{reason}"' in FW and reason in P.FAULT_REASON_TEXT
    assert 'sensor_mismatch' in FW and "sensor_mismatch" in P.EVENT_TEXT
    assert "mikro switch basılı: arabayı elle uçtan uzaklaştırın" in FW
    assert "mikro switch basılı: arabayı elle uçtan uzaklaştırın" in Path(MD.__file__).read_text(encoding="utf-8")
    pins = (ROOT / "firmware" / "esp32_pendulum" / "board_pins.h").read_text(encoding="utf-8")
    assert pins.count("PIN_OPT_L = ") == 2                              # iki kart
    wf = (ROOT / ".github" / "workflows" / "test.yml").read_text(encoding="utf-8")
    assert "end_sensors: [1, 2]" in wf and "-DEND_SENSORS=${{ matrix.end_sensors }}" in wf


@pytest.mark.parametrize("x0", [-0.38, -0.385, -0.395])
def test_homing_starting_inside_optical_zone_backs_off_first(x0):
    """Araba optik ile uç arasındayken home: önce optikten çekilip kenara yeniden yaklaşılır, sıfır kaymaz."""
    dev, out = _dev(x0=x0)
    assert dev._ref("L") and not dev._hard("L")
    _home(dev)
    assert dev.homed and dev.sup.mode == Mode.IDLE
    measured = dev.true_state[0] - dev.x_offset                       # firmware'in x'i (enc_sign = 1)
    assert measured == pytest.approx(dev.true_state[0], abs=0.003)    # sıfır hatası < 3 mm
    m = re.search(r"static const float HOME_BACKOFF_M = ([\d.]+)f;", FW)
    assert m and float(m.group(1)) == MD.HOME_BACKOFF


def test_backoff_with_stuck_optical_faults():
    dev, out = _dev(x0=0.0)
    dev.stuck_switch = "L"                                              # optik hep etkin (kopuk kablo)
    _home(dev)
    assert _events(out, "fault")[-1]["reason"] == "homing_switch"
