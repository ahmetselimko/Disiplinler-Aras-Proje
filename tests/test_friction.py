"""Sürtünme ölçümü (friction komutu): doğruluk, kabul/ret kuralları, telafi ve firmware eşliği."""

import copy
import json
import math
import re
from pathlib import Path

import pytest

from pendulum_lab import PendulumParams
from pendulum_lab.comm import protocol as P
from pendulum_lab.comm.mock_device import MockDevice
from pendulum_lab.control import friction as FR
from pendulum_lab.control.supervisor import Mode

FIRMWARE = Path(__file__).resolve().parents[1] / "firmware" / "esp32_pendulum" / "esp32_pendulum.ino"


def _device(T_f=0.0, homed=True, seed=1):
    """Fiziğinde T_f sürtünmesi olan, kontrolcüsü sürtünmeyi bilmeyen, ortada duran sanal cihaz."""
    p = PendulumParams()
    p.motor.T_f = T_f
    out = []
    dev = MockDevice(params=p, seed=seed, realtime=False, x0=0.0)
    dev.output = out.append
    dev.start()
    dev.sup.p = copy.deepcopy(p)
    dev.sup.p.motor.T_f = 0.0
    dev.homed, dev.x_offset = homed, 0.0
    dev.s = [0.0, 0.0, math.pi, 0.0]                               # sarkaç tam aşağıda, hareketsiz
    for _ in range(100):
        dev.step()
    out.clear()
    return dev, out


def _events(out, name):
    return [json.loads(o) for o in out if f'"evt":"{name}"'.encode() in o]


def _ack(out, cmd):
    return [json.loads(o) for o in out if f'"ack":"{cmd}"'.encode() in o][-1]


def _measure(dev, out, seconds=20.0):
    dev.feed(P.encode_command("friction"))
    for _ in range(int(seconds / dev.Ts)):
        dev.step()
        if _events(out, "friction") or dev.sup.mode == Mode.FAULT:
            break
    return _events(out, "friction")


@pytest.mark.parametrize("T_f", [0.0, 0.0018, 0.0027])
def test_measures_known_friction_as_voltage(T_f):
    dev, out = _device(T_f)
    res = _measure(dev, out)
    assert _ack(out, "friction")["ok"]
    assert res and res[-1]["ok"], res
    m, p = dev.p.motor, dev.p
    true_v = p.F_coulomb / m.alpha
    assert abs(res[-1]["v_c"] - true_v) < 0.1                      # simülasyonda ±0.1 V (bkz. friction.py)
    assert res[-1]["slope"] == pytest.approx((m.beta + p.b_cart) / m.alpha, rel=0.25)
    assert dev.sup.mode == Mode.IDLE
    assert abs(dev.s[0]) < p.rail_half - 0.15                       # ölçümden sonra araba rayın içinde


def test_rejected_unless_homed_idle_centered_and_still():
    dev, out = _device(homed=False)
    dev.feed(P.encode_command("friction"))
    dev.step()
    assert not _ack(out, "friction")["ok"] and dev.sup.mode == Mode.IDLE

    dev, out = _device()
    dev.s[0] = dev.p.rail_half - FR.FR_TRAVEL - FR.FR_X_MARGIN + 0.02
    dev.step()
    dev.step()
    dev.feed(P.encode_command("friction"))
    dev.step()
    assert "ortada değil" in _ack(out, "friction")["msg"]

    dev, out = _device()
    dev.s[2], dev.s[3] = math.pi - 0.6, 0.0                        # sarkaç aşağıda değil
    dev.step()
    dev.feed(P.encode_command("friction"))
    dev.step()
    assert "sarkaç" in _ack(out, "friction")["msg"] and dev.sup.mode == Mode.IDLE


def test_stop_aborts_measurement_and_estop_faults():
    dev, out = _device(0.0018)
    dev.feed(P.encode_command("friction"))
    for _ in range(300):
        dev.step()
    assert dev.sup.mode == Mode.FRICTION
    dev.feed(P.encode_command("stop"))
    dev.step()
    assert dev.sup.mode == Mode.IDLE and not _events(out, "friction")

    dev, out = _device(0.0018)
    dev.feed(P.encode_command("friction"))
    for _ in range(300):
        dev.step()
    dev.feed(P.encode_command("estop"))
    dev.step()
    assert dev.sup.mode == Mode.FAULT


def test_fc_volt_changes_controller_compensation_not_physics():
    dev, out = _device(0.0018)
    dev.feed(P.encode_command("set_param", fc_volt=0.5))
    dev.step()
    assert _ack(out, "set_param")["ok"]
    assert dev.sup.p.F_coulomb == pytest.approx(0.5 * dev.p.motor.alpha)
    assert dev.p.motor.T_f == 0.0018                               # fizik değişmedi


def test_firmware_friction_matches_python():
    src = FIRMWARE.read_text(encoding="utf-8")
    for name in [n for n in dir(FR) if n.startswith("FR_")]:
        m = re.search(rf"static const float {name} = ([\d.]+)f;", src)
        assert m, name
        assert float(m.group(1)) == getattr(FR, name), name
    for reason in ("sarkaç sallandı", "araba durmadı", "yeterli hıza ulaşılamadı (gerilim/yol/süre sınırı)",
                   "araba ortada değil: önce KALİBRE ET", "araba hareket ediyor: durmasını bekleyin",
                   "sarkaç aşağıda ve hareketsiz olmalı"):
        assert f'"{reason}"' in src, reason
    names = re.search(r"MODE_NAMES\[\] = \{([^}]*)\}", src).group(1)
    assert [s.strip().strip('"') for s in names.split(",")] == [m.value for m in Mode]


def test_gui_writes_measured_friction_and_sends_compensation(monkeypatch):
    import os
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    QtWidgets = pytest.importorskip("PyQt5.QtWidgets")
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])   # referans tutulmalı (yoksa Qt çöker)
    from gui.live_tab import LiveTab
    from gui.state import AppState

    st = AppState("yok/params.json")
    tab = LiveTab(st)
    sent = []
    monkeypatch.setattr(st, "send", lambda cmd, **f: sent.append((cmd, f)) or True)
    monkeypatch.setattr(QtWidgets.QMessageBox, "question", lambda *a, **k: QtWidgets.QMessageBox.Yes)
    tab.on_friction({"ok": True, "v_pos": 0.5, "v_neg": 0.46, "v_c": 0.48, "slope": 8.6})
    m = st.p.motor
    assert m.T_f == pytest.approx(0.48 * m.alpha * m.r_pulley / m.gear)
    assert ("set_param", {"fc_volt": 0.48}) in sent
    assert app is not None
