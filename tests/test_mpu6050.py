"""#50 Aşama 5: MPU6050 jiroskop desteği (firmware USE_MPU6050, sanal ESP32 gyro=True, SensorModel, kayıtlar)."""

import math
import re
from pathlib import Path

import numpy as np
import pytest

from pendulum_lab import load_params
from pendulum_lab.comm import mock_device as MD
from pendulum_lab.comm import protocol as P
from pendulum_lab.comm.mock_device import MockDevice
from pendulum_lab.control.lqr import design_lqr
from pendulum_lab.control.supervisor import Mode, Supervisor
from pendulum_lab.sim.simulator import GYRO_RANGE, SensorModel, mix_gyro, simulate_closed_loop

ROOT = Path(__file__).resolve().parents[1]
FW = (ROOT / "firmware" / "esp32_pendulum" / "esp32_pendulum.ino").read_text(encoding="utf-8")


def _dev(gyro=True, **kw):
    out = []
    dev = MockDevice(load_params(ROOT / "config" / "params.json"), realtime=False, seed=2, gyro=gyro, **kw)
    dev.output = out.append
    dev.start()
    return dev, out


def _msgs(out):
    return [P.parse_line(o) for o in out]


def test_default_off_telemetry_unchanged():
    dev, out = _dev(gyro=False)
    for _ in range(20):
        dev.step()
    tel = [m for m in _msgs(out) if isinstance(m, P.Telemetry)]
    assert tel and all(m.gy is None for m in tel)
    assert all(b'"gy"' not in o for o in out)
    assert re.search(r"#ifndef USE_MPU6050\s*\n#define USE_MPU6050 0", FW)


def test_mock_gyro_telemetry_bias_and_mix():
    dev, out = _dev()
    dev.s[3] = 1.5                                          # sarkaç dönüyor
    for _ in range(5):
        dev.step()
    tel = [m for m in _msgs(out) if isinstance(m, P.Telemetry)]
    assert tel[-1].gy == pytest.approx(dev.s[3], abs=0.05)  # bias çıkarılmış, θ ile aynı işaret
    out.clear()
    dev.feed(P.encode_command("calib"))
    dev.step()
    calib = [m for m in _msgs(out) if isinstance(m, P.Event) and m.name == "calib"][0]
    assert calib.data["gyro_bias"] == pytest.approx(dev.sensor.gyro_bias, abs=0.005)
    dev.feed(P.encode_command("set_param", gyro_mix=0.7))
    dev.feed(b'{"cmd":"set_param","gyro_mix":1.5}\n')   # sınır dışı (encode_command zaten reddeder)
    out.clear()
    dev.step()
    acks = [m for m in _msgs(out) if isinstance(m, P.Ack)]
    assert [a.ok for a in acks] == [True, False] and dev.gyro_mix == 0.7
    assert mix_gyro(1.0, 3.0, 0.25) == pytest.approx(1.5) and mix_gyro(1.0, 3.0, 0.0) == 1.0


def test_gyro_failure_is_not_a_fault_and_event_once():
    dev, out = _dev()
    dev.gyro_mix = 1.0
    dev.sup.mode = Mode.IDLE
    dev.gyro_fail = True
    for _ in range(20):
        dev.step()
    ev = [m for m in _msgs(out) if isinstance(m, P.Event) and m.name == "gyro"]
    assert [e.data for e in ev] == [{"ok": False}]
    assert not dev.mpu_ok and dev.sup.mode != Mode.FAULT
    out.clear()
    for _ in range(10):
        dev.step()
    tel = [m for m in _msgs(out) if isinstance(m, P.Telemetry)]
    assert tel and all(m.gy is None for m in tel)            # jiroskopsuz devam
    for k in ("MPU_FAIL_LIMIT", "MPU_BIAS_SAMPLES"):
        m = re.search(rf"static const int\s+{k} = (\d+);", FW)
        assert m and int(m.group(1)) == getattr(MD, k)
    assert 'Serial.print("{\\"evt\\":\\"gyro\\",\\"ok\\":false}\\n");' in FW
    assert "raiseFault" not in FW.split("++mpu_fails >= MPU_FAIL_LIMIT", 1)[1].split("\n", 1)[0]


def test_protocol_and_firmware_fields():
    assert P.SET_PARAM_LIMITS["gyro_mix"] == (0.0, 1.0)
    assert '{"gyro_mix", 0.0f, 1.0f}' in FW
    t = P.parse_line(P.encode_telemetry(1, 0.1, 0, 0, 0.2, 0.3, 0, "BALANCE", gy=0.31))
    assert t.gy == pytest.approx(0.31)
    assert P.parse_line(P.encode_telemetry(1, 0.1, 0, 0, 0.2, 0.3, 0, "BALANCE")).gy is None
    assert '\\"gy\\":%.4f' in FW and '\\"gyro_bias\\":%.5f' in FW
    assert "0x43 + 2 * MPU_AXIS" in FW and "mpuWrite(0x1B, 0x18)" in FW and "who != 0x68" in FW
    wf = (ROOT / ".github" / "workflows" / "test.yml").read_text(encoding="utf-8")
    assert "mpu: 1" in wf and "-DUSE_MPU6050=${{ matrix.mpu }}" in wf


def test_csv_gy_column_and_old_files(tmp_path):
    from pendulum_lab.comm.reader import COLUMNS, OLD_COLUMNS
    from pendulum_lab.recorder import Recorder, load_recording
    rec = Recorder(tmp_path / "yeni.csv", meta={})
    rec.write_rows([(0.0, 0.0, 0.0, 3.1, 0.0, 0.0, 0.0, 0.25), (0.01, 0.0, 0.0, 3.1, 0.0, 0.0, 0.0, math.nan)])
    rec.close()
    s = load_recording(tmp_path / "yeni.csv")
    assert s.gy[0] == pytest.approx(0.25) and math.isnan(s.gy[1])
    old = tmp_path / "eski.csv"
    old.write_text("# theta_pozitif: sol\n" + ",".join(OLD_COLUMNS) + "\n0,0,0,3.1,0,0,IDLE\n0.01,0,0,3.1,0,0,IDLE\n",
                   encoding="utf-8")
    assert load_recording(old).gy is None and COLUMNS[-1] == "gy"


def test_simulation_mix_reduces_dth_noise():
    p = load_params(ROOT / "config" / "params.json")
    K, _, _ = design_lqr(p, (10, 1, 100, 1), [[0.1]], Ts=0.002)
    res = {}
    for mix in (0.0, 0.8):
        sup = Supervisor(p, K.ravel())
        sup.start()
        sensor = SensorModel.for_params(p, gyro=True, seed=1)
        r = simulate_closed_loop(p, sup, [0, 0, math.pi, 0], 3.0, sensor=sensor, gyro_mix=mix)
        out_kf = ~np.isfinite(r.dth_kf)                       # türev bölgesi
        res[mix] = float(np.sqrt(np.mean((r.dth_used[out_kf] - r.s[out_kf, 3]) ** 2)))
        assert r.gy is not None and np.abs(r.gy).max() <= GYRO_RANGE
    assert res[0.8] < 0.5 * res[0.0]
    r0 = simulate_closed_loop(p, Supervisor(p, K.ravel()), [0, 0, 0.05, 0], 0.5, sensor=SensorModel.for_params(p))
    assert r0.gy is None and np.array_equal(r0.dth_diff, r0.dth_mix)   # jiroskop yokken karışım etkisiz


def test_gui_gyro_plots_and_rmse():
    import os
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PyQt5 import QtWidgets
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(["test"])
    from gui.analysis_tab import SimulationPage
    from gui.live_tab import LiveTab
    from gui.state import AppState
    from pendulum_lab.comm.reader import COLUMNS, RingBuffer
    st = AppState(ROOT / "config" / "params.json")
    page = SimulationPage(st)
    page.th0.setValue(180.0)
    page.T.setValue(2.0)
    page.gyro_cb.setChecked(True)
    page.run()
    text = page.info.text()
    assert "θ̇ RMSE" in text and "karışım γ=0.50" in text and "yalnız jiroskop" in text

    class _Reader:
        rate_hz = 200.0
        last = None
        def __init__(self):
            self.buffer = RingBuffer(400, len(COLUMNS))
            self.applied_settings = {}
    live = LiveTab(st)
    rd = _Reader()
    rd.buffer.extend([[i * 0.005, 0, 0, 3.0, 0.1, 0, 3.0, 0.12] for i in range(200)])
    live.refresh(rd)
    assert live.p_w.isVisible() and not live.gyro_mix.isHidden()
    rd.buffer.clear()
    rd.buffer.extend([[i * 0.005, 0, 0, 3.0, 0.1, 0, 3.0, math.nan] for i in range(200)])
    live.refresh(rd)
    assert not live.p_w.isVisible() and live.gyro_mix.isHidden()
    live.close()
    assert app is not None



def test_frozen_gyro_detected_and_normal_run_not():
    """İnceleme (Aşama 5, 1. tur): MPU resetlenip uykuya dönerse okuma başarılı ama değer 0 → tutarlılık
    denetimi jiroskobu kapatır (FAULT yok, olay bir kez). Normal swing-up'ta denetim yanlış tetiklenmez."""
    assert "GYRO_CHECK_RAD_S = 3.0f" in FW and "GYRO_CHECK_N = 25;" in FW
    assert MD.GYRO_CHECK_RAD_S == 3.0 and MD.GYRO_CHECK_N == 25
    assert "MPU_PWR_CHECK_N = 50;" in FW and MD.MPU_PWR_CHECK_N == 50 and "pwr != 0x01" in FW
    for frozen in (False, True):
        dev, out = _dev()
        dev.feed(P.encode_command("set_param", gyro_mix=1.0))
        dev.feed(P.encode_command("home"))
        for _ in range(6000):
            dev.step()
            if dev.homed and dev.sup.mode == Mode.IDLE:
                break
        assert dev.homed
        dev.gyro_frozen = frozen
        dev.feed(P.encode_command("start"))
        modes = set()
        for _ in range(5000):
            dev.step()
            modes.add(dev.sup.mode)
        ev = [m for m in _msgs(out) if isinstance(m, P.Event) and m.name == "gyro"]
        assert dev.mpu_ok is (not frozen) and len(ev) == (1 if frozen else 0)
        assert Mode.FAULT not in modes and Mode.BALANCE in modes   # γ = 1 iken de (donmuşta AS5600'e dönüp) yakalar


def test_calib_with_failed_gyro_and_boot_bias():
    dev, out = _dev()
    boot = [m for m in _msgs(out) if isinstance(m, P.Event) and m.name == "calib"][0]
    assert "gyro_bias" in boot.data                         # firmware gibi açılış calib olayında da
    out.clear()
    dev.gyro_fail = True
    dev.feed(P.encode_command("calib"))
    dev.step()
    names = [m.name for m in _msgs(out) if isinstance(m, P.Event)]
    assert names == ["calib", "gyro"] and not dev.mpu_ok           # firmware commTask sırası: önce calib, sonra gyro
    assert "vTaskDelay(pdMS_TO_TICKS(50));                     // uykudan çıkış" in FW   # uykudan çıkışı bekle
    assert FW.count("else if (angle_ok) gyro_bias = b;") == 1 and "if (mpu_ok && angle_ok) gyro_bias = b;" in FW


def test_gyro_mix_resent_even_when_k_blocked(tmp_path):
    """İnceleme (Aşama 5, 2. tur): γ, K engellerinden (yer tutucu parametreler, gerçek kart) bağımsız yeniden gider."""
    from gui.state import AppState
    st = AppState(ROOT / "config" / "params.json")
    sent = []

    class R:
        transport = object()                                  # SimTransport değil: gerçek kart
        def send(self, cmd, **f):
            sent.append((cmd, f))
    st.reader = R()
    assert st.real_device and st.placeholder_on_real_device   # bugünkü params.json yer tutucu
    st.gyro_mix = 0.8
    st.on_device_info({"fw": "ref-1.2", "theta": "sol", "Ts": 0.002}, boot=True)
    assert ("set_param", {"gyro_mix": 0.8}) in sent
    sent.clear(); st.gyro_mix = 0.0
    st.on_device_info({"fw": "ref-1.2", "theta": "sol", "Ts": 0.002}, boot=True)
    assert all("gyro_mix" not in f for _, f in sent)          # varsayılan 0: gereksiz alan gönderilmez


def test_rmse_lines_without_kalman_has_no_nan():
    from types import SimpleNamespace
    from gui.analysis_tab import dth_rmse_lines
    n = 50
    true = np.zeros((n, 4)); true[:, 3] = np.linspace(0, 1, n)
    r = SimpleNamespace(s=true, dth_diff=true[:, 3] + 0.1, dth_mix=true[:, 3] + 0.05, gy=true[:, 3],
                        dth_kf=np.full(n, np.nan))
    text = "\n".join(dth_rmse_lines(r, 0.5))
    assert "nan" not in text and "Kalman" not in text
