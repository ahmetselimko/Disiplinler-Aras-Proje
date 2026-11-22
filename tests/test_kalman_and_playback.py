"""Kalman filtresi titreşim azaltma (#14) ve Kayıt Oynatma sekmesi (#15) testleri."""

from __future__ import annotations

import copy
import math
import os
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from PyQt5 import QtWidgets

from gui.main_window import MainWindow
from gui.state import AppState
from pendulum_lab import PendulumParams, SensorModel, Supervisor, simulate_closed_loop
from pendulum_lab.comm import protocol as P
from pendulum_lab.comm.mock_device import MockDevice
from pendulum_lab.comm.reader import TelemetryReader
from pendulum_lab.control.lqr import design_lqr
from pendulum_lab.export import gains_header
from pendulum_lab.filters import KalmanFilter, design_kalman
from pendulum_lab.model.dynamics import wrap_angle
from pendulum_lab.recorder import Recorder, compare_recording_with_simulation, load_recording

ROOT = Path(__file__).resolve().parents[1]
FIRMWARE_DIR = ROOT / "firmware" / "esp32_pendulum"


@pytest.fixture(scope="module")
def qapp():
    os.environ["QT_QPA_PLATFORM"] = "offscreen"
    app = QtWidgets.QApplication.instance()
    if app is None:
        app = QtWidgets.QApplication(["test"])
    yield app


def test_kalman_reduces_balance_voltage_ripple():
    p = PendulumParams()
    Ts = 0.002
    K, _, _ = design_lqr(p, [10.0, 1.0, 100.0, 1.0], [[0.1]], Ts=Ts)

    sup_lp = Supervisor(p, K)
    sup_lp.start()
    r_lp = simulate_closed_loop(p, sup_lp, [0, 0, math.pi, 0], 15.0, dt=Ts, sensor=SensorModel(seed=0), use_kalman=False)

    sup_kf = Supervisor(p, K)
    sup_kf.start()
    r_kf = simulate_closed_loop(p, sup_kf, [0, 0, math.pi, 0], 15.0, dt=Ts, sensor=SensorModel(seed=0), use_kalman=True)

    assert r_lp.mode[-1] == "BALANCE"
    assert r_kf.mode[-1] == "BALANCE"

    u_lp = r_lp.u[r_lp.t >= 10.0]
    u_kf = r_kf.u[r_kf.t >= 10.0]
    assert np.std(u_kf) <= 0.5 * np.std(u_lp)
    assert np.max(np.abs(u_lp)) > 1.4
    assert np.max(np.abs(u_kf)) < 0.8


def test_kalman_steady_state_gain_matches_dynamic_filter_and_wraps_angle():
    p = PendulumParams()
    Ts = 0.002
    Ad, Bd, L, _ = design_kalman(p, Ts)
    s0 = np.array([0.02, -0.05, 0.03, 0.12])
    kf = KalmanFilter.from_params(p, Ts, s0=s0.copy())
    s_ss = s0.copy()

    for i in range(100):
        y = [0.02 + 0.001 * math.sin(0.2 * i), 0.03 + 0.002 * math.cos(0.2 * i)]
        u = 0.4 * math.sin(0.1 * i)
        s_dyn = kf.step(y, u)

        s_pred = Ad @ s_ss + (Bd * u).ravel()
        s_pred[2] = wrap_angle(float(s_pred[2]))
        inn = np.array([y[0] - s_pred[0], wrap_angle(float(y[1] - s_pred[2]))])
        s_ss = s_pred + L @ inn
        s_ss[2] = wrap_angle(float(s_ss[2]))

        np.testing.assert_allclose(s_dyn, s_ss, atol=1e-12)

    # ±pi geçişinde açı sarma yeniliği büyük bir sahte hız sıçraması üretmemeli
    kf_wrap = KalmanFilter.from_params(p, Ts, s0=[0.0, 0.0, math.pi - 0.01, 0.0])
    s_wrap = kf_wrap.step([0.0, -math.pi + 0.01], 0.0)
    assert abs(s_wrap[3]) < 1.0


def test_firmware_kalman_matches_python():
    p = PendulumParams()
    h = gains_header(p, [0, 0, 0, 0], 0.002)
    gains_src = (FIRMWARE_DIR / "gains.h").read_text(encoding="utf-8")
    ino_src = (FIRMWARE_DIR / "esp32_pendulum.ino").read_text(encoding="utf-8")

    for macro in ("KALMAN_AD", "KALMAN_BD", "KALMAN_L"):
        assert macro in h
        assert macro in gains_src

    assert "struct KalmanEst" in ino_src
    assert "kf.update(" in ino_src
    assert "kf.reset();" in ino_src


def _record_mock_session(csv_path: Path, st: AppState) -> None:
    """Sanal ESP32 ile homing + swing-up + denge kaydı üretir (depoya bağımlı olmayan test verisi)."""
    dev = MockDevice(params=copy.deepcopy(st.p), seed=0, realtime=False)   # arayüz gibi: aynı parametreler
    dev.sup.set_gain(st.K)                                                  # ve aynı K (bağlanınca gönderilir)
    out: list[bytes] = []
    dev.output = out.append
    dev.start()
    dev.feed(P.encode_command("home"))
    for _ in range(int(15.0 / dev.Ts)):
        dev.step()
        if dev.homed:
            break
    assert dev.homed

    dev.feed(P.encode_command("start"))
    for _ in range(int(8.0 / dev.Ts)):
        dev.step()
    assert dev.sup.mode.value == "BALANCE"

    rd = TelemetryReader(SimpleNamespace(write=lambda data: None))
    rec = Recorder(csv_path, meta={"kaynak": "simülasyon (test)", **st.get_metadata()})
    rd.sinks.append(rec)
    rd._handle_lines(b"".join(out).splitlines(keepends=True))
    rec.close()


def test_mock_device_recording_and_playback_comparison(tmp_path):
    csv_path = tmp_path / "sim_test.csv"
    st = AppState(ROOT / "config" / "params.json")
    _record_mock_session(csv_path, st)

    session = load_recording(csv_path)
    comp = compare_recording_with_simulation(session, fallback_state=st)

    assert comp.mode[-1] == "BALANCE"
    assert comp.sim.mode[-1] == "BALANCE"
    assert math.isfinite(comp.rmse_th_deg) and comp.rmse_th_deg < 10.0
    assert math.isfinite(comp.rmse_x) and comp.rmse_x < 0.05


def test_playback_tab_gui_loads_and_renders(qapp, tmp_path, monkeypatch):
    # kayitlar/ .gitignore'da: test kendi kaydını üretir, her bilgisayarda çalışır
    csv_path = tmp_path / "gui_test.csv"
    st = AppState(ROOT / "config" / "params.json")
    _record_mock_session(csv_path, st)

    # hata olursa modal pencere testi kilitlemesin, testi düşürsün
    def _fail(*args, **kwargs):
        raise AssertionError(f"beklenmeyen uyarı penceresi: {args[1:]}")
    monkeypatch.setattr(QtWidgets.QMessageBox, "warning", _fail)

    win = MainWindow(st)
    try:
        tab_titles = [win.tabs.tabText(i) for i in range(win.tabs.count())]
        assert "Kayıt Oynatma" in tab_titles

        win.playback.load_file(csv_path)
        x_rec, y_rec = win.playback.c_rec_th.getData()
        x_sim, y_sim = win.playback.c_sim_th.getData()
        assert len(x_rec) > 10 and len(y_rec) == len(x_rec)
        assert len(x_sim) > 10 and len(y_sim) == len(x_sim)
        assert "RMSE(θ)" in win.playback.info.text()
        assert win.playback.slider.maximum() == len(x_rec) - 1

        mid_idx = len(x_rec) // 2
        win.playback.slider.setValue(mid_idx)
        assert win.playback.cursor[0].value() == pytest.approx(float(x_rec[mid_idx]))
    finally:
        win.close()


# ---------------------------------------------------------------- inceleme bulguları (regresyon)

def _record_via_reader(csv_path: Path, st: AppState, *, before_start=(), after_start=(),
                       idle_s: float = 1.0, run_s: float = 8.0) -> None:
    """Sanal ESP32'yi TelemetryReader üzerinden sürer: komutlar ve ACK'ler arayüzdeki gibi kayda olay
    olarak yazılır. Homing'den sonra kayıt açılır, idle_s beklenir, before_start komutları gönderilir,
    BAŞLAT verilir, 1 s sonra after_start komutları gönderilir."""
    dev = MockDevice(params=copy.deepcopy(st.p), seed=0, realtime=False)   # arayüz gibi: aynı parametreler
    dev.sup.set_gain(st.K)                                                  # ve aynı K
    out: list[bytes] = []
    dev.output = out.append

    class _Tr:
        name = "mock"

        def write(self, data):
            dev.feed(data)

    rd = TelemetryReader(_Tr())

    def pump(sec: float):
        for _ in range(int(round(sec / dev.Ts))):
            dev.step()
        rd._handle_lines(b"".join(out).splitlines(keepends=True))
        out.clear()

    dev.start()
    rd.send("home")
    pump(15.0)
    assert dev.homed
    rec = Recorder(csv_path, meta={"kaynak": "test", **st.get_metadata()})
    rd.sinks.append(rec)
    pump(idle_s)
    for cmd, fields in before_start:
        rd.send(cmd, **fields)
        pump(0.02)
    rd.send("start")
    pump(1.0)
    for cmd, fields in after_start:
        rd.send(cmd, **fields)
        pump(0.02)
    pump(run_s)
    rec.close()


def test_replay_keeps_initial_velocity_on_first_step():
    """Bulgu 1: gerçekçi sensörde hız kestiricileri ilk adımda başlangıç hızını sıfırlamamalı."""
    p = PendulumParams()
    Ts = 0.002
    K, _, _ = design_lqr(p, [10.0, 1.0, 100.0, 1.0], [[0.1]], Ts=Ts)
    s0 = [0.0, 0.10, 0.0, 0.60]
    ideal = Supervisor(p, K)
    ideal.mode = ideal.mode.BALANCE
    r_ideal = simulate_closed_loop(p, ideal, s0, 0.02, dt=Ts, sensor=None)
    assert abs(r_ideal.u[0]) > 5.0
    for kf in (False, True):
        sup = Supervisor(p, K)
        sup.mode = sup.mode.BALANCE
        r = simulate_closed_loop(p, sup, s0, 0.02, dt=Ts, sensor=SensorModel(seed=0), use_kalman=kf)
        assert r.u[0] == pytest.approx(r_ideal.u[0], abs=0.5), f"use_kalman={kf}"


def test_replay_applies_settings_acknowledged_during_recording(tmp_path):
    """Bulgu 2: kayıt açıldıktan sonra cihazın onayladığı ayarlar (u_max) simülasyona yansımalı."""
    st = AppState(ROOT / "config" / "params.json")
    csv_path = tmp_path / "umax.csv"
    _record_via_reader(csv_path, st, before_start=[("set_param", {"u_max": 2.0})])
    session = load_recording(csv_path)
    assert session.meta["device_acknowledged_settings"] == {}     # başlıkta yok, yalnız olaylarda
    comp = compare_recording_with_simulation(session, fallback_state=st)
    assert comp.params.u_max == pytest.approx(2.0)
    assert np.max(np.abs(comp.u)) <= 2.0 + 1e-6
    assert np.max(np.abs(comp.sim.u)) <= 2.0 + 1e-6
    assert not comp.changed_during_trial


def test_replay_reports_settings_changed_after_start(tmp_path):
    """Bulgu 2 (devamı): BAŞLAT'tan sonra değişen ayar başlangıç koşuluna karışmamalı, uyarı olarak dönmeli."""
    st = AppState(ROOT / "config" / "params.json")
    csv_path = tmp_path / "later.csv"
    _record_via_reader(csv_path, st, after_start=[("set_param", {"x_ref": 0.05})])
    comp = compare_recording_with_simulation(load_recording(csv_path), fallback_state=st)
    assert comp.changed_during_trial == ["x_ref"]


def test_full_recording_mode_starts_simulation_at_start_command(tmp_path):
    """Bulgu 3: tam kayıt modunda simülasyon kayıttaki BAŞLAT anında başlamalı (IDLE'da swing-up yok)."""
    st = AppState(ROOT / "config" / "params.json")
    csv_path = tmp_path / "full.csv"
    _record_via_reader(csv_path, st, idle_s=2.0)
    session = load_recording(csv_path)
    assert session.mode[0] == "IDLE"
    act = compare_recording_with_simulation(session, fallback_state=st, active_only=True)
    full = compare_recording_with_simulation(session, fallback_state=st, active_only=False)
    i_start = next(i for i, m in enumerate(session.mode) if m != "IDLE")
    assert full.t_start == pytest.approx(session.t[i_start] - session.t[0])
    assert full.sim.t[0] == pytest.approx(full.t_start)
    assert full.t[0] == 0.0 and act.t_start == 0.0
    assert full.rmse_th_deg == pytest.approx(act.rmse_th_deg, abs=1e-9)
    assert full.rmse_th_deg < 10.0


def test_playback_speed_follows_csv_timestamps(qapp, tmp_path):
    """Bulgu 4: oynatma hızı düzensiz örnek aralığında da kayıt zamanını doğru izlemeli."""
    st = AppState(ROOT / "config" / "params.json")
    csv_path = tmp_path / "speed.csv"
    _record_mock_session(csv_path, st)
    win = MainWindow(st)
    try:
        tab = win.playback
        tab.load_file(csv_path)
        assert len(set(np.round(np.diff(tab.comparison.t), 4))) > 1     # aralıklar gerçekten düzensiz
        for label, factor in (("0.25x", 0.25), ("0.5x", 0.5), ("1x", 1.0), ("2x", 2.0)):
            tab.speed.setCurrentText(label)
            tab.slider.setValue(0)
            tab._rebase_clock()
            i = tab.frame_at(1.6)                # 100 × 16 ms
            advanced = tab.comparison.t[i] - tab.comparison.t[0]
            assert advanced == pytest.approx(1.6 * factor, abs=0.01), label
    finally:
        win.close()


def test_firmware_compiles_kalman_out_without_matrices():
    """Bulgu 5: eski gains.h (KALMAN_* yok) ile firmware derlenebilmeli; Kalman kodu derleme dışı kalmalı."""
    h = gains_header(PendulumParams(), [0, 0, 0, 0], 0.002)
    assert "#define KALMAN_AVAILABLE 1" in h
    assert "#define KALMAN_AVAILABLE 1" in (FIRMWARE_DIR / "gains.h").read_text(encoding="utf-8")
    ino = (FIRMWARE_DIR / "esp32_pendulum.ino").read_text(encoding="utf-8")
    assert "#if USE_KALMAN && !defined(KALMAN_AVAILABLE)" in ino
    body = ino[ino.index("struct KalmanEst"):]
    body = body[:body.index("\n};")]
    refs = ("KALMAN_AD[", "KALMAN_BD[", "KALMAN_L[")
    guard, rest = body.split("#if !USE_KALMAN", 1)
    disabled, enabled = rest.split("#else", 1)
    assert not any(r in guard or r in disabled for r in refs)
    assert all(r in enabled for r in refs)


def test_replay_config_ignores_removed_swingup_fields_and_reads_known_ones():
    """Eski kayıtlardaki kaldırılmış alanlar (E_target) oynatmayı bozmamalı; tanınan alanlar okunmalı."""
    from pendulum_lab.recorder import SWINGUP_FIELDS, _extract_replay_config

    su = {"k_E": 25.0, "a_max": 7.0, "k_x": 9.0, "k_v": 5.0, "a_kick": 2.0, "E_target": 0.0, "eski_alan": 3.0}
    swingup = _extract_replay_config({"swingup": su})[4]
    assert swingup.k_E == 25.0 and swingup.a_max == 7.0
    ack = {k: 1.0 for k in SWINGUP_FIELDS if k != "a_kick"}
    swingup = _extract_replay_config({"swingup": su, "device_acknowledged_settings": ack})[4]
    assert all(getattr(swingup, k) == 1.0 for k in ack)


def test_replay_catch_limits_priority():
    """Geçiş sınırları: cihazın onayladığı değer > kayıt başlığı > AppState; Supervisor'da olmayan atlanır."""
    from pendulum_lab.recorder import _catch_limits

    meta = {"catch_dth": 3.0, "catch_x": 0.2, "device_acknowledged_settings": {"catch_dth": 2.0}}
    fb = SimpleNamespace(catch_dx=1.5)
    assert _catch_limits(meta, fb) == {"catch_dth": 2.0, "catch_x": 0.2, "catch_dx": 1.5}
    assert _catch_limits({}) == {}


def test_kalman_encoder_cpr_matches_firmware():
    """Kalman ölçüm gürültüsündeki enkoder çözünürlüğü firmware'deki ENC_CPR_MOTOR ile aynı olmalı."""
    import re

    from pendulum_lab.filters import ENC_CPR_MOTOR

    ino = (FIRMWARE_DIR / "esp32_pendulum.ino").read_text(encoding="utf-8")
    m = re.search(r"ENC_CPR_MOTOR\s*=\s*([\d.]+)f", ino)
    assert m and float(m.group(1)) == ENC_CPR_MOTOR


def test_simulated_encoder_resolution_matches_firmware():
    """Simülasyonun enkoder sayımı firmware'deki X_PER_COUNT = 2π·r / (ENC_CPR_MOTOR·MOTOR_GEAR) ile aynı;
    sanal ESP32 de bunu kullanır (eski motorun 1.25e-4 m değeri kalmamalı)."""
    from pendulum_lab.filters import ENC_CPR_MOTOR, encoder_resolution
    from pendulum_lab.model.params import load_params
    from pendulum_lab.sim.simulator import SensorModel

    p = load_params(ROOT / "config" / "params.json")
    res = 2 * math.pi * p.motor.r_pulley / (ENC_CPR_MOTOR * p.motor.gear)
    assert encoder_resolution(p) == pytest.approx(res)
    assert SensorModel.for_params(p).x_resolution == pytest.approx(res)
    assert SensorModel().x_resolution == pytest.approx(res)               # varsayılan da 48 CPR
    assert MockDevice(params=p, seed=0).sensor.x_resolution == pytest.approx(res)
