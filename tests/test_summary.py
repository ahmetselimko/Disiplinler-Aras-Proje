"""Deneme özeti, ilk dengeleme zamanı, doyum süresi ve CSV ayar başlığı testleri (Issue #6)."""

import math

import numpy as np
import pytest

from pendulum_lab.analysis.summary import (
    TrialSummary,
    compute_trial_summary,
    find_settling_time,
    input_saturation_time,
)
from pendulum_lab.model.params import PendulumParams
from pendulum_lab.recorder import Recorder
from pendulum_lab.analysis.summary import TrialAccumulator, LiveTrialTracker
from pendulum_lab.comm.protocol import Telemetry


def sample(n, t, mode="BALANCE", x=0., th=0., u=0.):
    return Telemetry(n, t, x, 0., th, 0., u, mode)


def test_stream_matches_batch_with_short_saturation_and_peak():
    t = np.arange(1001) * .002
    x, th, u = np.zeros_like(t), np.zeros_like(t), np.zeros_like(t)
    x[25], u[25] = .36, 12.
    tracker = LiveTrialTracker()
    for i in range(len(t)):
        tracker.update(sample(i, t[i], x=x[i], th=th[i], u=u[i]))
    actual, active = tracker.snapshot()
    expected = compute_trial_summary(t, x, th, u, modes=["BALANCE"]*len(t))
    assert active
    assert actual.to_dict() == expected.to_dict()
    assert actual.max_x == .36 and actual.sat_time == pytest.approx(.002)


def test_wait_balance_wait_is_one_trial_and_fault_is_included():
    tracker = LiveTrialTracker()
    for msg in [sample(0, 8., "WAIT", th=math.pi), sample(1, 8.1),
                sample(2, 8.2, "WAIT", th=math.pi), sample(3, 8.3),
                sample(4, 8.4, "FAULT", x=.39)]:
        tracker.update(msg)
    s, active = tracker.snapshot()
    assert not active and s.final_mode == "FAULT" and s.max_x == .39
    assert s.first_lqr_time == pytest.approx(.1)
    assert s.samples == 5 and s.settling_time is None


def test_lost_packets_cannot_establish_balance_or_extrapolate_saturation():
    tracker = LiveTrialTracker()
    tracker.update(sample(0, 0., u=12.))
    tracker.update(sample(200, 1.1, u=12.))
    s, _ = tracker.snapshot()
    assert s.settling_time is None and s.sat_time == 0
    assert s.missing_time == 1.1


def test_pause_cannot_establish_balance_or_saturation():
    tracker = LiveTrialTracker()
    tracker.update(sample(0, 0., u=12.))
    tracker.set_paused(True)
    for i in range(1, 22):
        tracker.update(sample(i, i*.1, u=12.))
    s, _ = tracker.snapshot()
    assert s.settling_time is None and s.sat_time == 0
    tracker.set_paused(False)
    for i in range(22, 34):
        tracker.update(sample(i, i*.1))
    assert tracker.snapshot()[0].settling_time == pytest.approx(2.2)


def test_disconnect_preserves_last_summary_and_restart_resets_clock():
    tracker = LiveTrialTracker()
    tracker.update(sample(0, 10., x=.2))
    tracker.finish()
    s, active = tracker.snapshot()
    assert s.max_x == .2 and not active
    tracker.update(sample(0, 0., "WAIT"))
    s, active = tracker.snapshot()
    assert s.max_x == 0 and s.samples == 1 and active


def test_long_trials_do_not_forget_early_peak():
    tracker = LiveTrialTracker()
    for i in range(25001):
        tracker.update(sample(i, i*.002, x=.35 if i == 0 else 0.))
    assert tracker.snapshot()[0].max_x == .35
    assert not hasattr(tracker, "_trial_samples")


@pytest.mark.parametrize("t, th", [([0, 0], [0, 0]), ([1, 0], [0, 0]), ([0, 1], [0]), ([0, 1], [0, float('nan')])])
def test_invalid_metric_inputs_are_rejected(t, th):
    with pytest.raises(ValueError):
        find_settling_time(t, th)


def test_saturation_left_hold_on_irregular_timestamps():
    assert input_saturation_time([0., .1, .7, 1.], [12., 0., -12., 0.], 12.) == pytest.approx((.4, 40.))


def test_find_settling_time_success():
    # 0'dan 5s'ye: ilk 2 saniye sarkaç sallanıyor (10°), 2. saniyeden itibaren 1°'ye düşüp dengede kalıyor
    t = np.linspace(0, 5.0, 501)
    th = np.zeros_like(t)
    th[t < 2.0] = math.radians(10.0)
    th[t >= 2.0] = math.radians(1.0)  # |θ| = 1° < 2°, 3 saniye boyunca kalıyor

    settle = find_settling_time(t, th, threshold_deg=2.0, min_duration=1.0)
    assert settle is not None
    assert pytest.approx(settle, abs=0.02) == 2.0


def test_find_settling_time_insufficient_duration():
    # 2.0'da girip 2.5'te çıkıyor (yalnızca 0.5s kalıyor, min_duration=1.0s)
    t = np.linspace(0, 5.0, 501)
    th = np.full_like(t, math.radians(15.0))
    th[(t >= 2.0) & (t <= 2.5)] = math.radians(1.0)

    settle = find_settling_time(t, th, threshold_deg=2.0, min_duration=1.0)
    assert settle is None


def test_find_settling_time_never_enters():
    t = np.linspace(0, 3.0, 301)
    th = np.full_like(t, math.radians(180.0))
    assert find_settling_time(t, th) is None


def test_find_settling_time_angle_wrap():
    # 360° veya -360° yakınında (0° ile eşdeğer)
    t = np.linspace(0, 2.0, 201)
    th = np.full_like(t, 2 * math.pi + math.radians(0.5))  # sarmalanınca 0.5°
    settle = find_settling_time(t, th, threshold_deg=2.0, min_duration=1.0)
    assert settle is not None
    assert pytest.approx(settle, abs=0.02) == 0.0


def test_input_saturation_time():
    t = np.linspace(0, 10.0, 1001)  # 10s
    u = np.zeros_like(t)
    # 2s boyunca 12V (doyum)
    u[(t >= 1.0) & (t <= 3.0)] = 12.0
    # 1s boyunca -12V (doyum)
    u[(t >= 6.0) & (t <= 7.0)] = -12.0

    sat_time, sat_pct = input_saturation_time(t, u, u_max=12.0)
    assert pytest.approx(sat_time, abs=0.05) == 3.0
    assert pytest.approx(sat_pct, abs=0.5) == 30.0


def test_compute_trial_summary():
    t = np.linspace(0, 4.0, 401)
    x = 0.1 * np.sin(t)
    th = np.full_like(t, math.radians(180.0))
    th[t >= 1.5] = math.radians(0.5)
    u = np.full_like(t, 5.0)
    u[t < 0.5] = 12.0  # 0.5s doyum
    modes = ["SWING_UP"] * 150 + ["BALANCE"] * 251

    summary = compute_trial_summary(t, x, th, u, modes=modes, u_max=12.0, rail_half=0.4)
    assert summary.settling_time is not None
    assert pytest.approx(summary.settling_time, abs=0.05) == 1.5
    assert pytest.approx(summary.max_x, abs=0.01) == 0.1
    assert pytest.approx(summary.max_u, abs=0.01) == 12.0
    assert pytest.approx(summary.sat_time, abs=0.05) == 0.5
    assert summary.final_mode == "BALANCE"
    assert summary.first_lqr_time is not None

    lines = summary.format_lines(rail_half=0.4, u_max=12.0)
    text = "\n".join(lines)
    assert "İlk dengeleme zamanı" in text
    assert "Giriş sınırında geçen süre" in text
    assert "Maksimum araba hareketi" in text


def test_recorder_contains_all_settings(tmp_path):
    # AppState'in tüm ayarları eksiksiz CSV başlığına kaydettiğini test et
    from gui.state import AppState
    st = AppState("config/params.json")
    meta = st.get_metadata()

    assert "K" in meta
    assert "Q" in meta
    assert "R" in meta
    assert "Ts" in meta
    assert "catch_deg" in meta
    assert "release_deg" in meta
    assert "swingup_enabled" in meta
    assert "x_ref" in meta
    assert "swingup" in meta
    assert "k_E" in meta["swingup"]
    assert "params" in meta

    csv_path = tmp_path / "test_run.csv"
    rec = Recorder(csv_path, meta=meta)
    rec.write_rows([(0.0, 0.05, 0.0, 0.01, 0.0, 1.5, 4.0)])
    rec.close()

    content = csv_path.read_text(encoding="utf-8")
    for key in ("K", "Q", "R", "Ts", "catch_deg", "release_deg", "swingup", "params"):
        assert f"# {key}:" in content


def test_design_changes_during_recording_keep_new_qr_and_ts(tmp_path):
    import json
    from gui.state import AppState
    st = AppState("config/params.json")
    path = tmp_path / "design.csv"
    st.recorder = Recorder(path, meta=st.get_metadata())
    st.Q = [20., 1., 200., 1.]
    st.Ts = .004
    st.compute_gains()
    st.recorder.close()
    events = [json.loads(line[len("# event: "):]) for line in path.read_text(encoding="utf-8").splitlines()
              if line.startswith("# event: ")]
    assert events[-1]["kind"] == "pc_design_snapshot"
    assert events[-1]["settings"]["Q"] == st.Q
    assert events[-1]["settings"]["Ts"] == .004
    assert events[-1]["settings"]["K"] == st.K.tolist()
