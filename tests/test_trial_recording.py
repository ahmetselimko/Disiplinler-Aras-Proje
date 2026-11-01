"""Trial summaries and recording settings follow the full serial stream."""
import json
from types import SimpleNamespace

import pytest

from pendulum_lab.comm import protocol as P
from pendulum_lab.comm.reader import TelemetryReader
from pendulum_lab.recorder import Recorder


def reader():
    return TelemetryReader(SimpleNamespace(write=lambda data: None))


def tel(n, t, mode="BALANCE", u=0.):
    return P.encode_telemetry(n, t, 0., 0., 0., 0., u, mode)


def test_settings_commands_and_acceptance_are_preserved_in_csv(tmp_path):
    rd = reader()
    path = tmp_path / "run.csv"
    rec = Recorder(path, meta={"Q": [10, 1, 100, 1], "swingup": {"k_E": 100.}})
    rd.sinks.append(rec)
    rd._handle_lines([tel(0, 0.)])
    rd.send("set_param", u_max=6.)
    rd._handle_lines([P.encode_ack("set_param"), tel(1, .002, u=6.), tel(2, .004)])
    rd.send("set_param", u_max=8.)
    rd._handle_lines([P.encode_ack("set_param", False, "reddedildi")])
    rec.close()
    lines = path.read_text(encoding="utf-8").splitlines()
    assert json.loads(lines[0].split(": ", 1)[1]) == [10, 1, 100, 1]
    events = [json.loads(line[len("# event: "):]) for line in lines if line.startswith("# event: ")]
    assert [e["kind"] for e in events] == ["command_sent", "Ack", "command_sent", "Ack"]
    assert events[0]["fields"] == {"u_max": 6.}
    assert events[-1]["ok"] is False
    assert rd.trials.u_max == 6.
    assert rd.applied_settings["u_max"] == 6.
    assert rd.trials.snapshot()[0].sat_time == pytest.approx(.002)


def test_pause_is_applied_only_after_successful_ack():
    rd = reader()
    rd._handle_lines([tel(0, 0.)])
    rd.send("pause", on=1)
    rd._handle_lines([P.encode_ack("pause", False), tel(1, 1.)])
    assert rd.trials.snapshot()[0].settling_time == 0.
    rd.send("reset", th0_deg=180.)
    rd._handle_lines([P.encode_ack("reset"), tel(2, 1.1, "IDLE")])
    assert not rd.trials.snapshot()[1]
    rd._handle_lines([tel(3, 2.)])
    rd.send("pause", on=1)
    rd._handle_lines([P.encode_ack("pause"), tel(4, 2.5), tel(5, 3.), tel(6, 3.5)])
    assert rd.trials.snapshot()[0].settling_time is None


def test_boot_resets_paused_state_and_does_not_join_trials():
    rd = reader()
    rd._handle_lines([tel(0, 10.)])
    rd.trials.set_paused(True)
    rd._handle_lines([P.encode_event("boot"), tel(0, 0.), tel(1, 1.)])
    s, active = rd.trials.snapshot()
    assert active and s.settling_time == 0. and s.samples == 2


def test_recording_failure_does_not_report_successful_command_as_failed():
    rd = reader()
    def fail(*args):
        raise OSError("disk full")
    sink = SimpleNamespace(write_event=fail)
    rd.sinks.append(sink)
    rd.send("stop")
    assert sink not in rd.sinks
    assert rd.messages.get_nowait().name == "recorder_error"


def test_nonfinite_sample_does_not_bridge_settling_or_saturation():
    rd = reader()
    rd._handle_lines([tel(0, 0., u=12.), tel(1, .5, u=float("nan")), tel(2, 1.1)])
    s, _ = rd.trials.snapshot()
    assert s.settling_time is None and s.sat_time == 0.
