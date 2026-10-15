import math
import time

import numpy as np
import pytest

from pendulum_lab.comm import protocol as P
from pendulum_lab.comm.mock_device import MockDevice
from pendulum_lab.comm.reader import RingBuffer, TelemetryReader, COLUMNS
from pendulum_lab.comm.transport import SimTransport, Transport
from pendulum_lab.recorder import Recorder
from pendulum_lab.export import gains_header
from pendulum_lab import PendulumParams


# ------------------------------------------------------------------ protokol
def test_telemetry_roundtrip():
    line = P.encode_telemetry(7, 1.25, 0.12, -0.3, 0.05, 1.1, 2.5, "BALANCE")
    m = P.parse_line(line)
    assert isinstance(m, P.Telemetry)
    assert (m.n, m.t, m.x, m.th, m.mode) == (7, 1.25, 0.12, 0.05, "BALANCE")


def test_pdf_draft_format_accepted():
    m = P.parse_line('{"t": 1.25, "theta": 0.05, "x": 0.12}')
    assert m.th == 0.05 and m.x == 0.12 and m.n == -1


def test_event_and_ack():
    assert P.parse_line(P.encode_event("limit", side="L")) == P.Event("limit", {"side": "L"})
    assert P.parse_line(P.encode_ack("start", False, "önce home")) == P.Ack("start", False, "önce home")


@pytest.mark.parametrize("bad", [b"", b"{", b"[1,2]", b'{"foo":1}', b'{"t":"abc"}', b"\xff\xfe"])
def test_bad_lines_raise(bad):
    with pytest.raises(P.ProtocolError):
        P.parse_line(bad)


def test_command_validation():
    assert P.parse_command(P.encode_command("set_gain", K=[1, 2, 3, 4])) == ("set_gain", {"K": [1.0, 2.0, 3.0, 4.0]})
    with pytest.raises(P.ProtocolError):
        P.encode_command("set_gain", K=[1, 2])
    with pytest.raises(P.ProtocolError):
        P.encode_command("set_param", catch_deg=80)
    with pytest.raises(P.ProtocolError):
        P.encode_command("format_disk")


# ------------------------------------------------------------------ halka tampon
def test_ring_buffer_wraps_in_order():
    rb = RingBuffer(5, 1)
    rb.extend(np.arange(3).reshape(-1, 1))
    rb.extend(np.arange(3, 8).reshape(-1, 1))
    np.testing.assert_array_equal(rb.latest().ravel(), [3, 4, 5, 6, 7])
    np.testing.assert_array_equal(rb.latest(2).ravel(), [6, 7])
    rb.extend(np.arange(100, 112).reshape(-1, 1))  # kapasiteden büyük blok
    np.testing.assert_array_equal(rb.latest().ravel(), [107, 108, 109, 110, 111])


# ------------------------------------------------------------------ okuyucu
class ScriptedTransport(Transport):
    """Önceden belirlenmiş baytları rastgele parçalara bölerek veren sahte port."""

    def __init__(self, data: bytes, chunk: int = 7):
        self.data, self.chunk, self.pos, self._open = data, chunk, 0, False
        self.written = b""

    def open(self): self._open = True
    def close(self): self._open = False
    @property
    def is_open(self): return self._open
    def write(self, data): self.written += data

    def read(self, timeout=0.05):
        if self.pos >= len(self.data):
            time.sleep(0.005)
            return b""
        out = self.data[self.pos:self.pos + self.chunk]
        self.pos += self.chunk
        return out


def _wait(cond, timeout=3.0):
    t0 = time.time()
    while not cond() and time.time() - t0 < timeout:
        time.sleep(0.01)
    return cond()


def test_reader_reassembles_split_lines_and_counts_drops_and_errors():
    lines = [P.encode_telemetry(n, n * 0.005, 0, 0, 0, 0, 0, "IDLE") for n in range(100) if n != 50]
    lines.insert(10, b"garbage\n")
    lines.insert(20, P.encode_event("homed"))
    tr = ScriptedTransport(b"".join(lines), chunk=13)
    rd = TelemetryReader(tr)
    rd.start()
    assert _wait(lambda: len(rd.buffer) == 99)
    rd.stop()
    assert rd.dropped == 1
    assert rd.parse_errors == 1
    assert rd.messages.get_nowait() == P.Event("homed", {})
    t = rd.buffer.latest()[:, COLUMNS.index("t")]
    assert np.all(np.diff(t) > 0)


def test_reader_sends_commands():
    tr = ScriptedTransport(b"")
    rd = TelemetryReader(tr)
    rd.start()
    rd.send("start")
    rd.stop()
    assert tr.written == b'{"cmd":"start"}\n'


# ------------------------------------------------------------------ sanal cihaz
def _drain(rd):
    out = []
    while not rd.messages.empty():
        out.append(rd.messages.get_nowait())
    return out


def test_mock_device_offline_homing_swingup_balance():
    """Gerçek zaman beklemeden (realtime=False) firmware mantığının tam senaryosu."""
    out = []
    dev = MockDevice(seed=1, realtime=False)
    dev.output = out.append
    dev.start()
    dev.feed(P.encode_command("start"))
    dev.step()
    assert any(b'"ok":false' in o and b"start" in o for o in out)     # homing yapılmadan başlamaz
    dev.feed(P.encode_command("home"))
    for _ in range(int(15 / dev.Ts)):
        dev.step()
        if dev.homed:
            break
    assert dev.homed and abs(dev.true_state[0]) < 0.01
    dev.feed(P.encode_command("start"))
    for _ in range(int(15 / dev.Ts)):
        dev.step()
    assert dev.sup.mode.value == "BALANCE"
    assert abs(math.remainder(dev.true_state[2], 2 * math.pi)) < math.radians(2)
    # dürt → denge bozulmamalı (küçük dürtme)
    dev.feed(P.encode_command("kick", dth=0.5))
    for _ in range(int(3 / dev.Ts)):
        dev.step()
    assert dev.sup.mode.value == "BALANCE"
    dev.feed(P.encode_command("estop"))
    dev.step()
    assert dev.sup.mode.value == "FAULT"


def test_realtime_mock_through_reader_rate_and_no_drops():
    tr = SimTransport(MockDevice(seed=2, telemetry_hz=500))
    rd = TelemetryReader(tr)
    rd.start()
    time.sleep(2.0)
    rate, n, dropped, errs = rd.rate_hz, len(rd.buffer), rd.dropped, rd.parse_errors
    rd.stop()
    assert errs == 0 and dropped == 0
    assert n > 600                     # 2 s × 500 Hz ≈ 1000; yüklü makinede zamanlayıcı payı bırakıldı
    assert rate > 300
    assert any(isinstance(m, P.Event) and m.name == "boot" for m in _drain(rd))


def test_recorder_writes_csv(tmp_path):
    rec = Recorder(tmp_path / "a.csv", meta={"not": "test"})
    rec.write_rows([(0.0, 0.1, 0, 0.01, 0, 1.0, 4.0)])
    rec.close()
    text = (tmp_path / "a.csv").read_text(encoding="utf-8").splitlines()
    assert text[0] == "# not: test"
    assert text[1] == ",".join(COLUMNS)
    assert text[2].endswith("BALANCE")


def test_gains_header_contains_K():
    h = gains_header(PendulumParams(), [-1, -2, -3, -4], 0.002)
    assert "LQR_K[4] = {-1.000000f, -2.000000f, -3.000000f, -4.000000f}" in h
    assert "#define CTRL_RATE_HZ       500" in h


# ------------------------------------------------------------------ inceleme sonrası eklenen testler
import re
from pathlib import Path

from pendulum_lab.comm.reader import mode_name

FIRMWARE = Path(__file__).resolve().parents[1] / "firmware" / "esp32_pendulum" / "esp32_pendulum.ino"


def _offline(**kw):
    out = []
    dev = MockDevice(seed=3, realtime=False, **kw)
    dev.output = out.append
    dev.start()
    return dev, out


def _run(dev, seconds, until=None):
    for _ in range(int(seconds / dev.Ts)):
        dev.step()
        if until and until():
            return True
    return until() if until else True


def _acks(out, cmd):
    msgs = [P.parse_line(o) for o in out]
    return [m for m in msgs if isinstance(m, P.Ack) and m.cmd == cmd]


def _events(out, name):
    msgs = [P.parse_line(o) for o in out]
    return [m for m in msgs if isinstance(m, P.Event) and m.name == name]


def test_mode_name_unknown_is_not_fault():
    assert mode_name(-1.0) == "?"
    assert mode_name(99) == "?"
    assert mode_name(4.0) == "BALANCE"


def test_ack_carries_extra_data():
    a = P.parse_line(P.encode_ack("ping", True, "pong", Ts=0.002, fw="x"))
    assert a.data == {"Ts": 0.002, "fw": "x"} and a.msg == "pong"


def test_ping_reports_Ts():
    dev, out = _offline()
    dev.feed(P.encode_command("ping"))
    dev.step()
    assert _acks(out, "ping")[0].data["Ts"] == dev.Ts
    assert _events(out, "boot")[0].data["Ts"] == dev.Ts


def test_set_param_is_atomic_and_checks_hysteresis():
    dev, out = _offline()
    before = (dev.sup.catch, dev.sup.swingup.k_E)
    dev.feed(b'{"cmd":"set_param","catch_deg":10,"k_E":5,"x_ref":0.9}\n')   # x_ref aralık dışı
    dev.step()
    assert _acks(out, "set_param")[-1].ok is False
    assert (dev.sup.catch, dev.sup.swingup.k_E) == before                   # hiçbiri uygulanmadı
    dev.feed(b'{"cmd":"set_param","catch_deg":40}\n')                       # release=30 → histerezis bozulur
    dev.step()
    assert _acks(out, "set_param")[-1].ok is False
    with pytest.raises(P.ProtocolError):
        P.encode_command("set_param", catch_deg=20, release_deg=10)
    with pytest.raises(P.ProtocolError):
        P.encode_command("set_param", x_ref=0.9)


def test_mock_u_max_limited_to_supply():
    dev, out = _offline()
    dev.feed(P.encode_command("set_param", u_max=20))
    dev.step()
    assert _acks(out, "set_param")[-1].ok is False


def test_homing_when_already_on_left_switch():
    dev, out = _offline(x0=-0.4)
    dev.feed(P.encode_command("home"))
    assert _run(dev, 10, until=lambda: dev.homed)
    assert abs(dev.true_state[0]) < 0.01


def test_homing_faults_on_wrong_side_switch(monkeypatch):
    """Motor/enkoder yönü ters olursa araba sağa gidip sağ switch'e basar → hemen FAULT, ray sonuna yüklenmez."""
    import pendulum_lab.comm.mock_device as md
    monkeypatch.setattr(md, "HOME_SPEED", -0.10)          # ters bağlanmış motoru taklit et
    dev, out = _offline(x0=0.2)
    dev.feed(P.encode_command("home"))
    _run(dev, 5.0, until=lambda: dev.sup.mode.value == "FAULT")
    assert dev.sup.mode.value == "FAULT"
    assert [e.data["reason"] for e in _events(out, "fault")] == ["homing_wrong_side"]


def test_estop_emits_single_fault_event_and_blocks_start():
    dev, out = _offline()
    dev.feed(P.encode_command("home"))
    assert _run(dev, 15, until=lambda: dev.homed)
    dev.feed(P.encode_command("start"))
    _run(dev, 0.5)
    dev.feed(P.encode_command("estop"))
    _run(dev, 0.5)
    assert [e.data["reason"] for e in _events(out, "fault")] == ["estop"]
    dev.feed(P.encode_command("start"))
    dev.step()
    assert _acks(out, "start")[-1].ok is False                              # hatadan doğrudan başlamaz
    dev.feed(P.encode_command("stop") + P.encode_command("start"))
    dev.step()
    assert _acks(out, "start")[-1].ok is True


def test_reader_clears_buffer_on_boot_and_survives_sink_error():
    class BadSink:
        def write_rows(self, rows):
            raise OSError("disk dolu")
    lines = [P.encode_telemetry(n, n * 0.005, 0, 0, 0, 0, 0, "IDLE") for n in range(20)]
    lines.append(P.encode_event("boot", fw="x"))
    lines += [P.encode_telemetry(n, n * 0.005, 0, 0, 0, 0, 0, "IDLE") for n in range(5)]
    rd = TelemetryReader(ScriptedTransport(b"".join(lines), chunk=40))
    bad = BadSink()
    rd.sinks.append(bad)
    rd.start()
    assert _wait(lambda: rd.rows_total == 25)
    rd.stop()
    assert rd.error is None and bad not in rd.sinks
    assert len(rd.buffer) == 5 and rd.dropped == 0
    msgs = []
    while not rd.messages.empty():
        msgs.append(rd.messages.get_nowait())
    assert any(isinstance(m, P.Event) and m.name == "recorder_error" for m in msgs)


def test_recorder_as_reader_sink(tmp_path):
    lines = [P.encode_telemetry(n, n * 0.005, 0.1, 0, 0.01, 0, 1.0, "BALANCE") for n in range(30)]
    rd = TelemetryReader(ScriptedTransport(b"".join(lines)))
    rec = Recorder(tmp_path / "s.csv")
    rd.sinks.append(rec)
    rd.start()
    assert _wait(lambda: rec.rows == 30)
    rd.stop()
    rec.close()
    assert len((tmp_path / "s.csv").read_text(encoding="utf-8").splitlines()) == 31


def test_recorder_never_overwrites(tmp_path):
    a = Recorder.default_path(tmp_path)
    Recorder(a).close()
    b = Recorder.default_path(tmp_path)
    assert a != b


def test_gains_header_rejects_fractional_ms():
    with pytest.raises(ValueError):
        gains_header(PendulumParams(), [0, 0, 0, 0], 0.0025)


def test_firmware_param_limits_match_python():
    src = FIRMWARE.read_text(encoding="utf-8")
    table = dict((k, (float(lo), float(hi))) for k, lo, hi in
                 re.findall(r'\{"(\w+)",\s*([-\d.]+)f,\s*([-\d.]+)f\}', src))
    assert table == {k: (float(lo), float(hi)) for k, (lo, hi) in P.SET_PARAM_LIMITS.items()}


def test_firmware_handles_every_protocol_command():
    src = FIRMWARE.read_text(encoding="utf-8")
    handled = set(re.findall(r'strcmp\(cmd, "(\w+)"\)', src))
    assert set(P.COMMANDS) - {"kick"} == handled          # kick yalnız simülasyonda
