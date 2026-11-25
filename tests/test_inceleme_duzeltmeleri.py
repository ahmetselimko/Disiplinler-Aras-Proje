"""Bağımsız inceleme (22.11.2026) bulgularının testleri: bağlantı bekçisi, bozuk boot satırı, ayarların
yeniden yüklenmesi, arayüzde yarım gönderim, sanal ESP32 ile firmware arasındaki farklar."""

import os
import re
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from pendulum_lab import PendulumParams
from pendulum_lab.comm import protocol as P
from pendulum_lab.comm.mock_device import MockDevice
from pendulum_lab.comm.reader import TelemetryReader
from pendulum_lab.export import gains_header

ROOT = Path(__file__).resolve().parents[1]
INO = (ROOT / "firmware" / "esp32_pendulum" / "esp32_pendulum.ino").read_text(encoding="utf-8")


def _offline(**kw):
    out = []
    dev = MockDevice(seed=3, realtime=False, **kw)
    dev.output = out.append
    dev.start()
    return dev, out


def _run(dev, seconds, hb_every=None):
    for i in range(int(seconds / dev.Ts)):
        if hb_every and i % int(hb_every / dev.Ts) == 0:
            dev.feed(P.encode_command("hb"))
        dev.step()


def _home(dev, hb_every=None):
    dev.feed(P.encode_command("home"))
    for i in range(int(20 / dev.Ts)):
        if hb_every and i % int(hb_every / dev.Ts) == 0:
            dev.feed(P.encode_command("hb"))
        dev.step()
        if dev.homed:
            return
    raise AssertionError("homing bitmedi")


# ------------------------------------------------------------------ bağlantı bekçisi (hb)
def test_link_watchdog_faults_when_pc_goes_silent():
    dev, out = _offline(link_timeout=P.LINK_TIMEOUT_S)
    _home(dev, hb_every=P.HB_PERIOD_S)
    dev.feed(P.encode_command("start"))
    _run(dev, 3.0, hb_every=P.HB_PERIOD_S)                      # hb akarken FAULT yok
    assert dev.sup.mode.value in ("SWING_UP", "BALANCE")
    _run(dev, P.LINK_TIMEOUT_S + 0.1)                           # PC sustu
    assert dev.sup.mode.value == "FAULT"
    assert any(b'"reason":"link"' in o for o in out)


def test_link_watchdog_ignores_idle_and_is_off_by_default():
    dev, out = _offline(link_timeout=P.LINK_TIMEOUT_S)
    _run(dev, 3.0)                                              # IDLE: denetlenmez
    assert dev.sup.mode.value == "IDLE"
    dev2, _ = _offline()                                        # varsayılan kapalı (adım adım testler)
    _home(dev2)
    dev2.feed(P.encode_command("start"))
    _run(dev2, 3.0)
    assert dev2.sup.mode.value != "FAULT"


def test_hb_gets_no_reply_and_is_not_recorded():
    dev, out = _offline()
    n0 = len(out)
    dev.feed(P.encode_command("hb"))
    dev.step()
    assert not any(b'"ack"' in o for o in out[n0:])

    written, events = [], []
    rd = TelemetryReader(SimpleNamespace(write=written.append))
    rd.sinks.append(SimpleNamespace(write_event=lambda kind, data: events.append(data.get("cmd"))))
    rd.send("hb")
    assert written == [b'{"cmd":"hb"}\n']
    assert not rd._pending_commands and events == []


def test_firmware_link_watchdog_matches_protocol():
    m = re.search(r"LINK_TIMEOUT_MS = (\d+);", INO)
    assert m and int(m.group(1)) == round(P.LINK_TIMEOUT_S * 1000)
    assert 'strcmp(cmd, "hb")' in INO
    assert 'raiseFault(mode, "link")' in INO
    assert "SET_SHARED(last_rx_ms, millis())" in INO


# ------------------------------------------------------------------ bozuk boot satırı
def test_reader_recovers_boot_line_after_rom_garbage():
    rd = TelemetryReader(SimpleNamespace(write=lambda d: None))
    rd._handle_lines([P.encode_telemetry(5, 1.0, 0, 0, 0, 0, 0, "IDLE"),
                      b"\x00\x80\xff\xe0" + P.encode_event("boot", fw="ref-1.1", Ts=0.002)])
    msgs = []
    while not rd.messages.empty():
        msgs.append(rd.messages.get_nowait())
    assert any(isinstance(m, P.Event) and m.name == "boot" for m in msgs)
    assert rd.parse_errors == 0


def test_firmware_boot_line_and_first_telemetry():
    i = INO.index('Serial.printf("{\\"evt\\":\\"boot\\"')
    assert 'Serial.print("\\n")' in INO[i - 200:i]               # boot'tan önce boş satır
    assert "s.tel_valid && s.n != last_n" in INO                  # açılıştaki sıfır yapı gönderilmez
    assert "S.tel_valid = true" in INO


# ------------------------------------------------------------------ ayarların yeniden yüklenmesi
class _Reader:
    """Gönderilenleri saklar; geçersiz komutu gerçek okuyucu gibi encode_command'da reddeder."""
    def __init__(self, transport=None):
        self.transport = transport or SimpleNamespace(name="COM3")
        self.sent = []

    def send(self, cmd, **fields):
        P.encode_command(cmd, **fields)
        self.sent.append((cmd, fields))


def _state(placeholder=False):
    from gui.state import AppState
    st = AppState("yok/params.json")
    st.p = PendulumParams(placeholder=placeholder)
    return st


def test_reconnect_within_one_second_still_pushes_settings():
    st = _state()
    st.set_reader(_Reader())
    st.on_device_info({"Ts": 0.002})
    first = len(st.reader.sent)
    assert first > 0
    st.set_reader(None)
    st.set_reader(_Reader())                                       # hemen yeniden bağlan
    st.on_device_info({"Ts": 0.002})
    assert len(st.reader.sent) == first


def test_boot_always_pushes_but_ping_after_boot_does_not_repeat():
    st = _state()
    st.set_reader(_Reader())
    st.on_device_info({"Ts": 0.002}, boot=True)
    n = len(st.reader.sent)
    st.on_device_info({"Ts": 0.002})                               # bağlanırken gelen ping yanıtı
    assert len(st.reader.sent) == n
    st.on_device_info({"Ts": 0.002}, boot=True)                    # kart hemen yeniden başladı
    assert len(st.reader.sent) == 2 * n


def test_push_continues_after_one_rejected_field():
    st = _state()
    st.p.u_max = 50.0                                              # protokol: 0–48 → yerelde reddedilir
    st.x_ref = 0.1
    st.set_reader(_Reader())
    assert st.push_to_device() is False
    sent_keys = {k for _, f in st.reader.sent for k in f}
    assert "x_ref" in sent_keys and "fc_volt" in sent_keys and "u_max" not in sent_keys


def test_clamp_xref_respects_protocol_limit():
    st = _state()
    st.p.rail_half = 1.0
    assert st.clamp_xref(0.8) == pytest.approx(P.SET_PARAM_LIMITS["x_ref"][1])


# ------------------------------------------------------------------ arayüz
@pytest.fixture(scope="module")
def qapp():
    os.environ["QT_QPA_PLATFORM"] = "offscreen"
    from PyQt5 import QtWidgets
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(["test"])
    yield app


def test_design_error_blocks_send(qapp, monkeypatch):
    from PyQt5 import QtWidgets
    from gui import design_tab
    st = _state()
    st.set_reader(_Reader(SimpleNamespace(name="sim")))
    tab = design_tab.DesignTab(st)
    monkeypatch.setattr(QtWidgets.QMessageBox, "warning", staticmethod(lambda *a, **k: None))
    called = []
    monkeypatch.setattr(design_tab, "push_with_confirm", lambda *a: called.append(1))
    tab.w_release.setValue(tab.w_catch.value())                    # histerezis hatası
    assert tab.calculate() is False
    tab.send()
    assert called == []


def test_freeze_keeps_mode_label_live(qapp):
    from gui.live_tab import LiveTab
    st = _state()
    from gui.live_tab import MODE_TR
    tab = LiveTab(st)
    tab.refresh_summary = lambda reader, force=False: None         # yalnız mod etiketi denetleniyor
    tab.pause_cb.setChecked(True)                                  # "Dondur"
    last = P.Telemetry(n=1, t=1.0, x=0.0, dx=0.0, th=0.0, dth=0.0, u=0.0, mode="FAULT")
    tab.refresh(SimpleNamespace(last=last, rate_hz=200.0))
    assert tab.mode_lbl.text() == MODE_TR["FAULT"]                # acil stop dondurulmuş ekranda da görünür


# ------------------------------------------------------------------ sanal ESP32 ↔ firmware
def test_mock_sends_calib_event_at_boot_and_after_calib():
    dev, out = _offline()
    assert any(b'"evt":"calib"' in o for o in out)
    n0 = len(out)
    dev.feed(P.encode_command("calib"))
    dev.step()
    new = out[n0:]
    assert new[0].startswith(b'{"ack":"calib","ok":true')
    assert any(b'"evt":"calib"' in o for o in new[1:])


def test_new_home_clears_homed_in_mock_and_firmware():
    dev, _ = _offline()
    _home(dev)
    dev.feed(P.encode_command("home"))
    dev.step()
    dev.feed(P.encode_command("stop"))                             # homing yarıda kaldı
    dev.step()
    assert dev.homed is False
    dev.feed(P.encode_command("start"))
    out = []
    dev.output = out.append
    dev.step()
    assert any(b'"ack":"start","ok":false' in o for o in out)
    assert "homed = false; SET_SHARED(homed, false);" in INO


def test_firmware_limit_debounce_and_live_state_checks():
    assert re.search(r"LIM_DEBOUNCE = \d+;", INO)
    assert "limL_cnt >= LIM_DEBOUNCE" in INO
    i, j = INO.index("static void handleCommand"), INO.index("void commTask")
    h = INO[i:j]
    for stale in ("fabsf(s.x)", "fabsf(s.dx)", "wrapAngle(s.th)", "fabsf(s.dth)"):
        assert stale not in h                                      # komut kontrolleri bayat örneği kullanmaz
    assert "S.lx = x; S.ldx = dx; S.lth = th; S.ldth = dth;" in INO


# ------------------------------------------------------------------ dışa aktarma
def test_gains_header_without_swingup_still_has_su_defines():
    h = gains_header(PendulumParams(), [0, 0, 0, 0], 0.002)
    for name in ("SU_K_E", "SU_A_MAX", "SU_K_X", "SU_K_V", "SU_A_KICK", "SU_E_RATIO"):
        assert f"#define {name}" in h


def test_gains_header_rejects_ts_above_20ms():
    with pytest.raises(ValueError):
        gains_header(PendulumParams(), [0, 0, 0, 0], 0.05)
    gains_header(PendulumParams(), [0, 0, 0, 0], 0.02)
    assert "CTRL_PERIOD_MS > 20" in INO


# ------------------------------------------------------------------ kayıt oynatma
def test_replay_warns_about_kick_during_trial(tmp_path):
    from pendulum_lab.recorder import _sim_actions_between
    session = SimpleNamespace(events=[
        {"_row": 5, "kind": "command_sent", "cmd": "kick"},
        {"_row": 6, "kind": "command_sent", "cmd": "pause"},
        {"_row": 1, "kind": "command_sent", "cmd": "reset"},        # denemeden önce: sayılmaz
        {"_row": 7, "kind": "command_sent", "cmd": "set_param"},
    ])
    assert _sim_actions_between(session, 2, 10) == ["dürtme", "duraklatma"]


# ------------------------------------------------------------------ ikinci tur (dış inceleme)
def test_heartbeat_comes_from_reader_thread():
    """hb okuyucu thread'inden gider: arayüzün ana thread'i donsa da cihaz motoru kesmez."""
    sent = []

    class _Tr:
        is_open = True

        def read(self, timeout):
            time.sleep(timeout)
            return b""

        def write(self, data):
            sent.append(data)

        def close(self):
            pass

    rd = TelemetryReader(_Tr(), heartbeat_s=0.05)
    rd.start()
    time.sleep(0.4)                                                # ana thread "donmuş"
    rd.stop()
    assert sent.count(b'{"cmd":"hb"}\n') >= 3


def test_ack_after_corrupted_line_does_not_shift():
    rd = TelemetryReader(SimpleNamespace(write=lambda d: None), u_max=12.0)
    rd.send("set_param", x_ref=0.1)                                # satır yolda bozuldu
    rd._handle_lines([P.encode_ack("?", False, "json")])
    rd.send("set_param", u_max=5.0)
    rd._handle_lines([P.encode_ack("set_param")])
    assert rd.applied_settings == {"u_max": 5.0}
    assert not rd._pending_commands


def test_lost_ack_is_skipped_not_shifted():
    from collections import deque
    from pendulum_lab.comm.reader import match_ack
    q = deque([("start", {}), ("set_param", {"a": 1}), ("set_param", {"b": 2})])
    assert match_ack(q, "set_param") == ("set_param", {"a": 1})    # start'ın yanıtı kayboldu
    assert match_ack(q, "set_param") == ("set_param", {"b": 2})
    assert not q


def test_command_recorded_before_it_is_written():
    order = []
    rd = TelemetryReader(SimpleNamespace(write=lambda d: order.append("write")))
    rd.sinks.append(SimpleNamespace(write_event=lambda kind, data: order.append(kind)))
    rd.send("start")
    assert order == ["command_sent", "write"]


def test_homing_with_reversed_encoder_faults_slowly_not_at_full_speed():
    from pendulum_lab import load_params
    dev = MockDevice(load_params(ROOT / "config" / "params.json"), realtime=False, seed=1, enc_sign=-1)
    out = []
    dev.output = out.append
    dev.start()
    dev.feed(P.encode_command("home"))
    vmax = 0.0
    for _ in range(int(20 / dev.Ts)):
        dev.step()
        vmax = max(vmax, abs(dev.true_state[1]))
        if dev.sup.mode.value == "FAULT":
            break
    assert dev.sup.mode.value == "FAULT" and not dev.homed
    assert vmax < 0.3
    assert 'raiseFault(mode, "homing_runaway")' in INO


def test_live_push_recomputes_k(qapp, monkeypatch):
    from gui.main_window import MainWindow
    st = _state()
    win = MainWindow(st)
    calls = []
    monkeypatch.setattr(win.design, "calculate", lambda: calls.append(1) or False)
    win.live.b_push.clicked.emit(False)                            # buton bağlı değilken pasif; sinyal yeterli
    assert calls == [1]
    win.timer.stop()
    win.close()


# ------------------------------------------------------------------ dosya hataları uygulamayı kapatmasın
def test_record_start_failure_is_reported_not_raised(qapp, monkeypatch):
    from gui import live_tab
    st = _state()
    st.set_reader(SimpleNamespace(transport=SimpleNamespace(name="sim"), sinks=[], applied_settings={}))
    tab = live_tab.LiveTab(st)

    class _FailingRecorder:                                         # kayıt klasörüne yazılamıyor
        default_path = staticmethod(lambda *a, **k: Path("x.csv"))

        def __init__(self, *a, **k):
            raise PermissionError("yazma izni yok")
    monkeypatch.setattr(live_tab, "Recorder", _FailingRecorder)
    logs = []
    st.log.connect(logs.append)
    tab.b_rec.setChecked(True)
    tab.toggle_record(True)
    assert st.recorder is None and not tab.b_rec.isChecked()
    assert any("Kayıt başlatılamadı" in m for m in logs)


def test_save_and_export_failures_show_warning(qapp, monkeypatch, tmp_path):
    from PyQt5 import QtWidgets
    from gui import design_tab
    st = _state()
    tab = design_tab.DesignTab(st)
    warned = []
    monkeypatch.setattr(QtWidgets.QMessageBox, "warning", staticmethod(lambda *a, **k: warned.append(a[1])))
    monkeypatch.setattr(QtWidgets.QFileDialog, "getSaveFileName", staticmethod(lambda *a, **k: (str(tmp_path / "x"), "")))

    def boom(*a, **k):
        raise PermissionError("yazma izni yok")
    monkeypatch.setattr(design_tab, "save_params", boom)
    monkeypatch.setattr(design_tab, "write_gains_header", boom)
    tab.save()
    tab.export()
    assert warned == ["Kaydedilemedi", "gains.h yazılamadı"]


def test_excepthook_keeps_app_alive(qapp, monkeypatch):
    import sys
    from PyQt5 import QtWidgets
    from gui.widgets import install_excepthook
    shown = []
    monkeypatch.setattr(QtWidgets.QMessageBox, "exec_", lambda self: shown.append(self.text()))
    old = sys.excepthook
    try:
        install_excepthook(qapp)
        try:
            raise RuntimeError("deneme")
        except RuntimeError:
            sys.excepthook(*sys.exc_info())
    finally:
        sys.excepthook = old
    assert shown and "RuntimeError: deneme" in shown[0]


def test_excepthook_shows_one_window_at_a_time(qapp, monkeypatch):
    """Pencere açıkken (iç olay döngüsünde) tekrar eden hatalar yeni pencere açmamalı."""
    import sys
    from PyQt5 import QtWidgets
    from gui.widgets import install_excepthook
    opened = []

    def fake_exec(self):
        opened.append(self.text())
        if len(opened) == 1:                                       # pencere açıkken _tick yine hata veriyor
            for _ in range(5):
                try:
                    raise RuntimeError("tekrar")
                except RuntimeError:
                    sys.excepthook(*sys.exc_info())
    monkeypatch.setattr(QtWidgets.QMessageBox, "exec_", fake_exec)
    old = sys.excepthook
    try:
        install_excepthook(qapp)
        for _ in range(2):                                         # kapandıktan sonraki hata yine gösterilir
            try:
                raise RuntimeError("ilk")
            except RuntimeError:
                sys.excepthook(*sys.exc_info())
    finally:
        sys.excepthook = old
    assert len(opened) == 2


def test_excepthook_logs_every_error_to_panel_and_file(qapp, monkeypatch, tmp_path):
    """Pencere açıkken gelen hatalar da kaybolmamalı: log alanına satır, dosyaya ayrıntı, kapanınca özet."""
    import sys
    from PyQt5 import QtWidgets
    from gui.widgets import install_excepthook
    logs, opened = [], []
    log_path = tmp_path / "kayitlar" / "hatalar.log"

    def raise_and_hook(msg):
        try:
            raise RuntimeError(msg)
        except RuntimeError:
            sys.excepthook(*sys.exc_info())

    def fake_exec(self):
        opened.append(1)
        raise_and_hook("ikinci")
        raise_and_hook("üçüncü")
    monkeypatch.setattr(QtWidgets.QMessageBox, "exec_", fake_exec)
    old = sys.excepthook
    try:
        install_excepthook(qapp, log=logs.append, log_path=log_path)
        raise_and_hook("ilk")
    finally:
        sys.excepthook = old
    assert opened == [1]
    text = log_path.read_text(encoding="utf-8")
    assert all(m in text for m in ("ilk", "ikinci", "üçüncü"))
    assert any("ikinci" in m for m in logs) and any("üçüncü" in m for m in logs)
    assert any("2 hata daha" in m for m in logs)


# ------------------------------------------------------------------ üçüncü tur (bağımsız inceleme, 23.11.2026)
def test_swingup_flag_reaches_real_board_with_placeholder_params():
    """Yer tutucu parametrelerle K gerçek karta gitmez ama swing-up kapalı bilgisi HER boot'ta gitmeli
    (firmware reset sonrası varsayılan 'açık'a döner; ilk donanım denemesi swing-up kapalı yapılır)."""
    st = _state(placeholder=True)
    st.swingup_enabled = False
    st.set_reader(_Reader(SimpleNamespace(name="COM3")))
    st.on_device_info({"Ts": 0.002}, boot=True)
    st.on_device_info({"Ts": 0.002}, boot=True)                  # kart resetlendi
    assert st.reader.sent == [("set_param", {"swingup": 0})] * 2


def test_replay_summary_shows_how_trial_ended(tmp_path):
    from pendulum_lab.recorder import compare_recording_with_simulation, load_recording
    from pendulum_lab.recorder import Recorder
    from gui.state import AppState
    import copy
    st = AppState(ROOT / "config" / "params.json")
    dev = MockDevice(params=copy.deepcopy(st.p), seed=0, realtime=False)
    dev.sup.set_gain(st.K)
    out = []
    dev.output = out.append
    dev.start()
    _home(dev)
    dev.feed(P.encode_command("start"))
    _run(dev, 7.0)
    dev.feed(P.encode_command("estop"))
    _run(dev, 0.5)
    rd = TelemetryReader(SimpleNamespace(write=lambda d: None))
    rec = Recorder(tmp_path / "f.csv", meta={"kaynak": "test", **st.get_metadata()})
    rd.sinks.append(rec)
    rd._handle_lines(b"".join(out).splitlines(keepends=True))
    rec.close()
    comp = compare_recording_with_simulation(load_recording(tmp_path / "f.csv"), fallback_state=st)
    assert comp.rec_summary.final_mode == "FAULT"


def test_exact_value_survives_tie_rounding(qapp):
    from gui import design_tab
    st = _state()
    st.p.M = 0.0625                                                # kutu 3 ondalık: Qt 0.063 gösterir
    tab = design_tab.DesignTab(st)
    assert tab._params_from_widgets().M == 0.0625


def test_friction_sync_keeps_unsaved_motor_edits(qapp):
    from gui import design_tab
    st = _state()
    tab = design_tab.DesignTab(st)
    tab.w_mot["R"].setValue(3.0)                                   # kaydedilmemiş motor düzenlemesi
    st.p.motor.T_f = 0.002
    st.params_replaced.emit()
    assert tab.w_mot["R"].value() == 3.0
    assert abs(tab._val(tab.w_mot["T_f"]) - 0.002) < 1e-12


def test_mock_start_check_uses_measured_position():
    dev, out = _offline(limit_switches=False, enc_sign=-1, x0=0.16)
    dev.feed(P.encode_command("home"))                            # elle ortalama: ölçülen x = 0 olur
    dev.step()
    dev.feed(P.encode_command("start"))
    dev.step()
    assert any(b'"ack":"start","ok":true' in o for o in out)

def test_mock_u_max_applies_to_controller_after_fc_volt():
    dev, _ = _offline()
    for f in (dict(u_max=12.0), dict(fc_volt=0.0), dict(u_max=3.0)):
        dev.feed(P.encode_command("set_param", **f))
        dev.step()
    assert dev.p.u_max == 3.0 and dev.sup.p.u_max == 3.0


def test_mock_survives_malformed_sim_command():
    dev, out = _offline()
    dev.feed(b'{"cmd":"kick","F":null}\n')
    dev.step()
    assert any(b'"ack":"?","ok":false' in o for o in out)
    n = len(out)
    dev.feed(P.encode_command("ping"))
    dev.step()
    assert any(b'"ack":"ping","ok":true' in o for o in out[n:])


def test_swingup_off_during_swingup_switches_to_wait():
    from pendulum_lab import load_params
    dev, _ = _offline()
    _home(dev)
    dev.feed(P.encode_command("start"))
    _run(dev, 0.2)
    assert dev.sup.mode.value == "SWING_UP"
    dev.feed(P.encode_command("set_param", swingup=0))
    _run(dev, 0.05)
    assert dev.sup.mode.value == "WAIT"
    dev.feed(P.encode_command("set_param", swingup=1))
    _run(dev, 0.05)
    assert dev.sup.mode.value == "SWING_UP"
    assert "else if (mode != BALANCE) mode = p.swingup_on ? SWING_UP : WAIT;" in INO


def test_push_refused_for_non_voltage_input():
    st = _state()
    st.p.input_mode = "force"
    st.set_reader(_Reader())
    assert st.push_to_device() is False
    assert st.reader.sent == [("set_param", {"swingup": 1})]       # K gitmez; yalnız swing-up anahtarı


def test_design_tab_keeps_exact_file_values(qapp):
    from gui import design_tab
    from pendulum_lab import load_params
    st = _state()
    st.p = load_params(ROOT / "config" / "params.json")
    st.p.c = 2.4e-5
    tab = design_tab.DesignTab(st)
    p = tab._params_from_widgets()
    assert p.motor.r_pulley == st.p.motor.r_pulley == 0.009549
    assert p.c == 2.4e-5
    tab.w_par["c"].setValue(0.0005)                                # kullanıcı değiştirirse yeni değer
    assert tab._params_from_widgets().c == 0.0005


def test_friction_result_does_not_apply_unsaved_design_edits(qapp):
    from gui import design_tab
    st = _state()
    tab = design_tab.DesignTab(st)
    K0 = st.K.copy()
    tab.w_Q[2].setValue(500.0)                                     # kaydedilmemiş düzenleme (K Hesapla yok)
    st.p.motor.T_f = 0.002
    st.params_replaced.emit()
    assert st.Q[2] != 500.0
    assert abs(tab._val(tab.w_mot["T_f"]) - 0.002) < 1e-12
    assert (abs(st.K - K0) < 1e-9).all()                           # T_f LQR'ı değiştirmez


def test_status_bar_marks_k_not_on_device(qapp):
    from gui.main_window import MainWindow
    st = _state()
    win = MainWindow(st)
    st.set_reader(SimpleNamespace(applied_settings={"K": [0.0, 0.0, 0.0, 0.0]}, error=None))
    win._update_k_label()
    assert "gönderilmedi" in win.s_K.text()
    st.reader.applied_settings = {"K": list(st.K)}
    win._update_k_label()
    assert "gönderilmedi" not in win.s_K.text()
    st.reader = None
    win.timer.stop()
    win.close()


# ------------------------------------------------------------------ beşinci tur (23.11.2026)
def test_out_of_range_file_value_uses_what_the_box_shows(qapp):
    from gui import design_tab
    st = _state()
    st.p.motor.V_max, st.p.u_max, st.p.M = 60.0, 60.0, 0.005     # kutu aralıkları: u_max ≤ 48, M ≥ 0.01
    tab = design_tab.DesignTab(st)
    p = tab._params_from_widgets()
    assert p.u_max == tab.w_par["u_max"].value() == 48.0
    assert p.M == tab.w_par["M"].value()


def test_friction_sync_keeps_edit_made_after_calculate(qapp):
    from gui import design_tab
    st = _state()
    tab = design_tab.DesignTab(st)
    tab.w_mot["R"].setValue(3.0)
    assert tab.calculate()
    tab.w_mot["R"].setValue(4.0)                                   # hesaplamadan sonra yeni, kaydedilmemiş düzenleme
    st.p.motor.T_f = 0.002
    st.params_replaced.emit()
    assert tab.w_mot["R"].value() == 4.0
    assert abs(tab._val(tab.w_mot["T_f"]) - 0.002) < 1e-12


def test_live_target_line_uses_acknowledged_xref():
    src = (ROOT / "gui" / "live_tab.py").read_text(encoding="utf-8")
    assert 'applied_settings.get("x_ref", 0.0)' in src


def test_replay_warns_when_real_board_never_acknowledged_k(tmp_path):
    from pendulum_lab.recorder import compare_recording_with_simulation, load_recording, Recorder
    from gui.state import AppState
    import copy
    st = AppState(ROOT / "config" / "params.json")
    dev = MockDevice(params=copy.deepcopy(st.p), seed=0, realtime=False)
    out = []
    dev.output = out.append
    dev.start()
    _home(dev)
    dev.feed(P.encode_command("start"))
    _run(dev, 2.0)
    for kaynak, expect in (("gerçek kart (COM5)", True), ("simülasyon (sanal ESP32)", False)):
        rd = TelemetryReader(SimpleNamespace(write=lambda d: None))
        path = tmp_path / f"{expect}.csv"
        rec = Recorder(path, meta={"kaynak": kaynak, **st.get_metadata()})
        rd.sinks.append(rec)
        rd._handle_lines(b"".join(out).splitlines(keepends=True))
        rec.close()
        comp = compare_recording_with_simulation(load_recording(path), fallback_state=st)
        assert comp.k_unconfirmed is expect


def test_placeholder_messages_mention_swingup_is_still_sent():
    for f in ("gui/state.py", "gui/connection_panel.py"):
        assert "swing-up açık/kapalı" in (ROOT / f).read_text(encoding="utf-8")


# ------------------------------------------------------------------ tam proje denetimi (25.11.2026)
def test_homing_faults_when_left_sensor_reads_pressed_all_the_time():
    dev, out = _offline(x0=-0.2)
    dev.stuck_switch = "L"                                        # NC/TCST2103'te kopuk kablo = hep "basılı"
    dev.feed(P.encode_command("home"))
    _run(dev, 6.0)
    assert not dev.homed and dev.sup.mode.value == "FAULT"
    assert any(b'"reason":"homing_switch"' in o for o in out)
    assert 'raiseFault(mode, "homing_switch")' in INO and "HOME_SW_CLEAR_M = 0.05f" in INO


def test_homing_still_works_with_healthy_switches():
    dev, out = _offline(x0=-0.2)
    _home(dev)
    assert not any(b'homing_switch' in o for o in out)


def test_ping_reports_angle_calibration_and_gui_warns():
    dev, out = _offline()
    dev.feed(P.encode_command("ping"))
    dev.step()
    assert any(b'"angle_ok":true' in o for o in out)
    assert '\\"angle_ok\\":%s' in INO and "s.angle_ok" in INO
    st = _state()
    logs = []
    st.log.connect(logs.append)
    st.on_device_info({"Ts": 0.002, "fw": "x", "angle_ok": False})
    assert any("açı kalibrasyonu BAŞARISIZ" in m for m in logs)


def test_clamped_xref_is_accepted_for_awkward_rail_lengths():
    import numpy as np
    from pendulum_lab import load_params
    p = load_params(ROOT / "config" / "params.json")
    for rail in (0.302, 0.309, 0.346):
        p.rail_half = rail
        dev = MockDevice(params=p, seed=0, realtime=False)
        out = []
        dev.output = out.append
        dev.start()
        st = _state()
        st.p.rail_half = rail
        x = float(np.float32(st.clamp_xref(1.0)))                 # firmware float32 olarak alır
        dev.feed(P.encode_command("set_param", x_ref=x))
        dev.step()
        assert any(b'"ack":"set_param","ok":true' in o for o in out), rail
    assert "RAIL_HALF_M - 0.1f + 1e-4f" in INO


def test_measured_checkbox_is_fail_safe(qapp, monkeypatch):
    from gui import design_tab
    from PyQt5 import QtWidgets
    monkeypatch.setattr(QtWidgets.QMessageBox, "warning", lambda *a, **k: None)
    st = _state(placeholder=False)
    tab = design_tab.DesignTab(st)
    assert tab.w_measured.isChecked() and st.p.placeholder is False
    tab.w_release.setValue(tab.w_catch.value())                  # hesap başarısız olacak (histerezis yok)
    tab.w_measured.setChecked(False)                              # güvenli yön: hesap başarısız olsa da uygulanır
    assert st.p.placeholder is True
    tab.w_measured.setChecked(True)                               # hesap başarısız → kutu geri alınır
    assert not tab.w_measured.isChecked() and st.p.placeholder is True
