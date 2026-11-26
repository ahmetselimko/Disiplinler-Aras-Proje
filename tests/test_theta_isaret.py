"""#50 Aşama 1: θ "sola yatınca pozitif" oldu; fiziksel davranış değişmemeli.

tests/data/theta_isaret_referans.json eski kodla (θ sağa +) üretildi (tests/theta_senaryolari.py).
Aynı senaryolar yeni kodla, başlangıç θ ve θ̇'sı ters işaretle koşulur: x, ẋ, u aynı; θ, θ̇ ters olmalı.
"""

import json
import math

import numpy as np
import pytest

import theta_senaryolari as S

REF = json.loads(S.REF_PATH.read_text(encoding="utf-8"))

# Sıkı tolerans: denklemler işaret dönüşümünde birebir aynı; yalnız Kalman matrislerinin (expm) son bitleri farklı.
STRICT = dict(x=1e-6, dx=1e-5, th=1e-4, dth=1e-3, u=1e-3)
# Gevşek tolerans: Coulomb sürtünmesi (T_f > 0) dengede ölü bant / takılıp kayma oluşturur ve küçük farkları
# büyütür. ESKİ kodda başlangıç açısı 1e-13 rad değiştirilince de aynı büyüklükte fark çıkıyor (25.11.2026'da
# ölçüldü: x 1.5 mm, ẋ 0.024 m/s, θ 1.3e-3 rad, θ̇ 0.077 rad/s, u 0.93 V; modlar aynı). Bu yüzden aşağıdan
# kalkışta LOOSE_AFTER'dan sonrası gevşek toleransla karşılaştırılır.
LOOSE = dict(x=5e-3, dx=0.06, th=5e-3, dth=0.25, u=2.5)
LOOSE_AFTER = {"asagidan_kalkis": 7.5}
# Kuantalı sensör: iki koşu arasındaki 1 ulp'lik fark (ör. Kalman expm'in son bitleri; işletim sistemine göre değişir)
# ölçülen x ya da θ'yı bir kuantalama adımı kaydırabilir ve u'yu birden değiştirir. Windows'ta dik_arti5/eksi5'te
# 2.9 s'de gözlendi (Linux'ta yok); eski kod aynı makinede referansla 6e-9 m içinde. Bu yüzden gerçek durum farkı
# yarım adımı aştığı andan sonrası gevşek toleransla karşılaştırılır; o ana kadar sıkı ve en az MIN_STRICT_S boyunca.
MIN_STRICT_S = 1.0


def _wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def _check(x_new, x_ref, dx_new, dx_ref, th_new, th_ref, dth_new, dth_ref, u_new, u_ref, t=None, t_loose=None):
    """x, ẋ, u aynı; θ, θ̇ ters. t_loose verilirse t ≥ t_loose için gevşek tolerans (bkz. LOOSE)."""
    a = lambda v: np.asarray(v, dtype=float)
    err = {"x": np.abs(a(x_new) - a(x_ref)), "dx": np.abs(a(dx_new) - a(dx_ref)),
           "th": np.abs(_wrap(a(th_new) + a(th_ref))),              # θ_yeni = −θ_eski
           "dth": np.abs(a(dth_new) + a(dth_ref)), "u": np.abs(a(u_new) - a(u_ref))}
    loose = np.zeros(len(err["x"]), bool) if t_loose is None else a(t) >= t_loose
    flip = np.flatnonzero((err["x"] > 0.5 * S.X_RES) | (err["th"] > 0.5 * S.TH_RES))
    if len(flip) and t is not None:
        assert a(t)[flip[0]] >= MIN_STRICT_S, ("kuantalama adımı çok erken kaydı", a(t)[flip[0]])
        loose[flip[0]:] = True
    for k, e in err.items():
        assert e[~loose].max(initial=0) <= STRICT[k], (k, "sıkı", e[~loose].max())
        assert e[loose].max(initial=0) <= LOOSE[k], (k, "gevşek", e[loose].max())


@pytest.mark.parametrize("name", [c[0] for c in S.CLOSED_LOOP])
def test_kapali_cevrim_fizik_ayni(name):
    ref, new = REF[name], S.run_closed_loop(name, -1.0)
    rs, ns = np.asarray(ref["s"]), np.asarray(new["s"])
    assert new["mode"] == ref["mode"]
    _check(ns[:, 0], rs[:, 0], ns[:, 1], rs[:, 1], ns[:, 2], rs[:, 2], ns[:, 3], rs[:, 3], new["u"], ref["u"],
           t=ref["t"], t_loose=LOOSE_AFTER.get(name))
    # K: x ve ẋ kazançları aynı, θ ve θ̇ kazançları ters işaretli
    np.testing.assert_allclose(new["K"], np.asarray(ref["K"]) * [1, 1, -1, -1], rtol=1e-9, atol=1e-9)


def test_asagidan_kalkis_dengede_bitiyor():
    assert REF["asagidan_kalkis"]["mode"][-1] == "BALANCE"
    assert "SWING_UP" in REF["asagidan_kalkis"]["mode"]


def test_matlab_uyumlu_fizik_ayni():
    ref, new = REF["matlab_uyumlu"], S.run_matlab(-1.0)
    rs, ns = np.asarray(ref["s"]), np.asarray(new["s"])
    assert new["stop"] == ref["stop"]
    _check(ns[:, 0], rs[:, 0], ns[:, 1], rs[:, 1], ns[:, 2], rs[:, 2], ns[:, 3], rs[:, 3], new["u"], ref["u"])


def test_sanal_esp32_homing_ve_start_fizik_ayni():
    ref, new = REF["sanal_esp32"], S.run_mock(-1.0)
    assert ref["homed"] and new["homed"]
    assert new["mode"] == ref["mode"] and ref["mode"][-1] == "BALANCE"
    _check(new["x"], ref["x"], new["dx"], ref["dx"], new["th"], ref["th"], new["dth"], ref["dth"],
           new["u"], ref["u"])


def test_sola_yatik_sarkac_pozitif_aci():
    """Tanım: θ > 0 iken sarkacın ağırlık merkezi arabanın solunda (x − l·sinθ < x)."""
    from pendulum_lab.model.dynamics import accelerations
    p = S.params()
    p.motor.T_f = 0.0
    _, ddth = accelerations(p, [0.0, 0.0, math.radians(5), 0.0], 0.0)
    assert ddth > 0                      # sola yatık sarkaç yerçekimiyle daha da sola düşer
    ddx, ddth = accelerations(p, [0.0, 0.0, 0.0, 0.0], 3.0)
    assert ddx > 0 and ddth > 0          # araba sağa hızlanınca sarkaç sola (θ > 0) yatar


# ---- eski (θ sağa +) CSV kayıtları ----

def _make_old_csv(new_path, old_path):
    """Yeni kuralla alınmış kaydı, eski firmware/arayüz yazsaydı nasıl olacaksa öyle yazar: theta_pozitif
    satırı yok, θ ve θ̇ sütunları ile K'ların θ, θ̇ kazançları ters işaretli."""
    from pendulum_lab.recorder import THETA_SIGN_KEY, _flip_gain
    out = []
    for line in new_path.read_text(encoding="utf-8").splitlines():
        if line.startswith(f"# {THETA_SIGN_KEY}:") or line.startswith("# cihaz_theta_pozitif:"):
            continue                                       # eski arayüz bu alanları yazmazdı
        if line.startswith("# event: "):
            ev = json.loads(line[len("# event: "):])
            if isinstance(ev.get("fields"), dict) and "K" in ev["fields"]:
                ev["fields"]["K"] = _flip_gain(ev["fields"]["K"])
            st = ev.get("settings")
            if isinstance(st, dict):
                st["K"] = _flip_gain(st.get("K"))
                if "K" in (st.get("device_acknowledged_settings") or {}):
                    st["device_acknowledged_settings"]["K"] = _flip_gain(st["device_acknowledged_settings"]["K"])
            line = "# event: " + json.dumps(ev, ensure_ascii=False)
        elif line.startswith("# K: "):
            line = "# K: " + json.dumps(_flip_gain(json.loads(line[5:])))
        elif line.startswith("# device_acknowledged_settings: "):
            ack = json.loads(line.split(": ", 1)[1])
            if "K" in ack:
                ack["K"] = _flip_gain(ack["K"])
            line = "# device_acknowledged_settings: " + json.dumps(ack, ensure_ascii=False)
        elif not line.startswith("#") and line and line[0] in "-0123456789":
            c = line.split(",")
            c[3], c[4] = (f"{-float(v)!r}" for v in (c[3], c[4]))
            line = ",".join(c)
        out.append(line)
    old_path.write_text("\n".join(out) + "\n", encoding="utf-8")


def test_new_recording_header_marks_theta_sign(tmp_path):
    from pendulum_lab.recorder import Recorder
    rec = Recorder(tmp_path / "a.csv", meta={"kaynak": "test"})
    rec.close()
    text = (tmp_path / "a.csv").read_text(encoding="utf-8")
    assert text.splitlines()[0] == "# theta_pozitif: sol" and text.count("theta_pozitif") == 1
    # PC'nin yazdığı alanlar her zaman bu arayüzün kuralında: çağıran başka değer verse de "sol"
    rec = Recorder(tmp_path / "b.csv", meta={"kaynak": "test", "theta_pozitif": "sag", "cihaz_theta_pozitif": "sag"})
    rec.close()
    lines = (tmp_path / "b.csv").read_text(encoding="utf-8").splitlines()
    assert lines[0] == "# theta_pozitif: sol" and "# cihaz_theta_pozitif: sag" in lines
    with pytest.raises(ValueError):
        Recorder(tmp_path / "c.csv", meta={"cihaz_theta_pozitif": "yukari"})
    assert not (tmp_path / "c.csv").exists()


def test_old_recording_is_converted_on_load(tmp_path):
    from gui.state import AppState
    from pendulum_lab.recorder import compare_recording_with_simulation, load_recording
    from test_kalman_and_playback import ROOT, _record_via_reader
    st = AppState(ROOT / "config" / "params.json")
    new_path, old_path = tmp_path / "yeni.csv", tmp_path / "eski.csv"
    _record_via_reader(new_path, st, before_start=[("set_gain", {"K": st.K.tolist()})], run_s=4.0)
    _make_old_csv(new_path, old_path)
    assert "theta_pozitif" not in old_path.read_text(encoding="utf-8").split("# event:")[0]
    new, old = load_recording(new_path), load_recording(old_path)
    assert "theta_isareti_donusturuldu" not in new.meta and old.meta["theta_isareti_donusturuldu"]
    assert old.meta["theta_pozitif"] == "sol"
    assert np.abs(new.th).max() > 0.5                        # kayıtta gerçekten açı değişimi var
    np.testing.assert_allclose(old.th, new.th, atol=1e-5)
    np.testing.assert_allclose(old.dth, new.dth, atol=1e-4)
    np.testing.assert_allclose(old.x, new.x)
    np.testing.assert_allclose(old.meta["K"], new.meta["K"], atol=1e-9)
    c_new = compare_recording_with_simulation(new, fallback_state=st)
    c_old = compare_recording_with_simulation(old, fallback_state=st)
    np.testing.assert_allclose(c_old.K, c_new.K, atol=1e-9)
    assert c_old.rmse_th_deg == pytest.approx(c_new.rmse_th_deg, abs=1e-3)
    assert c_new.rmse_th_deg < 5.0                           # karşılaştırma anlamlı: simülasyon kaydı izliyor


def test_unknown_theta_sign_value_is_rejected(tmp_path):
    from pendulum_lab.recorder import load_recording
    path = tmp_path / "x.csv"
    path.write_text("# theta_pozitif: yukari\nt,x,dx,th,dth,u,mode\n0,0,0,0,0,0,IDLE\n0.01,0,0,0,0,0,IDLE\n",
                    encoding="utf-8")
    with pytest.raises(ValueError, match="theta_pozitif"):
        load_recording(path)
    for bad in ("yukari", "SAG"):
        path.write_text(f"# theta_pozitif: sol\n# cihaz_theta_pozitif: {bad}\nt,x,dx,th,dth,u,mode\n"
                        "0,0,0,0,0,0,IDLE\n0.01,0,0,0,0,0,IDLE\n", encoding="utf-8")
        with pytest.raises(ValueError, match="cihaz_theta_pozitif"):
            load_recording(path)


def test_firmware_uses_left_positive_theta():
    """Firmware'in swing-up ve ivme → gerilim formülleri Python'la aynı (θ sola +) işarette; gains.h yeni K'da."""
    from pathlib import Path
    fw_dir = Path(__file__).resolve().parents[1] / "firmware" / "esp32_pendulum"
    fw = (fw_dir / "esp32_pendulum.ino").read_text(encoding="utf-8")
    assert "float ddth = (ml * P_G * s - P_C * dth + ml * c * a) / P_J;" in fw
    assert "float F = (P_M_CART + P_M_POLE) * a - ml * c * ddth + ml * s * dth * dth + P_B * dx;" in fw
    assert "a = -p.su_kE * (E - E_target) * dth * cosf(th);" in fw
    assert "SOLA" in fw.split("ANGLE_SIGN", 1)[1].split("\n", 1)[0]
    assert "#ifndef THETA_POSITIVE_LEFT\n#error" in fw                 # eski gains.h derlenmesin
    gains = (fw_dir / "gains.h").read_text(encoding="utf-8")
    assert "#define THETA_POSITIVE_LEFT 1" in gains
    from pendulum_lab.export import gains_header
    from pendulum_lab.model.params import PendulumParams
    assert "#define THETA_POSITIVE_LEFT 1" in gains_header(PendulumParams(), [0, 0, 0, 0], 0.002)
    line = next(s for s in gains.splitlines() if s.startswith("static const float LQR_K[4]"))
    K = [float(v.strip().rstrip("f")) for v in line.split("{")[1].split("}")[0].split(",")]
    assert K[2] > 0 and K[3] > 0                 # θ sola +: θ ve θ̇ kazançları pozitif (eski gains.h'de negatifti)


def test_old_recording_sim_commands_with_theta_fields_are_flipped(tmp_path):
    """Eski kayıttaki θ taşıyan komut alanları (eski biçim kick dth, reset th0_deg) da çevrilir."""
    from pendulum_lab.recorder import load_recording
    path = tmp_path / "eski.csv"
    path.write_text(
        '# event: {"kind": "command_sent", "cmd": "kick", "fields": {"dth": 1.5}}\n'
        '# event: {"kind": "command_sent", "cmd": "reset", "fields": {"th0_deg": 170.0}}\n'
        't,x,dx,th,dth,u,mode\n0,0,0,0.1,0.2,0,IDLE\n0.01,0,0,0.1,0.2,0,IDLE\n', encoding="utf-8")
    s = load_recording(path)
    assert [e["fields"] for e in s.events] == [{"dth": -1.5}, {"th0_deg": -170.0}]
    assert s.th[0] == pytest.approx(-0.1) and s.dth[0] == pytest.approx(-0.2)


def test_device_reports_theta_convention_and_old_firmware_gets_no_gain():
    """Firmware ve sanal ESP32 boot/ping'de "theta":"sol" bildirir; bildirmeyen cihaza arayüz K göndermez."""
    from pathlib import Path
    from gui.state import AppState
    from pendulum_lab.comm import protocol as P
    from pendulum_lab.comm.mock_device import MockDevice
    fw = (Path(__file__).resolve().parents[1] / "firmware" / "esp32_pendulum" / "esp32_pendulum.ino").read_text(
        encoding="utf-8")
    assert fw.count('\\"theta\\":\\"sol\\"') == 2 and P.THETA_CONVENTION == "sol"
    out = []
    dev = MockDevice(realtime=False)
    dev.output = out.append
    dev.start()
    dev.feed(P.encode_command("ping"))
    dev.step()
    msgs = [P.parse_line(o) for o in out]
    boot = next(m for m in msgs if isinstance(m, P.Event) and m.name == "boot")
    ping = next(m for m in msgs if isinstance(m, P.Ack) and m.cmd == "ping")
    assert boot.data["theta"] == "sol" and ping.data["theta"] == "sol"

    class _Reader:
        transport = object()
        def __init__(self): self.sent = []
        def send(self, cmd, **f): self.sent.append(cmd)
    st = AppState(Path(__file__).resolve().parents[1] / "config" / "params.json")
    st.p.placeholder = False
    st.set_reader(_Reader())
    assert not st.push_to_device(force=True) and "set_gain" not in st.reader.sent   # cihaz bilgisi gelmeden K yok
    assert not st.push_to_device() and "set_gain" not in st.reader.sent
    for info, gain_sent in (({"fw": "ref-1.1", "Ts": 0.002}, False),
                            ({"Ts": 0.002}, False),
                            ({"fw": "ref-1.2", "theta": "sag", "Ts": 0.002}, False),
                            ({"fw": "ref-1.2", "theta": "sol", "Ts": 0.002}, True)):
        st.set_reader(_Reader())
        logs = []
        st.log.connect(logs.append)
        st.on_device_info(info, boot=True)
        st.log.disconnect(logs.append)
        assert ("set_gain" in st.reader.sent) == gain_sent, info
        assert "set_param" in st.reader.sent                       # swing-up anahtarı her durumda gider
        assert any("θ kuralı" in m for m in logs) == (not gain_sent)
        assert st.get_metadata()["cihaz_theta_pozitif"] == ("sol" if gain_sent else "sag")   # kayıt başlığı
    st.set_reader(_Reader())
    assert st.get_metadata()["cihaz_theta_pozitif"] is None            # bilgi yok: kayıt başlatılmaz (live_tab)
    st.push_to_device()
    assert "ping" in st.reader.sent                                    # bilgi yeniden istendi


def test_new_gui_old_firmware_recording_flips_only_telemetry(tmp_path):
    """Yeni arayüz eski (θ sağa +) firmware'e bağlıyken: başlıkta cihaz_theta_pozitif: sag. Okurken yalnız θ, θ̇
    sütunları çevrilir; PC'nin yazdığı K'lar (yeni kuralda) olduğu gibi kalır."""
    from gui.state import AppState
    from pendulum_lab.recorder import _settings_at_row, compare_recording_with_simulation, load_recording
    from test_kalman_and_playback import ROOT, _record_via_reader
    st = AppState(ROOT / "config" / "params.json")
    new_path, dev_path = tmp_path / "yeni.csv", tmp_path / "eski_cihaz.csv"
    _record_via_reader(new_path, st, run_s=3.0)
    out = []
    for line in new_path.read_text(encoding="utf-8").splitlines():
        if line.startswith("# cihaz_theta_pozitif:"):
            line = "# cihaz_theta_pozitif: sag"
        elif not line.startswith("#") and line and line[0] in "-0123456789":
            c = line.split(",")
            c[3], c[4] = (f"{-float(v)!r}" for v in (c[3], c[4]))
            line = ",".join(c)
        out.append(line)
    dev_path.write_text("\n".join(out) + "\n", encoding="utf-8")
    new, dev = load_recording(new_path), load_recording(dev_path)
    assert dev.meta["cihaz_theta_donusturuldu"] and "theta_isareti_donusturuldu" not in dev.meta
    np.testing.assert_allclose(dev.th, new.th, atol=1e-5)
    np.testing.assert_allclose(dev.dth, new.dth, atol=1e-4)
    np.testing.assert_allclose(dev.meta["K"], st.K, atol=1e-9)            # PC'nin K'sına dokunulmadı
    c_new = compare_recording_with_simulation(new, fallback_state=st)
    c_dev = compare_recording_with_simulation(dev, fallback_state=st)
    np.testing.assert_allclose(c_dev.K, c_new.K, atol=1e-9)
    assert c_dev.rmse_th_deg == pytest.approx(c_new.rmse_th_deg, abs=1e-3)
    meta, _ = _settings_at_row(dev, len(dev.t) - 1)
    assert meta["theta_pozitif"] == "sol"                                 # anlık görüntüler θ alanını ezmez


def test_recording_waits_for_device_info(tmp_path, monkeypatch):
    """Cihaz bilgisi (θ kuralı) gelmeden kayıt başlatılmaz; ping yeniden istenir."""
    import os
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PyQt5 import QtWidgets
    from gui import live_tab
    from gui.state import AppState
    from test_kalman_and_playback import ROOT
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(["test"])
    st = AppState(ROOT / "config" / "params.json")
    st.base_dir = tmp_path
    sent = []
    from types import SimpleNamespace
    st.set_reader(SimpleNamespace(transport=SimpleNamespace(name="sim"), sinks=[], applied_settings={},
                                  send=lambda cmd, **f: sent.append(cmd)))
    tab = live_tab.LiveTab(st)
    tab.b_rec.setChecked(True)
    tab.toggle_record(True)
    assert st.recorder is None and not tab.b_rec.isChecked() and "ping" in sent
    tab.close()
    assert app is not None
