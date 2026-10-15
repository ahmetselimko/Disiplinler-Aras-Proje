"""ESP32 ↔ PC haberleşme protokolü (satır tabanlı JSON). Tam tanım: docs/protokol.md

ESP32 → PC
  telemetri : {"n":123,"t":1.2500,"x":0.1200,"dx":0.0,"th":0.0500,"dth":0.0,"u":1.30,"mode":"BALANCE"}
              (PDF'teki ilk taslak {"t","theta","x"} de kabul edilir; eksik alanlar 0 sayılır)
  olay      : {"evt":"limit","side":"L"}  {"evt":"homed"}  {"evt":"fault","reason":"rail"}  {"evt":"boot"}
  yanıt     : {"ack":"set_gain","ok":true}  /  {"ack":"start","ok":false,"msg":"önce home"}
PC → ESP32
  {"cmd":"set_gain","K":[k1,k2,k3,k4]}   {"cmd":"start"}  {"cmd":"stop"}  {"cmd":"estop"}
  {"cmd":"home"}  {"cmd":"ping"}  {"cmd":"calib"}  {"cmd":"set_param","catch_deg":15,...}

SET_PARAM_LIMITS bu protokolün TEK kaynağıdır; firmware'deki PARAM_LIMITS tablosu ve docs/protokol.md
bununla aynı olmalıdır (tests/test_comm.py::test_firmware_param_limits_match_python bunu denetler).
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field

TELEMETRY_KEYS = ("n", "t", "x", "dx", "th", "dth", "u", "mode")
COMMANDS = ("ping", "start", "stop", "estop", "home", "calib", "set_gain", "set_param", "kick")
# set_param ile değiştirilebilecek parametreler ve kabul edilen aralıklar
SET_PARAM_LIMITS = {
    "catch_deg": (1.0, 45.0),
    "release_deg": (2.0, 90.0),
    "swingup": (0, 1),
    "k_E": (0.0, 1e4),
    "a_max": (0.0, 50.0),
    "k_x": (0.0, 1e3),
    "k_v": (0.0, 1e3),
    "x_ref": (-0.3, 0.3),        # ray yarı boyu − 0.1 m (yer tutucu ray ±0.4 m)
    "u_max": (0.0, 48.0),        # cihaz ayrıca kendi besleme gerilimiyle sınırlar
    "telemetry_hz": (1, 500),
}


def validate_params(fields: dict) -> None:
    """set_param alanlarını doğrular (hepsi geçerli değilse hiçbiri uygulanmamalı)."""
    for key, val in fields.items():
        if key not in SET_PARAM_LIMITS:
            raise ProtocolError(f"bilinmeyen parametre: {key}")
        try:
            v = float(val)
        except (TypeError, ValueError):
            raise ProtocolError(f"{key} sayı olmalı") from None
        lo, hi = SET_PARAM_LIMITS[key]
        if not (lo <= v <= hi):
            raise ProtocolError(f"{key}={val} aralık dışı [{lo}, {hi}]")
    if "catch_deg" in fields and "release_deg" in fields and float(fields["catch_deg"]) >= float(fields["release_deg"]):
        raise ProtocolError("catch_deg < release_deg olmalı")


class ProtocolError(ValueError):
    pass


@dataclass(slots=True)
class Telemetry:
    n: int
    t: float
    x: float
    dx: float
    th: float
    dth: float
    u: float
    mode: str


@dataclass(slots=True)
class Event:
    name: str
    data: dict = field(default_factory=dict)


@dataclass(slots=True)
class Ack:
    cmd: str
    ok: bool = True
    msg: str = ""
    data: dict = field(default_factory=dict)   # ek alanlar, ör. ping yanıtında {"Ts": 0.002}


def parse_line(line: bytes | str) -> Telemetry | Event | Ack:
    """ESP32'den gelen bir satırı çözer. Bozuk satırda ProtocolError fırlatır."""
    try:
        d = json.loads(line)
    except (ValueError, UnicodeDecodeError) as e:
        raise ProtocolError(f"JSON değil: {line!r}") from e
    if not isinstance(d, dict):
        raise ProtocolError(f"nesne bekleniyordu: {line!r}")
    if "t" in d:
        try:
            return Telemetry(
                int(d.get("n", -1)), float(d["t"]), float(d.get("x", 0.0)), float(d.get("dx", 0.0)),
                float(d["th"] if "th" in d else d.get("theta", 0.0)), float(d.get("dth", 0.0)),
                float(d.get("u", 0.0)), str(d.get("mode", "")))
        except (TypeError, ValueError) as e:
            raise ProtocolError(f"telemetri alanı sayı değil: {line!r}") from e
    if "evt" in d:
        return Event(str(d.pop("evt")), d)
    if "ack" in d:
        cmd, ok, msg = str(d.pop("ack")), bool(d.pop("ok", True)), str(d.pop("msg", ""))
        return Ack(cmd, ok, msg, d)
    raise ProtocolError(f"tanınmayan mesaj: {line!r}")


def _dump(obj: dict) -> bytes:
    return (json.dumps(obj, separators=(",", ":"), ensure_ascii=False) + "\n").encode("utf-8")


def encode_command(cmd: str, **fields) -> bytes:
    """PC → ESP32 komutu. Geçersiz komut/parametreyi göndermeden önce yakalar."""
    if cmd not in COMMANDS:
        raise ProtocolError(f"bilinmeyen komut: {cmd}")
    if cmd == "set_gain":
        K = [float(k) for k in fields.get("K", [])]
        if len(K) != 4 or not all(math.isfinite(k) for k in K):
            raise ProtocolError("K 4 elemanlı ve sonlu olmalı")
        fields = {"K": K}
    elif cmd == "set_param":
        validate_params(fields)
    return _dump({"cmd": cmd, **fields})


def encode_telemetry(n: int, t: float, x: float, dx: float, th: float, dth: float, u: float, mode: str) -> bytes:
    """Cihaz tarafı (sanal ESP32) için hızlı biçimlendirme; firmware'deki snprintf ile aynı biçim."""
    return (f'{{"n":{n},"t":{t:.4f},"x":{x:.5f},"dx":{dx:.4f},"th":{th:.5f},'
            f'"dth":{dth:.4f},"u":{u:.3f},"mode":"{mode}"}}\n').encode()


def encode_event(name: str, **data) -> bytes:
    return _dump({"evt": name, **data})


def encode_ack(cmd: str, ok: bool = True, msg: str = "", **data) -> bytes:
    d = {"ack": cmd, "ok": ok}
    if msg:
        d["msg"] = msg
    d.update(data)
    return _dump(d)


def parse_command(line: bytes | str) -> tuple[str, dict]:
    """Cihaz tarafı: PC'den gelen komutu çözer."""
    try:
        d = json.loads(line)
        cmd = d.pop("cmd")
    except (ValueError, KeyError, AttributeError, TypeError) as e:
        raise ProtocolError(f"komut çözülemedi: {line!r}") from e
    if cmd not in COMMANDS:
        raise ProtocolError(f"bilinmeyen komut: {cmd}")
    return cmd, d
