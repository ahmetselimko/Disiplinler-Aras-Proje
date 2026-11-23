"""Telemetri oturumunu CSV'ye kaydeder (Excel / Matlab / pandas ile açılabilir).

TelemetryReader.sinks listesine eklenir; satırlar okuyucu thread'inden gelir, dosyaya tamponlu yazılır.
"""

from __future__ import annotations

import copy
import csv
import json
import math
from dataclasses import asdict, dataclass
from dataclasses import fields as dc_fields
from dataclasses import replace as dc_replace
from collections import deque
import threading
import time
from pathlib import Path

import numpy as np

from .analysis.summary import TrialSummary, compute_trial_summary
from .comm.reader import COLUMNS, match_ack, mode_name
from .control.lqr import design_lqr
from .control.supervisor import Mode, Supervisor
from .control.swingup import EnergySwingUp
from .model.params import PendulumParams
from .sim.simulator import SensorModel, SimResult, simulate_closed_loop


class Recorder:
    def __init__(self, path: str | Path, meta: dict | None = None):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._f = open(self.path, "x", newline="", encoding="utf-8")   # "x": varsa üzerine yazma
        for k, v in (meta or {}).items():
            encoded = v if isinstance(v, str) and '\n' not in v and '\r' not in v else json.dumps(v, ensure_ascii=False, allow_nan=False)
            self._f.write(f"# {k}: {encoded}\n")
        self._w = csv.writer(self._f)
        self._w.writerow(COLUMNS)
        self._lock = threading.Lock()
        self.rows = 0
        self._last_flush = time.perf_counter()

    def write_rows(self, rows):
        with self._lock:
            if self._f.closed:
                return
            self._w.writerows(r[:-1] + (mode_name(r[-1]),) for r in rows)
            self.rows += len(rows)
            if time.perf_counter() - self._last_flush > 1.0:
                self._f.flush()
                self._last_flush = time.perf_counter()

    def close(self):
        with self._lock:
            if not self._f.closed:
                self._f.close()

    def write_event(self, kind: str, data: dict):
        """JSON comment, ignored by CSV readers using comment='#'."""
        payload = json.dumps({"kind": kind, **data}, ensure_ascii=False, allow_nan=False)
        with self._lock:
            if not self._f.closed:
                self._f.write(f"# event: {payload}\n")

    def record_message(self, msg):
        self.write_event(type(msg).__name__, asdict(msg))

    @staticmethod
    def default_path(folder: str | Path = "kayitlar", prefix: str = "oturum") -> Path:
        """Zaman damgalı, var olan bir dosyanın üzerine asla yazmayan yol. prefix: ör. "sim" / "gercek"."""
        base = Path(folder) / time.strftime(f"{prefix}_%Y%m%d_%H%M%S")
        path, i = base.with_suffix(".csv"), 1
        while path.exists():
            path, i = Path(f"{base}_{i}.csv"), i + 1
        return path


ACTIVE_TRIAL_MODES = ("WAIT", "SWING_UP", "BALANCE")
SWINGUP_FIELDS = tuple(f.name for f in dc_fields(EnergySwingUp))
CATCH_LIMIT_KEYS = ("catch_dth", "catch_x", "catch_dx")


@dataclass
class RecordedSession:
    """CSV telemetri kaydının başlık bilgileri, olayları ve sayısal dizileri."""

    path: Path
    meta: dict
    events: list[dict]
    t: np.ndarray
    x: np.ndarray
    dx: np.ndarray
    th: np.ndarray
    dth: np.ndarray
    u: np.ndarray
    mode: list[str]

    def active_slice(self) -> slice:
        """İlk WAIT/SWING_UP/BALANCE örneğinden deneme bitişine kadar olan dilimi döndürür."""
        i0 = next((i for i, m in enumerate(self.mode) if m in ACTIVE_TRIAL_MODES), None)
        if i0 is None:
            return slice(0, len(self.t))
        for i in range(i0 + 1, len(self.mode)):
            if self.mode[i] not in ACTIVE_TRIAL_MODES:
                return slice(i0, i)        # deneme bitişindeki ilk IDLE/FAULT örneği karşılaştırmaya girmesin
        return slice(i0, len(self.mode))


@dataclass
class ReplayComparison:
    """CSV kaydı ile aynı başlangıç koşullu simülasyonun karşılaştırma sonucu."""

    session: RecordedSession
    t: np.ndarray
    x: np.ndarray
    dx: np.ndarray
    th: np.ndarray
    dth: np.ndarray
    u: np.ndarray
    mode: list[str]
    sim: SimResult
    params: PendulumParams
    K: np.ndarray
    rec_summary: TrialSummary
    sim_summary: TrialSummary
    rmse_th_deg: float
    rmse_x: float
    rmse_u: float
    max_err_th_deg: float
    max_err_x: float
    t_start: float = 0.0                 # simülasyonun başladığı an (BAŞLAT), gösterilen eksende [s]
    changed_during_trial: list = None    # deneme sırasında cihazın onayladığı ayar anahtarları
    sim_actions_during_trial: list = None  # deneme sırasında gönderilen dürt/duraklat/sıfırla (simülasyon içermez)
    x_ref: float = 0.0                   # simülasyonun kullandığı hedef konum [m]


def load_recording(path: str | Path) -> RecordedSession:
    """Recorder tarafından üretilmiş CSV dosyasını okur ve ayrıştırır."""
    p = Path(path)
    meta: dict = {}
    events: list[dict] = []
    header: list[str] | None = None
    rows: list[tuple[float, float, float, float, float, float, str]] = []

    with open(p, encoding="utf-8", newline="") as f:
        for raw_line in f:
            line = raw_line.strip()
            if not line:
                continue
            if line.startswith("# event: "):
                try:
                    ev = json.loads(line[len("# event: "):])
                except json.JSONDecodeError:
                    continue
                if isinstance(ev, dict):
                    ev["_row"] = len(rows)    # olay, bu indeksteki telemetri satırından önce geldi
                    events.append(ev)
                continue
            if line.startswith("#"):
                body = line[1:].strip()
                if ": " in body:
                    k, v = body.split(": ", 1)
                    try:
                        meta[k.strip()] = json.loads(v)
                    except json.JSONDecodeError:
                        meta[k.strip()] = v
                continue
            parts = [c.strip() for c in next(csv.reader([line]))]
            if header is None:
                if parts != list(COLUMNS):
                    raise ValueError(f"Beklenen CSV sütunları {list(COLUMNS)}, bulunan: {parts}")
                header = parts
                continue
            if len(parts) < 7:
                continue
            try:
                vals = [float(v) for v in parts[:6]]
            except ValueError:
                continue
            if not all(math.isfinite(v) for v in vals):
                continue
            rows.append((vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], parts[6]))

    if len(rows) < 2:
        raise ValueError(f"Kayıtta karşılaştırma için en az 2 geçerli telemetri satırı olmalı ({p.name}).")

    arr = np.array([r[:6] for r in rows], dtype=float)
    modes = [r[6] for r in rows]
    return RecordedSession(
        path=p,
        meta=meta,
        events=events,
        t=arr[:, 0],
        x=arr[:, 1],
        dx=arr[:, 2],
        th=arr[:, 3],
        dth=arr[:, 4],
        u=arr[:, 5],
        mode=modes,
    )


def _extract_replay_config(meta: dict, fallback_state=None):
    """Kayıt başlığından (veya eksikse AppState'ten) fiziksel model ve kontrol ayarlarını çıkarır."""
    fb_p = getattr(fallback_state, "p", None)
    fb_K = getattr(fallback_state, "K", None)
    fb_Ts = getattr(fallback_state, "Ts", 0.002)
    fb_su = getattr(fallback_state, "swingup", None)
    fb_catch = getattr(fallback_state, "catch_deg", 15.0)
    fb_release = getattr(fallback_state, "release_deg", 30.0)
    fb_su_en = getattr(fallback_state, "swingup_enabled", True)
    fb_xref = getattr(fallback_state, "x_ref", 0.0)

    if isinstance(meta.get("params"), dict):
        p = PendulumParams.from_dict(meta["params"])
    else:
        p = copy.deepcopy(fb_p) if fb_p is not None else PendulumParams()

    Ts_raw = meta.get("device_Ts") or meta.get("Ts") or fb_Ts
    Ts = float(Ts_raw) if Ts_raw and float(Ts_raw) > 0 else 0.002

    su_raw = meta.get("swingup")
    if isinstance(su_raw, dict):
        # yalnız EnergySwingUp'ın bugün tanıdığı alanlar: eski kayıtlardaki kaldırılmış alanlar
        # (ör. E_target) hata vermesin, yeni alanlar (ör. E_ratio) da kendiliğinden okunsun
        su_kw = {k: float(v) for k, v in su_raw.items() if k in SWINGUP_FIELDS}
        swingup = EnergySwingUp(**su_kw)
    else:
        swingup = copy.deepcopy(fb_su) if fb_su is not None else EnergySwingUp()

    catch_deg = float(meta.get("catch_deg", fb_catch))
    release_deg = float(meta.get("release_deg", fb_release))
    swingup_enabled = bool(meta.get("swingup_enabled", fb_su_en))
    x_ref = float(meta.get("x_ref", fb_xref))

    K_raw = meta.get("K")
    K = np.asarray(K_raw, dtype=float).ravel() if K_raw is not None else (
        np.asarray(fb_K, dtype=float).ravel() if fb_K is not None
        else design_lqr(p, [10.0, 1.0, 100.0, 1.0], [[0.1]], Ts=Ts)[0].ravel()
    )
    if K.size != 4 or not np.all(np.isfinite(K)):
        K = design_lqr(p, [10.0, 1.0, 100.0, 1.0], [[0.1]], Ts=Ts)[0].ravel()

    sup_p = copy.deepcopy(p)
    ack = meta.get("device_acknowledged_settings")
    if isinstance(ack, dict):
        if "K" in ack and len(ack["K"]) == 4:
            K = np.asarray(ack["K"], dtype=float).ravel()
        if "swingup" in ack:
            swingup_enabled = bool(ack["swingup"])
        if "catch_deg" in ack:
            catch_deg = float(ack["catch_deg"])
        if "release_deg" in ack:
            release_deg = float(ack["release_deg"])
        for k in SWINGUP_FIELDS:
            if k in ack:
                setattr(swingup, k, float(ack[k]))
        if "u_max" in ack:
            p.u_max = min(float(ack["u_max"]), p.motor.V_max) if p.input_mode == "voltage" else float(ack["u_max"])
            sup_p.u_max = p.u_max
        if "x_ref" in ack:
            x_ref = float(ack["x_ref"])
        if "fc_volt" in ack and sup_p.input_mode == "voltage" and sup_p.motor.alpha > 0:
            sup_p.motor.T_f = max(0.0, float(ack["fc_volt"])) * sup_p.motor.alpha * sup_p.motor.r_pulley / sup_p.motor.gear

    return p, sup_p, K, Ts, swingup, catch_deg, release_deg, swingup_enabled, x_ref


def _catch_limits(meta: dict, fallback_state=None) -> dict:
    """LQR'a geçiş sınırları (catch_dth, catch_x, catch_dx): cihazın onayladığı > kayıt başlığı > AppState."""
    ack = meta.get("device_acknowledged_settings") or {}
    out = {}
    for k in CATCH_LIMIT_KEYS:
        v = ack.get(k, meta.get(k, getattr(fallback_state, k, None)))
        if v is not None:
            out[k] = float(v)
    return out


def _settings_at_row(session: RecordedSession, row: int, end_row: int | None = None) -> tuple[dict, list[str]]:
    """Kayıt olaylarını sırayla işleyerek `row` indeksli telemetri satırından önce geçerli ayarları çıkarır.

    Başlıktaki ayarlar kaydın açıldığı anı gösterir; kayıt sırasında gönderilip cihazca onaylanan
    set_param/set_gain komutları (command_sent + Ack olayları) ve PC tasarım anlık görüntüleri
    (pc_design_snapshot) üzerine uygulanır. Cihaz yeniden başlarsa (boot) onaylı ayarlar sıfırlanır.
    Dönüş: (etkin meta, deneme sırasında [row, end_row) onaylanan ayar anahtarları).
    """
    meta = dict(session.meta)
    ack = dict(meta.get("device_acknowledged_settings") or {})
    pending: deque = deque()
    later: set[str] = set()
    for ev in session.events:
        at = int(ev.get("_row", 0))
        before = at <= row
        kind = ev.get("kind")
        if kind == "command_sent":
            pending.append((ev.get("cmd"), dict(ev.get("fields") or {})))
        elif kind == "Ack":
            req = match_ack(pending, ev.get("cmd"))     # okuyucuyla aynı sıra kuralı ("?" dahil)
            if req is None:
                continue
            if not ev.get("ok", True) or req[0] not in ("set_param", "set_gain"):
                continue
            if before:
                ack.update(req[1])
            elif end_row is None or at < end_row:
                later.update(req[1].keys())
        elif kind == "Event" and ev.get("name") == "boot":
            pending.clear()
            if before:
                ack = {}
        elif kind == "pc_design_snapshot" and before:
            snap = ev.get("settings")
            if isinstance(snap, dict):
                meta.update({k: v for k, v in snap.items() if k != "device_acknowledged_settings"})
    meta["device_acknowledged_settings"] = ack
    return meta, sorted(later)


SIM_ACTION_NAMES = {"kick": "dürtme", "pause": "duraklatma", "reset": "sıfırlama"}


def _sim_actions_between(session: RecordedSession, row: int, end_row: int | None) -> list[str]:
    """[row, end_row) aralığında gönderilen yalnız-simülasyon komutları (kick/pause/reset). Karşılaştırma
    simülasyonu bunları modellemez: dürtme yapılmaz, duraklatma kayıt zamanını kaydırır."""
    found = []
    for ev in session.events:
        at = int(ev.get("_row", 0))
        if ev.get("kind") == "command_sent" and ev.get("cmd") in SIM_ACTION_NAMES and at >= row \
                and (end_row is None or at < end_row):
            name = SIM_ACTION_NAMES[ev["cmd"]]
            if name not in found:
                found.append(name)
    return found


def compare_recording_with_simulation(
    session: RecordedSession,
    *,
    active_only: bool = True,
    use_recorded_config: bool = True,
    fallback_state=None,
    realistic_sensor: bool = True,
    use_kalman: bool = True,
) -> ReplayComparison:
    """Kayıttaki denemeyi aynı başlangıç koşullu kapalı çevrim simülasyonuyla karşılaştırır.

    Simülasyon her zaman denemenin BAŞLAT anındaki (ilk WAIT/SWING_UP/BALANCE satırı) durum ve o ana kadar
    cihazın onayladığı ayarlarla başlar. active_only=False iken kaydın tamamı gösterilir; simülasyon eğrisi
    BAŞLAT anına kaydırılır, hata metrikleri yine yalnız aktif deneme aralığında hesaplanır.
    """
    n_rows = len(session.t)
    act = session.active_slice()
    has_active = any(m in ACTIVE_TRIAL_MODES for m in session.mode)
    if act.stop - act.start < 2:
        act, has_active = slice(0, n_rows), False
    shown = act if active_only else slice(0, n_rows)
    i_start = act.start if has_active else shown.start

    t0 = float(session.t[shown.start])
    t_seg = session.t[shown] - t0
    x_seg = session.x[shown]
    dx_seg = session.dx[shown]
    th_seg = session.th[shown]
    dth_seg = session.dth[shown]
    u_seg = session.u[shown]
    m_seg = session.mode[shown]

    if use_recorded_config:
        meta, changed_later = _settings_at_row(session, i_start, act.stop)
    else:
        meta, changed_later = {}, []
    sim_actions = _sim_actions_between(session, i_start, act.stop)
    p, sup_p, K, Ts, swingup, catch_deg, release_deg, swingup_enabled, x_ref = _extract_replay_config(
        meta, fallback_state=fallback_state
    )

    sup = Supervisor(
        sup_p,
        K,
        swingup=swingup,
        catch_deg=catch_deg,
        release_deg=release_deg,
        swingup_enabled=swingup_enabled,
    )
    for k, v in _catch_limits(meta, fallback_state).items():
        if hasattr(sup, k):                 # Supervisor'ın desteklediği geçiş sınırları
            setattr(sup, k, v)
    lim = max(0.0, p.rail_half - 0.1)
    sup.x_ref = max(-lim, min(lim, x_ref))
    m_start = session.mode[i_start]
    if m_start == "BALANCE":
        sup.mode = Mode.BALANCE
    elif m_start == "WAIT":
        sup.mode = Mode.WAIT
    else:
        sup.start()

    s0 = [float(session.x[i_start]), float(session.dx[i_start]),
          float(session.th[i_start]), float(session.dth[i_start])]
    t_off = float(session.t[i_start]) - t0
    T = max(float(session.t[shown.stop - 1]) - float(session.t[i_start]), Ts)
    sensor = SensorModel.for_params(p, seed=0) if realistic_sensor else None
    sim = simulate_closed_loop(p, sup, s0, T, dt=Ts, sensor=sensor, use_kalman=use_kalman)
    sim_sum_src = sim
    sim = dc_replace(sim, t=sim.t + t_off)      # kaydın zaman eksenine (BAŞLAT anı) hizala

    # hata metrikleri: yalnız aktif deneme aralığı (tam kayıt modunda da)
    t_m = session.t[act] - t0
    th_m, x_m, u_m = session.th[act], session.x[act], session.u[act]
    th_sim_i = np.interp(t_m, sim.t, sim.th)
    x_sim_i = np.interp(t_m, sim.t, sim.x)
    u_sim_i = np.interp(t_m, sim.t, sim.u)
    err_th_deg = np.degrees(np.arctan2(np.sin(th_m - th_sim_i), np.cos(th_m - th_sim_i)))
    err_x = x_m - x_sim_i
    err_u = u_m - u_sim_i

    m_act = session.mode[act]
    inc = np.concatenate(([True], np.diff(t_m) > 0))
    rec_summary = compute_trial_summary(
        t_m[inc] - t_m[0], x_m[inc], th_m[inc], u_m[inc],
        modes=[m for m, ok in zip(m_act, inc) if ok],
        u_max=p.u_max,
        rail_half=p.rail_half,
    )
    sim_summary = compute_trial_summary(
        sim_sum_src.t, sim_sum_src.x, sim_sum_src.th, sim_sum_src.u,
        modes=sim_sum_src.mode, u_max=p.u_max, rail_half=p.rail_half,
    )

    return ReplayComparison(
        session=session,
        t=t_seg,
        x=x_seg,
        dx=dx_seg,
        th=th_seg,
        dth=dth_seg,
        u=u_seg,
        mode=m_seg,
        sim=sim,
        params=p,
        K=K,
        # aktif dilim bitişteki ilk IDLE/FAULT örneğini dışarıda bırakır; denemenin NASIL bittiği özette görünsün
        rec_summary=(dc_replace(rec_summary, final_mode=str(session.mode[act.stop]))
                     if act.stop < len(session.mode) and session.mode[act.stop] in ("FAULT", "IDLE")
                     else rec_summary),
        sim_summary=sim_summary,
        t_start=t_off,
        changed_during_trial=changed_later,
        sim_actions_during_trial=sim_actions,
        x_ref=float(sup.x_ref),
        rmse_th_deg=float(np.sqrt(np.mean(err_th_deg ** 2))),
        rmse_x=float(np.sqrt(np.mean(err_x ** 2))),
        rmse_u=float(np.sqrt(np.mean(err_u ** 2))),
        max_err_th_deg=float(np.max(np.abs(err_th_deg))),
        max_err_x=float(np.max(np.abs(err_x))),
    )
