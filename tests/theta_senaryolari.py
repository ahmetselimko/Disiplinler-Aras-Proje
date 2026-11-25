"""θ işaret kuralı değişikliğinin (#50 Aşama 1) referans senaryoları.

θ'nın pozitif yönü "sağa (+x) yatık"tan "sola yatık"a çevrildi (mekatronik raporu 8.1.3). Kural: fiziksel
davranış hiç değişmemeli. Aynı fiziksel durumda x, ẋ ve u aynı kalır; yalnız θ ve θ̇'nın işareti döner.

Bu modül senaryoları ESKİ kodla (commit 81d66f6, θ sağa +) bir kez çalıştırıp sonuçları
tests/data/theta_isaret_referans.json'a yazdı:

    python tests/theta_senaryolari.py --yaz      # yalnız eski kodda anlamlıdır; yeniden çalıştırmayın

tests/test_theta_isaret.py aynı senaryoları YENİ kodla, başlangıç açısı ve açısal hızı ters işaretle koşar ve
x, ẋ, u'nun aynı, θ, θ̇'nın ters olduğunu denetler.

Parametreler, sensör çözünürlüğü, swing-up ayarları ve Kalman ölçüm gürültüsü burada SABİT tutulur:
sonraki aşamalarda (ör. Aşama 2: params.json, ENC_CPR_MOTOR, E_ratio) varsayılanlar değişse de referans
geçerli kalsın.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pendulum_lab.comm import protocol as P  # noqa: E402
from pendulum_lab.comm.mock_device import MockDevice  # noqa: E402
from pendulum_lab.control.lqr import design_lqr  # noqa: E402
from pendulum_lab.control.supervisor import Mode, Supervisor  # noqa: E402
from pendulum_lab.control.swingup import EnergySwingUp  # noqa: E402
from pendulum_lab.filters import KalmanFilter  # noqa: E402
from pendulum_lab.model.params import PendulumParams  # noqa: E402
from pendulum_lab.sim.matlab_compat import force_model, simulate_matlab_compatible  # noqa: E402
from pendulum_lab.sim.simulator import SensorModel, simulate_closed_loop  # noqa: E402

REF_PATH = Path(__file__).resolve().parent / "data" / "theta_isaret_referans.json"

TS = 0.002
Q = (10.0, 1.0, 100.0, 1.0)
R = 0.1
# Referansın üretildiği anki config/params.json (25.11.2026); yalnız T_f = 0.0015 (dosyada 0) bilerek eklendi:
# swing-up'taki Coulomb telafisi ve sürtünmeli denge de denensin
PARAMS = {
    "M": 0.5, "m": 0.2, "l": 0.3, "I": 0.0, "b": 0.1, "c": 0.0005, "g": 9.81, "rail_half": 0.4,
    "u_max": 12.0, "input_mode": "voltage",
    "motor": {"R": 2.18, "Kt": 0.00892, "Ke": 0.01104, "gear": 10.0, "r_pulley": 0.009549,
              "V_max": 12.0, "J_rotor": 0.0, "T_f": 0.0015, "B_m": 0.0},
}
SWINGUP = dict(k_E=100.0, a_max=4.0, k_x=10.0, k_v=6.0, a_kick=2.0, E_ratio=1.0)
TH_RES = 2 * math.pi / 4096
X_RES = 2 * math.pi * 0.009549 / (48.0 * 10.0)      # 48 CPR × 10:1, HTD3M 20 diş
KALMAN_RN = ((X_RES ** 2) / 12.0, (TH_RES ** 2) / 12.0)
SUBSAMPLE = 10


def params() -> PendulumParams:
    return PendulumParams.from_dict(json.loads(json.dumps(PARAMS)))


def gain(p: PendulumParams) -> np.ndarray:
    K, _, _ = design_lqr(p, Q, [[R]], Ts=TS)
    return np.asarray(K, dtype=float).reshape(-1)


def _sup(p, K):
    sup = Supervisor(p, K, swingup=EnergySwingUp(**SWINGUP))
    sup.start()
    return sup


def _pulse(F, t0=1.0, dur=0.05):
    return lambda t: F if t0 <= t < t0 + dur else 0.0


# (ad, θ₀ [rad], θ̇₀ [rad/s], T [s], dürtme kuvveti [N], dürtme yeri). Açılar ESKİ işarette (sağa +).
CLOSED_LOOP = [
    ("asagidan_kalkis", math.pi - 0.05, 0.0, 10.0, 0.0, "pole"),
    ("dik_arti5", math.radians(5.0), 0.0, 3.0, 0.0, "pole"),
    ("dik_eksi5", math.radians(-5.0), 0.0, 3.0, 0.0, "pole"),
    ("durtme_sarkac", math.radians(1.0), 0.3, 3.0, 0.6, "pole"),
    ("durtme_araba", math.radians(-1.0), 0.0, 3.0, 3.0, "cart"),
]


def run_closed_loop(name: str, sign: float) -> dict:
    """sign = +1: tablodaki başlangıç durumu; sign = −1: θ₀ ve θ̇₀ ters (yeni işaret kuralında aynı fizik)."""
    spec = {c[0]: c for c in CLOSED_LOOP}[name]
    _, th0, dth0, T, F, at = spec
    p = params()
    K = gain(p)
    sensor = SensorModel(th_resolution=TH_RES, x_resolution=X_RES, seed=0)
    r = simulate_closed_loop(p, _sup(p, K), [0.0, 0.0, sign * th0, sign * dth0], T, dt=TS, sensor=sensor,
                             disturbance=_pulse(F) if F else None, disturbance_target=at,
                             kalman_Rn=KALMAN_RN)
    sl = slice(None, None, SUBSAMPLE)
    return {"t": r.t[sl].tolist(), "s": r.s[sl].tolist(), "u": r.u[sl].tolist(), "mode": r.mode[::SUBSAMPLE],
            "K": K.tolist()}


def run_matlab(sign: float) -> dict:
    """Matlab uyumlu deneme (cart_pendulum_application.m): kuvvet girişi, arabaya dürtme."""
    p = force_model(params())
    p.motor.T_f = 0.0
    K = gain(p)
    r = simulate_matlab_compatible(p, K, [0.02, 0.0, sign * math.radians(4.0), sign * 0.2], T=3.0,
                                   disturbance=_pulse(4.0))
    sl = slice(None, None, SUBSAMPLE)
    return {"t": r.t[sl].tolist(), "s": r.s[sl].tolist(), "u": r.u[sl].tolist(), "stop": r.stop_reason,
            "K": K.tolist()}


def run_mock(sign: float) -> dict:
    """Sanal ESP32: KALİBRE ET (homing) + BAŞLAT; telemetri satırları (sanal zaman, 6 s)."""
    p = params()
    out: list[bytes] = []
    dev = MockDevice(params=p, seed=3, realtime=False, x0=0.12, telemetry_hz=100)
    dev.s = [0.12, 0.0, sign * (math.pi - 0.02), 0.0]
    dev.x_offset = dev.s[0]
    dev.sensor = SensorModel(th_resolution=TH_RES, x_resolution=X_RES, seed=3)
    dev.kf = KalmanFilter.from_params(dev.sup.p, TS, Rn=KALMAN_RN)
    dev.sup.swingup = EnergySwingUp(**SWINGUP)
    dev.sup.set_gain(gain(p))
    dev.output = out.append
    dev.start()
    dev.feed(P.encode_command("home"))
    for _ in range(int(25 / TS)):
        dev.step()
        if dev.homed or dev.sup.mode == Mode.FAULT:
            break
    homed = dev.homed
    out.clear()
    dev.feed(P.encode_command("start"))
    for _ in range(int(6.0 / TS)):
        dev.step()
    tel = [m for m in (P.parse_line(o) for o in out) if isinstance(m, P.Telemetry)]
    return {"homed": homed, "t": [m.t for m in tel], "x": [m.x for m in tel], "dx": [m.dx for m in tel],
            "th": [m.th for m in tel], "dth": [m.dth for m in tel], "u": [m.u for m in tel],
            "mode": [m.mode for m in tel], "true_final": list(dev.true_state)}


def run_all(sign: float) -> dict:
    data = {name: run_closed_loop(name, sign) for name, *_ in CLOSED_LOOP}
    data["matlab_uyumlu"] = run_matlab(sign)
    data["sanal_esp32"] = run_mock(sign)
    return data


if __name__ == "__main__":
    if "--yaz" not in sys.argv:
        sys.exit("Referans yalnız eski kodla (θ sağa +) bir kez üretildi. Yazmak için: --yaz")
    REF_PATH.parent.mkdir(exist_ok=True)
    data = run_all(+1.0)
    data["_aciklama"] = ("#50 Aşama 1 öncesi kodla (commit 81d66f6, θ sağa +) üretildi. "
                         "tests/theta_senaryolari.py --yaz; elle düzenlemeyin.")
    REF_PATH.write_text(json.dumps(data, separators=(",", ":")), encoding="utf-8")
    for k, v in data.items():
        if isinstance(v, dict):
            m = v.get("mode") or []
            print(k, "son mod:", m[-1] if m else v.get("stop"), "homed:", v.get("homed", "-"))
