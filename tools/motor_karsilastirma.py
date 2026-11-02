"""Motor karşılaştırması (#7): Pololu 37D, Pololu 35:1 motorlar ve NEMA17 (Pololu #1200).

    python tools/motor_karsilastirma.py [sonuclar.csv]

Her aday için arayüzdeki adımlar toplu yapılır: K hesapla (varsayılan Q/R/Ts) → θ₀ = 180° ile 20 s
simülasyon; iki ray boyunda (±0.4 / ±0.2 m), DC motorlarda sürtünmesiz ve sürtünmeli (telafi açık) olarak.
"Sürtünmeli" denemelerde telafi GERÇEK T_f ile çalışır, yani sürtünmenin doğru bilindiği varsayılır; yazılımda
T_f = 0 kalırsa sarkaç kalkmaz (#30). Gerçek düzenekte sürtünme arayüzdeki Sürtünme Ölç ile ölçülmelidir.
Sarkaç/araba değerleri config/params.json'dan gelir (şu an YER TUTUCU). Ölçüler gelince yeniden çalıştırın.
Sonunda aday × kasnak başına kaç denemenin dengeye geldiğini yazdırır; CSV yolu verilirse tüm satırları kaydeder.

DC motor değerleri issue #7'deki yöntemle datasheet'ten türetilir (çıkış mili değerleri):
    R = V / I_durma,  Kt = T_durma / (N·I_durma),  Ke = (V − R·I_boşta) / (N·ω_boşta),  T_f ≈ Kt·I_boşta
NEMA17 ivme kaynağı kabul edilir (input_mode='acceleration'); gereken tork, hızla düşen kaba bir tork
eğrisiyle karşılaştırılır (stepper_tork). Datasheet'te tork–hız eğrisi yok; sonuç kabaca doğrudur.
"""

from __future__ import annotations

import copy
import csv
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from pendulum_lab import MotorParams, SensorModel, Supervisor, load_params, simulate_closed_loop  # noqa: E402
from pendulum_lab.control.lqr import design_lqr  # noqa: E402
from pendulum_lab.model.motor import max_cart_force, max_cart_speed, stepper_max_accel  # noqa: E402

OZIN = 0.00706155   # 1 oz·in [N·m]
KGCM = 0.0980665    # 1 kg·cm [N·m]


def dc_motor(V, rpm_bos, I_bos, I_durma, T_durma, N):
    """Datasheet → motor mili değerleri. rpm_bos ve T_durma redüktör çıkışında (N = 1: redüktörsüz)."""
    R = V / I_durma
    Kt = T_durma / (N * I_durma)
    Ke = (V - R * I_bos) / (rpm_bos * 2 * math.pi / 60 * N)
    return dict(R=R, Kt=Kt, Ke=Ke, V_max=V, T_f=Kt * I_bos)


# 37D 12V: redüktörsüz 10000 rpm, boşta 0.2 A, durma 5.5 A / 0.5 kg·cm (issue #7). J_rotor bilinmiyor → 0 ve
# en kötü durum (3e-6 kg·m² → 10:1 + 40 dişte arabaya ~1.9 kg ekler) denenir.
M37D = dc_motor(12.0, 10000, 0.2, 5.5, 0.5 * KGCM, 1)
J_37D = (0.0, 3e-6)
DC_ADAYLAR = [  # (ad, motor değerleri, redüktör, motor milinde enkoder CPR, denenecek J_rotor'lar)
    *[(f"37D {N}:1", M37D, N, 64, J_37D) for N in (10, 19, 30)],
    # "Pololu 35" için model numarası verilmedi; Pololu'nun 35:1 redüktörlü iki motoru denendi.
    # DOĞRULANMADI: hangi motorun kastedildiği ve aşağıdaki datasheet değerleri mekatronikle teyit edilmeli.
    ("#1091 35:1 mini 6V", dc_motor(6.0, 350, 0.07, 1.5, 20 * OZIN, 35), 35, 12, (0.0,)),
    ("#1597 35:1 15.5D 6V", dc_motor(6.0, 460, 0.06, 0.8, 13 * OZIN, 35), 35, 12, (0.0,)),
]
# NEMA17, Pololu #1200: 200 adım/tur, 1.2 A/faz, 3.3 Ω, 2.8 mH, tutma torku 3.2 kg·cm
NEMA17 = dict(T_hold=3.2 * KGCM, I=1.2, R=3.3, L=2.8e-3, J=68e-7)   # J sayfada yok: 48 mm NEMA17 için tipik — VARSAYIM
MIKROADIM = 16
TEETH = (20, 40, 60)                    # GT2: diş × 2 mm çevre
RAILS = (0.4, 0.2)
Q, R_LQR, TS, T_SIM = [10.0, 1.0, 100.0, 1.0], 0.1, 0.002, 20.0   # arayüzün varsayılanları


def ozet(res):
    modes = res.mode
    t_denge = None
    if modes[-1] == "BALANCE":   # kalıcı denge: bu andan sonra hep BALANCE
        i = len(modes) - 1
        while i > 0 and modes[i - 1] == "BALANCE":
            i -= 1
        t_denge = float(res.t[i])
    return dict(sonuc="denge" if modes[-1] == "BALANCE" else modes[-1], t_denge=t_denge,
                max_x=float(np.abs(res.x).max()), max_v=float(np.abs(res.s[:, 1]).max()))


def calistir(p, x_cozunurluk):
    K, _, _ = design_lqr(p, Q, [[R_LQR]], Ts=TS)
    sup = Supervisor(p, K)
    sup.start()
    return simulate_closed_loop(p, sup, [0, 0, math.pi, 0], T_SIM, dt=TS,
                                sensor=SensorModel(x_resolution=x_cozunurluk))


def dc_kos(base, ad, mot, N, cpr, J, teeth, rail, surtunme):
    p = copy.deepcopy(base)
    r = teeth * 0.002 / (2 * math.pi)
    m = dict(mot, T_f=mot["T_f"] if surtunme else 0.0)
    p.motor = MotorParams(gear=float(N), r_pulley=r, J_rotor=J, **m)
    p.rail_half, p.u_max = rail, m["V_max"]
    satir = dict(aday=ad, dis=teeth, ray=rail, J_rotor=J, surtunme="var+telafi" if surtunme else "yok",
                 v_teorik=max_cart_speed(p), F_durma=max_cart_force(p), F_surtunme=p.F_coulomb)
    try:
        res = calistir(p, 2 * math.pi * r / (cpr * N))
    except FloatingPointError:
        return {**satir, "sonuc": "ıraksadı"}
    swing = np.array([md == "SWING_UP" for md in res.mode])
    sat = np.abs(res.u) >= 0.99 * p.u_max
    return {**satir, **ozet(res), "doyum_yuzde": float(100 * sat[swing].mean()) if swing.any() else 0.0}


def stepper_kose_hizi(V_s, T_hold, I, R, L):
    """Sürücünün faz akımını hâlâ tam verebildiği en yüksek mil hızı [rad/s] (kaba):
    k_e·ω + I·|R + j·50ω·L| = V_s  (50 kutup çifti, k_e ≈ T_hold/(√2·I))."""
    ke = T_hold / (math.sqrt(2) * I)
    lo, hi = 0.0, 1000.0
    for _ in range(60):
        w = (lo + hi) / 2
        if ke * w + I * math.hypot(R, 50 * w * L) > V_s:
            hi = w
        else:
            lo = w
    return lo


def stepper_tork(w, w_c, T_hold):
    """Kullanılabilir tork: düşük hızda tutma torkunun %70'i, köşe hızından sonra 1/ω ile düşer (kaba)."""
    return 0.7 * T_hold * min(1.0, w_c / max(abs(w), 1e-9))


def stepper_kos(base, teeth, rail):
    p = copy.deepcopy(base)
    r = teeth * 0.002 / (2 * math.pi)
    n = NEMA17
    p.input_mode, p.rail_half = "acceleration", rail
    p.u_max = stepper_max_accel(n["T_hold"], r, p.M + p.m + n["J"] / r ** 2)   # tutma torkunun %50'si
    satir = dict(aday="NEMA17 #1200", dis=teeth, ray=rail, J_rotor=n["J"], surtunme="önemsiz", a_max=p.u_max)
    try:
        res = calistir(p, 2 * math.pi * r / (200 * MIKROADIM))
    except FloatingPointError:
        return {**satir, "sonuc": "ıraksadı"}
    # gereken kuvvet (accel_to_input'un kuvvet formülü) → mil torku; mil hızı
    _, dx, th, dth = res.s.T
    a = res.u
    ml = p.m * p.l
    ddth = (ml * p.g * np.sin(th) - p.c * dth - ml * np.cos(th) * a) / p.J
    F = (p.M + p.m) * a + ml * np.cos(th) * ddth - ml * np.sin(th) * dth ** 2 + p.b * dx
    T_req = np.abs(F * r + n["J"] * a / r)
    w = dx / r
    aktif = np.array([md in ("SWING_UP", "BALANCE", "HOMING") for md in res.mode])
    out = {**satir, **ozet(res), "max_rpm": float(np.abs(w).max() * 60 / (2 * math.pi)),
           "max_tork_yuzde": float(100 * T_req[aktif].max() / n["T_hold"]) if aktif.any() else 0.0}
    for V_s in (12.0, 24.0):   # tork oranı < 1: adım kaçırmadan yapabilir (kaba)
        w_c = stepper_kose_hizi(V_s, n["T_hold"], n["I"], n["R"], n["L"])
        T_av = np.array([stepper_tork(wi, w_c, n["T_hold"]) for wi in w])
        out[f"tork_orani_{int(V_s)}V"] = float((T_req[aktif] / T_av[aktif]).max()) if aktif.any() else 0.0
        out[f"kose_rpm_{int(V_s)}V"] = w_c * 60 / (2 * math.pi)
    return out


def main():
    base = load_params(ROOT / "config" / "params.json")
    rows = []
    for ad, mot, N, cpr, Js in DC_ADAYLAR:
        for teeth in TEETH:
            for rail in RAILS:
                for J in Js:
                    for surt in (False, True):
                        rows.append(dc_kos(base, ad, mot, N, cpr, J, teeth, rail, surt))
                        r_ = rows[-1]
                        print(f"{ad:<20} {teeth}diş ray=±{rail} J={J:.0e} sürtünme={r_['surtunme']:<10} "
                              f"→ {r_['sonuc']}", flush=True)
    for teeth in TEETH:
        for rail in RAILS:
            rows.append(stepper_kos(base, teeth, rail))
            print(f"{'NEMA17 #1200':<20} {teeth}diş ray=±{rail} → {rows[-1]['sonuc']}", flush=True)

    print("\nÖzet (dengeye gelen / deneme):")
    gruplar: dict = {}
    for r_ in rows:
        gruplar.setdefault((r_["aday"], r_["dis"]), []).append(r_)
    for (ad, teeth), g in gruplar.items():
        ilk = g[0]
        ek = (f"v_max≈{ilk['v_teorik']:.2f} m/s  F_durma≈{ilk['F_durma']:.0f} N" if "v_teorik" in ilk else
              f"a_max≈{ilk['a_max']:.0f} m/s²  {max(x.get('max_rpm', 0) for x in g):.0f} rpm  "
              f"tork oranı 12V/24V {max(x.get('tork_orani_12V', 0) for x in g):.2f}/"
              f"{max(x.get('tork_orani_24V', 0) for x in g):.2f}")
        print(f"  {ad:<20} {teeth} diş: {sum(x['sonuc'] == 'denge' for x in g)}/{len(g)}   {ek}")

    if len(sys.argv) > 1:
        alanlar: list = []
        for r_ in rows:
            alanlar += [k for k in r_ if k not in alanlar]
        with open(sys.argv[1], "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=alanlar)   # satırda olmayan alan boş kalır
            w.writeheader()
            w.writerows(rows)


if __name__ == "__main__":
    main()
