#!/usr/bin/env python3
"""Swing-up karar çalışması (#50 Aşama 3): uç kütlesi × geçiş kuralı × hedef enerji oranı × ray × sürtünme telafisi.

Mekatronikle yapılacak görüşme için sayısal dayanak üretir; mevcut koda ve varsayılanlara DOKUNMAZ.
Parametreler Aşama 2'nin dosyalarından (mekatronik ara raporu 23.11.2026, tasarım değeri, ölçüm değil):
config/params_ucsuz.json, config/params.json (+20 g), config/params_50g.json.

Her durum firmware gibi koşulur: simulate_closed_loop, Ts = 2 ms, SensorModel.for_params (12-bit açı + 44 CPR × 19
enkoder kuantalaması), dik bölgede Kalman, θ₀ = 180°, T = 30 s. K her profil için yeniden hesaplanır
(Q = diag(10, 1, 100, 1), R = 0.1, Ts = 2 ms). Geçiş kuralları Supervisor'a doğrudan verilir.

Sağlamlık: her durum 3 gürültü tohumuyla (ölçüme yarım sayımlık beyaz gürültü eklenir, yoksa tohum sonucu
değiştirmezdi) ve gerçek sarkacın m ile l'si birlikte −%5 / +%5 iken (model uyumsuzluğu; kontrolcü nominal
modeli kullanır, tohum 0) tekrar koşulur: durum başına 5 koşu.

Kullanım:
    python tools/swingup_karar.py                 # tam tarama (~900 koşu; 2 çekirdekte ~6 dk)
    python tools/swingup_karar.py --hizli         # küçük alt küme (deneme/test), rapor yazmaz
    python tools/swingup_karar.py --yalniz-rapor  # .json'dan .md'yi yeniden yazar (simülasyon yok)
Çıktı: docs/swingup_karar_calismasi.md ve .json (elle düzenlemeyin; betiği yeniden çalıştırın).
"""

from __future__ import annotations

import argparse
import copy
import itertools
import json
import math
import multiprocessing as mp
import sys
import time
from pathlib import Path

import numpy as np

ROOT_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT_DIR))
sys.path.insert(0, str(ROOT_DIR / "tools"))

from pendulum_lab.analysis.sensitivity import evaluate_result  # noqa: E402
from pendulum_lab.control.lqr import design_lqr  # noqa: E402
from pendulum_lab.control.supervisor import (CATCH_DEG, CATCH_DTH, CATCH_DX, CATCH_X, RELEASE_DEG,  # noqa: E402
                                              Supervisor)
from pendulum_lab.control.swingup import EnergySwingUp  # noqa: E402
from pendulum_lab.model.params import load_params  # noqa: E402
from pendulum_lab.sim.simulator import SensorModel, simulate_closed_loop  # noqa: E402
from rapor_sekilleri import classify  # noqa: E402  (şekiller ve ön değerlendirmeyle aynı sonuç sınıflandırması)

OK_STATUS = ("BAŞARILI", "SINIRDA")             # sonda dengede ve LQR bir kez yakaladıktan sonra hiç bırakmadı

PROFILES = {                                    # ad → parametre dosyası (Aşama 2)
    "uç kütlesiz": "config/params_ucsuz.json",
    "+20 g": "config/params.json",
    "+50 g": "config/params_50g.json",
}
RULES = {                                       # geçiş kuralları (Supervisor argümanları)
    # bizim: arayüz varsayılanları (supervisor.py'deki ortak sabitler; değişirse bu çalışma da izler)
    "bizim": dict(catch_deg=CATCH_DEG, release_deg=RELEASE_DEG, catch_dth=CATCH_DTH, catch_x=CATCH_X,
                  catch_dx=CATCH_DX),
    "mekatronik": dict(catch_deg=10.0, release_deg=25.0, catch_dth=1.5, catch_x=0.25, catch_dx=0.70),
}
RULE_LABELS = {"bizim": f"bizim ({CATCH_DEG:g}°/{RELEASE_DEG:g}°, {CATCH_DTH:g} rad/s, {CATCH_X:.2f} m, {CATCH_DX:g} m/s)",
               "mekatronik": "mekatronik (10°/25°, 1.5 rad/s, 0.25 m, 0.70 m/s)"}
E_RATIOS = (1.00, 1.01, 1.02, 1.05, 1.10)       # 1.01 = mekatroniğin E_ref = 1.02·mgl'si
RAILS = (0.40, 0.60, 0.80)                      # rail_half [m]
FRICTION_COMP = (True, False)                   # True: kontrolcü T_f'yi biliyor; False: fc_volt = 0
VARIANTS = (("t0", 0, 1.00), ("t1", 1, 1.00), ("t2", 2, 1.00), ("-5%", 0, 0.95), ("+5%", 0, 1.05))
Q, R, TS, T = (10.0, 1.0, 100.0, 1.0), 0.1, 0.002, 30.0
NOISE_COUNTS = 0.5                              # ölçüm gürültüsü: σ = yarım sayım (açı ve enkoder)

OUT_MD = ROOT_DIR / "docs" / "swingup_karar_calismasi.md"


def case_key(profile, rule, e_ratio, rail, comp):
    return f"{profile}|{rule}|{e_ratio:.2f}|{rail:.2f}|{'açık' if comp else 'kapalı'}"


def run_one(args) -> dict:
    """Tek koşu. args = (profil, kural, E_ratio, ray, telafi, varyant adı, tohum, m·l çarpanı, T)."""
    profile, rule, e_ratio, rail, comp, vname, seed, k, T_sim = args
    p = load_params(ROOT_DIR / PROFILES[profile])
    p.rail_half = rail
    K, _, _ = design_lqr(p, Q, [[R]], Ts=TS)
    p_ctrl = copy.deepcopy(p)
    if not comp:
        p_ctrl.motor.T_f = 0.0                  # kontrolcü sürtünmeyi bilmiyor; fizikte 3 N var
    real = copy.deepcopy(p)
    real.m, real.l = p.m * k, p.l * k            # model uyumsuzluğu (rapor şekilleriyle aynı tanım)
    sup = Supervisor(p_ctrl, K.ravel(), swingup=EnergySwingUp(E_ratio=e_ratio), **RULES[rule])
    sup.start()
    base = SensorModel.for_params(p)
    sensor = SensorModel.for_params(p, seed=seed, th_noise_std=NOISE_COUNTS * base.th_resolution,
                                    x_noise_std=NOISE_COUNTS * base.x_resolution)
    r = simulate_closed_loop(real, sup, [0.0, 0.0, math.pi, 0.0], T_sim, dt=TS, sensor=sensor)
    modes = np.asarray(r.mode)
    tb = float(r.t[np.argmax(modes == "BALANCE")]) if (modes == "BALANCE").any() else None
    ang = np.abs(np.degrees(np.arctan2(np.sin(r.th), np.cos(r.th))))
    fault = "FAULT" in r.mode
    status = classify(r, evaluate_result(r, real))
    up = status in OK_STATUS
    if fault:
        reason = "ray sınırı (FAULT)"
    elif up:
        reason = ""
    elif status == "DÜŞTÜ":
        reason = "düştü, swing-up yeniden kaldırdı"
    elif status == "OTURMADI":
        reason = "süre sonunda LQR'da ama ±2°'de oturmadı"
    elif tb is not None:
        reason = "yakalandı, tutulamadı"
    else:
        closest = ang[r.t > min(3.0, 0.5 * T_sim)].min()
        rl = RULES[rule]
        inzone = ang < rl["catch_deg"]
        if inzone.any():                       # açı bölgesine girdi ama hız/konum şartları tutmadı
            fails = [name for name, v, lim in (("|θ̇|", r.s[:, 3], rl["catch_dth"]), ("|x|", r.x, rl["catch_x"]),
                                               ("|ẋ|", r.s[:, 1], rl["catch_dx"]))
                     if (np.abs(v[inzone]) >= lim).all()]   # açı bölgesindeki her adımda bu şart tutmadı
            why = ", ".join(fails) if fails else "θ̇, x, ẋ birlikte"
            reason = f"dike geldi ({closest:.0f}°), yakalama şartı sağlanmadı: {why}"
        else:
            reason = f"kalkmadı (dike en fazla {closest:.0f}°)"
    return {"key": case_key(profile, rule, e_ratio, rail, comp), "variant": vname, "up": bool(up), "status": status,
            "t_catch": tb, "max_x": float(np.abs(r.x).max()), "max_u": float(np.abs(r.u).max()),
            "fault": bool(fault), "reason": reason}


def all_jobs(T_sim=T, profiles=None, rules=None, e_ratios=None, rails=None, comps=None, variants=None):
    for prof, rule, er, rail, comp in itertools.product(profiles or PROFILES, rules or RULES, e_ratios or E_RATIOS,
                                                        rails or RAILS, comps or FRICTION_COMP):
        for vname, seed, k in variants or VARIANTS:
            yield (prof, rule, er, rail, comp, vname, seed, k, T_sim)


def run_all(jobs, n_proc=None, progress=True) -> list[dict]:
    jobs = list(jobs)
    out, t0 = [], time.perf_counter()
    with mp.Pool(n_proc) as pool:
        for i, res in enumerate(pool.imap_unordered(run_one, jobs, chunksize=2), 1):
            out.append(res)
            if progress and (i % 20 == 0 or i == len(jobs)):
                el = time.perf_counter() - t0
                print(f"{i}/{len(jobs)}  {el:5.0f} s  (kalan ~{el / i * (len(jobs) - i):4.0f} s)", flush=True)
    return out


# ---------------------------------------------------------------- değerlendirme ve rapor

def summarize(results: list[dict]) -> dict:
    """Durum başına: nominal (t0) koşu ve 5 koşudan kaçının kalktığı."""
    by = {}
    for r in results:
        by.setdefault(r["key"], {})[r["variant"]] = r
    out = {}
    for key, vs in by.items():
        nom = vs.get("t0")
        out[key] = {"nominal": nom, "runs": list(vs.values()), "n_up": sum(v["up"] for v in vs.values()), "n": len(vs),
                    "n_fell": sum(v.get("status") == "DÜŞTÜ" for v in vs.values()),
                    "robust": all(v["up"] for v in vs.values()) and len(vs) == len(VARIANTS),
                    "worst_max_x": max(v["max_x"] for v in vs.values())}
    return out


def cell(s: dict) -> str:
    nom = s["nominal"]
    tag = f" ({s['n_up']}/{s['n']}" + (f", {s['n_fell']} düştü" if s.get("n_fell") else "") + ")"
    if nom["up"]:
        return f"{'✅' if s['robust'] else '⚠️'} {nom['t_catch']:.1f} s / {nom['max_x']:.2f} m{tag}"
    if nom.get("status") == "DÜŞTÜ":
        return f"🔄 düştü, yeniden kalktı (ilk yakalama {nom['t_catch']:.1f} s){tag}"
    return f"❌ {nom['reason']}{tag}"


def conclusions(summ: dict) -> list[str]:
    """Yalnız tablolardaki sonuçlardan çıkan cümleler (betik üretir)."""
    L = []
    robust = {k for k, s in summ.items() if s["robust"]}

    def keys(**f):
        for k in summ:
            prof, rule, er, rail, comp = k.split("|")
            d = dict(profile=prof, rule=rule, e=float(er), rail=float(rail), comp=comp)
            if all(d[a] == b for a, b in f.items()):
                yield k, d

    # 1. uç kütlesi başına sağlam kombinasyon sayısı
    L.append("**Sağlam kombinasyonlar** (3 tohumun hepsinde ve m, l ±%5'te kalkan; ✅):")
    for prof in PROFILES:
        n_all = sum(1 for _ in keys(profile=prof))
        n_ok = sum(1 for k, _ in keys(profile=prof) if k in robust)
        n_ok_nc = sum(1 for k, _ in keys(profile=prof, comp="kapalı") if k in robust)
        L.append(f"- {prof}: {n_ok}/{n_all} durum sağlam; sürtünme telafisi kapalıyken {n_ok_nc}/{n_all // 2}.")
    # 1b. ray uzunluğunun etkisi: aynı (uç kütlesi, kural, E_ratio, telafi) için raylar arasında sonuç değişiyor mu
    groups = {}
    for k, s in summ.items():
        prof, rule, er, rail, comp = k.split("|")
        groups.setdefault((prof, rule, er, comp), []).append((s["robust"], s["nominal"]["up"], s["n_up"]))
    differ = sum(1 for v in groups.values() if len(set(v)) > 1)
    ups = [r["max_x"] for s in summ.values() for r in s["runs"] if r["up"]]
    mx = max(ups) if ups else float("nan")
    L += ["", f"**Ray uzunluğu:** {len(groups)} (uç kütlesi, kural, E_ratio, telafi) grubundan {differ}'inde sonuç "
          f"(sağlamlık, nominal kalkış ya da 5 koşudan kalkan sayısı) ray boyuyla değişti. Kalkan koşularda en büyük "
          f"max |x| {mx:.2f} m."]
    # 2. her uç kütlesi ve kural için sağlam E_ratio'lar ve gereken en kısa ray
    L += ["", "**Kural ve hedef enerji oranına göre gereken en kısa ray** (sağlam olduğu en kısa `rail_half`; "
          "telafi açık / kapalı):", "",
          "| Uç kütlesi | Geçiş kuralı | " + " | ".join(f"E_ratio {e:.2f}" for e in E_RATIOS) + " |",
          "|---|---|" + "---|" * len(E_RATIOS)]
    for prof in PROFILES:
        for rule in RULES:
            cells = []
            for e in E_RATIOS:
                parts = []
                for comp in ("açık", "kapalı"):
                    ok = sorted(d["rail"] for k, d in keys(profile=prof, rule=rule, e=e, comp=comp) if k in robust)
                    parts.append(f"{ok[0]:.2f}" if ok else "—")
                cells.append(" / ".join(parts))
            L.append(f"| {prof} | {rule} | " + " | ".join(cells) + " |")
    # 3. en iyi öneri: tüm raylarda ve iki telafi durumunda sağlam olan (uç kütlesi, kural, E_ratio)
    L += ["", "**Her ray uzunluğunda ve telafi açık/kapalı sağlam olan (uç kütlesi, kural, E_ratio):**"]
    found = False
    for prof in PROFILES:
        for rule in RULES:
            for e in E_RATIOS:
                ks = list(keys(profile=prof, rule=rule, e=e))
                if ks and all(k in robust for k, _ in ks):
                    found = True
                    mx = max(summ[k]["worst_max_x"] for k, _ in ks)
                    L.append(f"- {prof}, {rule}, E_ratio {e:.2f} (5 koşunun en büyük max |x|'i {mx:.2f} m)")
    if not found:
        L.append("- yok")
    L += ["", "**Yalnız telafi açıkken (ilk denemeden önce sürtünme ölçülüp gönderilirse) her rayda sağlam olan:**"]
    found = False
    for prof in PROFILES:
        for rule in RULES:
            for e in E_RATIOS:
                ks = list(keys(profile=prof, rule=rule, e=e, comp="açık"))
                all_ks = list(keys(profile=prof, rule=rule, e=e))
                if ks and all(k in robust for k, _ in ks) and not all(k in robust for k, _ in all_ks):
                    found = True
                    L.append(f"- {prof}, {rule}, E_ratio {e:.2f}")
    if not found:
        L.append("- (yukarıdakilerden başka) yok")
    return L


def markdown(results: list[dict], meta: dict) -> str:
    summ = summarize(results)
    L = ["# Swing-up karar çalışması (#50 Aşama 3)", "",
         f"`tools/swingup_karar.py` ile {meta['date']} tarihinde üretildi ({meta['n_runs']} koşu, {meta['elapsed_s']:.0f} s). "
         "**Elle düzenlemeyin;** parametreler değişince betiği yeniden çalıştırın. Ham sonuçlar: "
         "[`swingup_karar_calismasi.json`](swingup_karar_calismasi.json).", "",
         "> ⚠ **Tasarım değerleriyle (ölçüm değil) üretildi: Aşama 2 değerleri** (mekatronik ara raporu 23.11.2026; "
         "`config/params_ucsuz.json`, `config/params.json` (+20 g), `config/params_50g.json`). Ölçüm gelince sayılar "
         "değişir.", "",
         "## Kurulum", "",
         f"- Simülasyon firmware gibi: Ts = {TS * 1000:g} ms ayrık kontrol, 12-bit açı ve 44 CPR × 19 enkoder "
         f"kuantalaması, dik bölgede Kalman, θ₀ = 180°, T = {meta['T']:g} s. Ölçüme σ = {NOISE_COUNTS:g} sayım beyaz "
         "gürültü eklenir (tohum farkı anlamlı olsun).",
         f"- K her profil ve ray için yeniden hesaplanır: Q = diag{Q}, R = {R:g}, Ts = {TS * 1000:g} ms. "
         "Swing-up kazançları arayüz varsayılanları; değişen yalnız E_ratio.",
         "- Geçiş kuralları: " + "; ".join(f"**{k}**: {v}" for k, v in RULE_LABELS.items()) + ".",
         "- E_ratio bizim tanımımız (aşağıdan ölçülen hedef enerji / 2mgl). **1.01 = mekatroniğin E_ref = "
         "1.02·mgl'si.**",
         "- Sürtünme telafisi **açık**: kontrolcü 3 N'u biliyor (sürtünme ölçümü yapılıp `fc_volt` gönderilmiş gibi); "
         "**kapalı**: `fc_volt` = 0, fizikte yine 3 N.",
         "- Hücre: nominal koşu (tohum 0) için LQR'a geçiş zamanı / max |x|; parantezde 5 koşunun (3 tohum, m ve l "
         "−%5, +%5) kaçında son mod BALANCE. Nominal koşuya da gürültü eklendiği için sayılar gürültüsüz "
         "`mekatronik_on_degerlendirme.md` tablolarıyla birebir karşılaştırılamaz. \"Kalktı\": sonda dengede ve LQR "
         "yakaladıktan sonra hiç bırakmadı (`tools/rapor_sekilleri.py::classify`; LQR'ın bırakıp swing-up'ın "
         "yeniden kaldırdığı koşu \"düştü\" sayılır, kalkmış sayılmaz). ✅ beşinde de kalktı; ⚠️ nominalde kalktı ama "
         "en az birinde kalkmadı; 🔄 nominalde düştü ve yeniden kalktı; ❌ nominalde kalkmadı (sebep).", ""]
    for prof in PROFILES:
        L += [f"## {prof}", ""]
        for comp in ("açık", "kapalı"):
            L += [f"**Sürtünme telafisi {comp}**", "",
                  "| E_ratio | Geçiş kuralı | " + " | ".join(f"ray ±{r:.2f} m" for r in RAILS) + " |",
                  "|---|---|" + "---|" * len(RAILS)]
            for e in E_RATIOS:
                for rule in RULES:
                    cells = []
                    for rail in RAILS:
                        k = case_key(prof, rule, e, rail, comp == "açık")
                        cells.append(cell(summ[k]) if k in summ else "—")
                    lab = f"{e:.2f}" + (" (= mekatronik 1.02·mgl)" if e == 1.01 else "")
                    L.append(f"| {lab} | {rule} | " + " | ".join(cells) + " |")
            L.append("")
    L += ["## Sonuç", "", "Aşağıdaki maddeleri betik tablolardaki sonuçlardan üretir.", ""]
    L += conclusions(summ)
    L += ["", "Bu çalışma yalnız karar verisidir: `params.json`, firmware, `supervisor.py` ve swing-up "
          "varsayılanları değiştirilmedi (#11 ve iki ekibin ortak kararı).", ""]
    return "\n".join(L)


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hizli", action="store_true", help="küçük alt küme (1 durum, kısa T); rapor yazılmaz")
    ap.add_argument("--T", type=float, default=T, help="koşu süresi [s]")
    ap.add_argument("--jobs", type=int, default=None, help="işlem sayısı (varsayılan: çekirdek sayısı)")
    ap.add_argument("--out", type=Path, default=OUT_MD)
    ap.add_argument("--yalniz-rapor", action="store_true",
                    help="simülasyon yapmadan, mevcut .json sonuçlarından .md'yi yeniden yazar")
    args = ap.parse_args()
    if args.yalniz_rapor:
        data = json.loads(args.out.with_suffix(".json").read_text(encoding="utf-8"))
        args.out.write_text(markdown(data["results"], data["meta"]), encoding="utf-8")
        print(f"Rapor yeniden yazıldı: {args.out}")
        return
    if args.hizli:
        jobs = all_jobs(T_sim=min(args.T, 8.0), profiles=["+20 g"], rules=["bizim"], e_ratios=[1.05],
                        rails=[0.40], comps=[True], variants=VARIANTS[:1])
        res = run_all(jobs, args.jobs)
        for r in res:
            print(r)
        return
    t0 = time.perf_counter()
    res = run_all(all_jobs(T_sim=args.T), args.jobs)
    res.sort(key=lambda r: (r["key"], r["variant"]))
    meta = {"date": time.strftime("%d.%m.%Y"), "n_runs": len(res), "elapsed_s": time.perf_counter() - t0,
            "T": args.T, "Q": Q, "R": R, "Ts": TS, "rules": RULES, "e_ratios": E_RATIOS, "rails": RAILS,
            "profiles": PROFILES, "noise_counts": NOISE_COUNTS,
            "variants": [dict(name=n, seed=s, ml_factor=k) for n, s, k in VARIANTS]}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(markdown(res, meta), encoding="utf-8")
    args.out.with_suffix(".json").write_text(json.dumps({"meta": meta, "results": res}, ensure_ascii=False,
                                                        indent=1), encoding="utf-8")
    print(f"Tamamlandı: {meta['elapsed_s']:.0f} s; rapor: {args.out}")


if __name__ == "__main__":
    main()
