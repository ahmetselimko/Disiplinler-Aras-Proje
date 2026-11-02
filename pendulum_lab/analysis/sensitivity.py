"""Tek seferde bir fiziksel parametrenin saptırıldığı deneyler; nominal kontrolcü sabit kalır."""
from __future__ import annotations

import copy
import math
from dataclasses import asdict, dataclass, field

import numpy as np

from ..control.lqr import design_lqr
from ..control.supervisor import Supervisor
from ..control.swingup import EnergySwingUp
from ..model.params import PendulumParams
from ..sim.simulator import SensorModel, simulate_closed_loop
from .summary import find_settling_time, input_saturation_time

DEFAULT_VARIATIONS = (-.30, -.20, -.10, .10, .20, .30)
I0_37D = 0.2     # Pololu 37D boşta akımı [A] (motor belgesi): kuru sürtünme tahmini T_f ≈ Kt·I₀ (#30)
DEFAULT_PARAMS_TO_TEST = (
    ("l", "Pivot–ağırlık merkezi mesafesi", "m"),
    ("m", "Sarkaç kütlesi", "kg"),
    ("M", "Araba kütlesi", "kg"),
    ("I", "Sarkacın ağırlık merkezi etrafındaki ataleti", "kg·m²"),
    ("b", "Araba viskoz sürtünmesi", "N·s/m"),
    ("c", "Pivot viskoz sürtünmesi", "N·m·s/rad"),
    ("motor.R", "Motor direnci", "Ω"),
    ("motor.Kt", "Motor tork sabiti", "N·m/A"),
    ("motor.Ke", "Motor ters EMK sabiti", "V·s/rad"),
    ("motor.gear", "Redüktör oranı", "1"),
    ("motor.r_pulley", "Kasnak yarıçapı", "m"),
    ("motor.J_rotor", "Motor rotor ataleti", "kg·m²"),
    ("motor.T_f", "Motor Coulomb sürtünme torku", "N·m"),
    ("motor.B_m", "Motor viskoz sürtünmesi", "N·m·s/rad"),
)


@dataclass
class CaseResult:
    balanced: bool
    settling_time: float | None
    max_x: float
    sat_time: float
    final_mode: str
    status: str
    final_hold_s: float = 0.0
    error: str | None = None


@dataclass
class VariationResult:
    variation: float
    nominal_val: float
    perturbed_val: float
    balanced: bool
    settling_time: float | None
    max_x: float
    sat_time: float
    final_mode: str
    status: str
    balance_case: CaseResult | None = None
    swing_case: CaseResult | None = None
    note: str = ""


@dataclass
class ParamSensitivity:
    param_path: str
    description: str
    unit: str
    nominal_val: float
    results: list[VariationResult] = field(default_factory=list)
    criticality: str = "DEĞERLENDİRİLMEDİ"
    reference_scale: float | None = None

    def evaluate_criticality(self):
        tested = [r for r in self.results if r.status != "UYGULANAMAZ"]
        if not tested:
            self.criticality = "DEĞERLENDİRİLEMEDİ"
        elif any(r.status == "HATA" for r in tested):
            self.criticality = "HATA — İNCELENMELİ"
        elif any(not r.balanced for r in tested if abs(r.variation) <= .200001):
            self.criticality = "KRİTİK (Çok Hassas)"
        elif any(not r.balanced or r.status == "SINIRDA" for r in tested):
            self.criticality = "ORTA HASSASİYET"
        else:
            self.criticality = "TOLERANSLI (yalnız taranan koşullarda)"


class SensitivitySweep(list):
    """Liste gibi kullanılan sonuçlar; raporu yeniden üretmek için gereken tüm girdilerle birlikte."""
    def __init__(self, nominal, settings, baseline):
        super().__init__()
        self.nominal, self.settings, self.baseline = nominal, settings, baseline

    def to_dict(self):
        return {"nominal": self.nominal, "settings": self.settings,
                "baseline": {k: asdict(v) for k, v in self.baseline.items()},
                "results": [asdict(r) for r in self]}


def set_nested_attr(obj, attr_path, value):
    parts = attr_path.split(".")
    for part in parts[:-1]:
        obj = getattr(obj, part)
    setattr(obj, parts[-1], value)


def get_nested_attr(obj, attr_path):
    for part in attr_path.split("."):
        obj = getattr(obj, part)
    return float(obj)


def evaluate_result(r, p, threshold_deg=2.0, hold_s=1.0):
    """Son bekleme penceresi boyunca BALANCE modu ve açı bandı şartını arar."""
    angle = np.abs(np.degrees(np.arctan2(np.sin(r.th), np.cos(r.th))))
    max_x = float(np.max(np.abs(r.x)))
    settle = find_settling_time(r.t, r.th, threshold_deg, hold_s)
    sat_time, _ = input_saturation_time(r.t, r.u, p.u_max)
    good = (angle <= threshold_deg) & (np.asarray(r.mode) == "BALANCE")
    bad = np.flatnonzero(~good)
    start = int(bad[-1] + 1) if len(bad) else 0
    final_hold = float(r.t[-1] - r.t[start]) if start < len(r.t) else 0.0
    rail_hit = max_x >= p.rail_half - 1e-9
    fault = "FAULT" in r.mode
    balanced = bool(final_hold >= hold_s - 1e-12 and not fault and not rail_hit)
    status = ("RAY SINIRI" if rail_hit else "FAULT" if fault else
              "SINIRDA" if balanced and max_x > .75 * p.rail_half else
              "BAŞARILI" if balanced else "BAŞARISIZ")
    return CaseResult(balanced, settle, max_x, sat_time, str(r.mode[-1]), status, final_hold)


def _run_case(nominal, real, K, sim_time, dt, initial_deg, threshold_deg, hold_s,
              swingup=None, catch_deg=15.0, release_deg=30.0):
    sup = Supervisor(copy.deepcopy(nominal), K, swingup=copy.deepcopy(swingup),
                     catch_deg=catch_deg, release_deg=release_deg)
    sup.start()
    try:
        r = simulate_closed_loop(real, sup, [0., 0., math.radians(initial_deg), 0.], sim_time,
                                 dt=dt, sensor=SensorModel(seed=0), end_stops=True)
    except (FloatingPointError, OverflowError) as exc:
        return CaseResult(False, None, math.nan, math.nan, "HATA", "HATA", error=str(exc))
    return evaluate_result(r, nominal, threshold_deg, hold_s)


def run_single_simulation(nominal, real, K, sim_time=12.0, dt=.002):
    """Swing-up deneyi için uyumluluk sarmalayıcısı."""
    case = _run_case(nominal, real, K, sim_time, dt, 180., 2., 1.)
    return (case.balanced, case.settling_time, case.max_x, case.sat_time, case.final_mode, case.status)


def _zero_scales(p):
    """Sıfır nominalli parametreler için keşif amaçlı mutlak ölçekler; ölçüm ya da nominal değer değildir."""
    return {"I": p.m * p.l**2, "b": .1, "c": .0005,
            "motor.J_rotor": p.M * (p.motor.r_pulley / p.motor.gear)**2,
            # 37D tahmininin 5 katı: +%10/20/30 sütunları tahminin 0.5/1/1.5 katı (kontrolcü T_f'yi bilmez)
            "motor.T_f": 5 * p.motor.Kt * I0_37D,
            "motor.B_m": max(p.b, .1) * (p.motor.r_pulley / p.motor.gear)**2}


def run_sensitivity_sweep(nominal=None, K=None, params_to_test=DEFAULT_PARAMS_TO_TEST,
                          variations=DEFAULT_VARIATIONS, sim_time=12.0, dt=.002,
                          threshold_deg=2.0, hold_s=1.0, Q=(10., 1., 100., 1.), R=.1,
                          swingup=None, catch_deg=15., release_deg=30., zero_scales=None,
                          progress=None):
    """Sabit nominal K/model ile birbirinden bağımsız swing-up ve 5° denge deneyleri.

    Sıfır nominalli parametrelerde belgelenmiş mutlak ölçeklerin pozitif oranları kullanılır.
    Negatif değerler atlanır; sessizce kırpılmaz ve toleranslı sayılmaz.
    """
    nominal = copy.deepcopy(nominal or PendulumParams())
    if not all(math.isfinite(v) and v > 0 for v in (dt, sim_time, threshold_deg, hold_s)) or sim_time < hold_s:
        raise ValueError("Pozitif süre/adım/eşik ve sim_time >= hold_s gerekli")
    variations = tuple(float(v) for v in variations)
    if not variations or any(not math.isfinite(v) or v <= -1 for v in variations) or len(set(variations)) != len(variations):
        raise ValueError("Sapmalar benzersiz, sonlu ve -1'den büyük olmalı")
    Q = np.asarray(Q, dtype=float).tolist()
    designed_here = K is None
    if designed_here:
        K, _, _ = design_lqr(nominal, Q, R, Ts=dt)
    K = np.asarray(K, dtype=float).reshape(-1)
    if K.shape != (4,) or not np.all(np.isfinite(K)):
        raise ValueError("K sonlu dört eleman içermeli")
    su = copy.deepcopy(swingup or EnergySwingUp())
    scales = _zero_scales(nominal)
    scales.update(zero_scales or {})
    if any(not math.isfinite(v) or v <= 0 for v in scales.values()):
        raise ValueError("Sıfır nominal parametre referansları pozitif olmalı")
    settings = dict(sim_time=sim_time, dt=dt, threshold_deg=threshold_deg, hold_s=hold_s,
                    Q=Q if designed_here else None, R=R if designed_here else None,
                    K_source="nominal Q/R ile tasarlandı" if designed_here else "dışarıdan verilen sabit K",
                    K=K.tolist(), variations=variations,
                    swingup=asdict(su), catch_deg=catch_deg, release_deg=release_deg,
                    sensor={"th_resolution": 2*math.pi/4096,
                    "x_resolution": .000125, "noise_std": 0., "seed": 0},
                    initial_angles_deg={"swing": 180., "balance": 5.}, zero_scales=scales)
    settings.update(substeps=4, vel_fc_hz=25., delay_steps=0, end_stops=True)

    def run(real, initial):
        return _run_case(nominal, real, K, sim_time, dt, initial, threshold_deg, hold_s,
                         su, catch_deg, release_deg)

    baseline = {name: run(nominal, angle) for name, angle in (("swing", 180.), ("balance", 5.))}
    results = SensitivitySweep(nominal.to_dict(), settings, baseline)
    baseline_ok = all(r.balanced for r in baseline.values())
    for path, desc, unit in params_to_test:
        value = get_nested_attr(nominal, path)
        item = ParamSensitivity(path, desc, unit, value)
        if value == 0:
            item.reference_scale = scales.get(path)
        for var in variations:
            note = ""
            target = value * (1 + var) if value else (scales.get(path, 0.) * var)
            if path.startswith("motor.") and nominal.input_mode != "voltage":
                note = "Motor parametresi bu giriş modunda kullanılmıyor"
            elif value == 0 and (var <= 0 or path not in scales):
                note = "Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır"
            if note:
                item.results.append(VariationResult(var, value, target, False, None, 0., 0., "—", "UYGULANAMAZ", note=note))
                continue
            real = copy.deepcopy(nominal)
            set_nested_attr(real, path, target)
            swing, balance = run(real, 180.), run(real, 5.)
            ok = swing.balanced and balance.balanced
            status = "BAŞARILI" if ok else "BAŞARISIZ"
            if "HATA" in (swing.status, balance.status):
                status = "HATA"
            elif ok and "SINIRDA" in (swing.status, balance.status):
                status = "SINIRDA"
            item.results.append(VariationResult(var, value, target, ok, swing.settling_time,
                                max(swing.max_x, balance.max_x), swing.sat_time, swing.final_mode,
                                status, balance, swing))
            if progress:
                progress(path, var, status)
        item.evaluate_criticality()
        if not baseline_ok:
            item.criticality = "NOMİNAL BAŞARISIZ — SIRALANAMAZ"
        results.append(item)
    return results


def generate_markdown_report(results):
    """Yalnız verilen deneylerin desteklediği sonuçları yazar."""
    lines = ["# Parametre Hassasiyet Analizi Raporu", "",
             "Issue #13 — Tek seferde bir fiziksel parametre değiştirilir; nominal model ve K sabit kalır.",
             "Gerçek donanım doğrulaması değildir. Parametreler yer tutucu olabilir; sonuçlar yalnız bu modele,",
             "başlangıç durumlarına, sensöre ve deney süresine aittir. Birleşik hatalar ayrıca test edilmelidir.", ""]
    if hasattr(results, "settings"):
        import json
        cfg = results.settings
        lines += ["## Tekrarlanabilir deney ayarları", "",
                  f"- Her deney: {cfg['sim_time']:g} s; Ts = {cfg['dt']:g} s; sensör: AS5600 12-bit, gürültü yok.",
                  f"- Başarı: son en az {cfg['hold_s']:g} s boyunca BALANCE ve |θ| ≤ {cfg['threshold_deg']:g}°; FAULT ve ray teması yok.",
                  "- Ayrı başlangıçlar: aşağıdan 180° swing-up; dik çevresinde 5° denge.",
                  "- Doyum: sol örnek sabit kabul edilerek |u| ≥ u_max − 0.001 integralidir.",
                  "- Nominal kontroller:"]
        for name, case in results.baseline.items():
            lines.append(f"  - {name}: {case.status}; son denge süresi {case.final_hold_s:.3f} s")
        lines += ["", "Tüm fiziksel parametreler, K, Q/R ve kontrol ayarları:", "", "```json",
                  json.dumps({"nominal": results.nominal, "settings": cfg}, ensure_ascii=False, indent=2), "```", ""]
    lines += ["## Ölçüm öncelikleri", "", "| Parametre | Sonuç | Ölçüm önerisi |", "|---|---|---|"]
    def rank(item):
        return 0 if "KRİTİK" in item.criticality else 1 if "ORTA" in item.criticality else 2
    for item in sorted(results, key=rank):
        advice = ("Önce nominal başarısızlığı çözün; hassasiyet sıralaması yapılamaz." if "NOMİNAL" in item.criticality else
                  "Ölçümü ve model varsayımını öncelikle doğrulayın; küçük tarama sapmalarında başarısızlık var." if "KRİTİK" in item.criticality else
                  "Ölçümü doğrulayın; taramada başarısızlık veya ray sınırına yaklaşma var." if "ORTA" in item.criticality else
                  "Bu koşullarda başarısızlık gözlenmedi; daha geniş tolerans için kanıt değildir." if "TOLERANSLI" in item.criticality else
                  "Sonuç üretilemedi; model/girdi ve hata ayrıntılarını kontrol edin.")
        if item.reference_scale is not None:
            advice += " Nominal sıfır: bu, yüzde tolerans değil ek atalet/sürtünme deneyidir."
        lines.append(f"| `{item.param_path}` — {item.description} | {item.criticality} | {advice} |")
    variations = sorted({r.variation for item in results for r in item.results})
    lines += ["", "## Parametre sapma tablosu", "",
              "✓ iki deney başarılı; ⚠ başarılı fakat rayın %75'inden fazla kullanılmış; ✗ başarısız; — uygulanamaz.",
              "Sıfır nominal satırlarda +% değerleri, aşağıda belirtilen mutlak referansın oranıdır.", "",
              "| Parametre | Nominal | " + " | ".join(f"{v:+.0%}" for v in variations) + " |",
              "|---|---|" + "---|" * len(variations)]
    for item in results:
        by_var = {r.variation: r for r in item.results}
        cells = [{"BAŞARILI": "✓", "SINIRDA": "⚠", "UYGULANAMAZ": "—"}.get(by_var[v].status, "✗")
                 if v in by_var else "—" for v in variations]
        lines.append(f"| `{item.param_path}` | {item.nominal_val:.6g} {item.unit} | " + " | ".join(cells) + " |")
    lines += ["", "## Deney ayrıntıları", "",
              "İlk dengeleme geçmişteki ilk kesintisiz bant girişidir; başarı kararı son denge penceresine dayanır.", ""]
    for item in results:
        lines += [f"### {item.param_path} — {item.description}", ""]
        if item.reference_scale is not None:
            lines.append(f"Sıfır nominal için mutlak referans: {item.reference_scale:.8g} {item.unit}; yalnız pozitif ekleme yapılır.")
        lines += ["", "| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |",
                  "|---|---|---|---|---|---|---|---|"]
        for r in item.results:
            for name, case in (("swing-up", r.swing_case), ("denge", r.balance_case)):
                if case is None:
                    continue
                settle = "—" if case.settling_time is None else f"{case.settling_time:.3f}"
                lines.append(f"| {r.variation:+.0%} | {r.perturbed_val:.8g} | {name} | {case.status} | {settle} | {case.final_hold_s:.3f} | {case.max_x:.4f} | {case.sat_time:.3f} |")
                if case.error:
                    lines.append(f"\nHata: `{case.error}`\n")
            if r.note:
                lines.append(f"\n{r.variation:+.0%}: {r.note}.\n")
    lines += ["", "## Sınırlar ve yeniden çalıştırma", "",
              "- Sıfır nominal referansları keşif ölçekleridir; ölçülmüş motor verisi değildir.",
              "- I için referans m·l²; rotor için arabaya yansıması M olan atalet; B_m için max(b, 0.1) N·s/m kullanılır.",
              "- T_f için referans, Pololu 37D tahmininin (Kt·I₀, I₀ = 0.2 A) 5 katıdır: +%10/20/30 sütunları tahminin "
              "0.5/1/1.5 katıdır. Kontrolcü T_f'yi bilmez (telafi kapalı, #30); arayüzdeki Sürtünme Ölç ile ölçülüp "
              "telafi açılırsa bu sonuç değişir. Sürtünme ölçülmeden gerçek denemeye geçilmemelidir.",
              "- Gerçek I / motor kayıpları ölçüldüğünde params.json güncellenmeli; ardından ±% taraması yeniden çalıştırılmalı.",
              "- Motor parametreleri yalnız voltage modunda değerlendirilir. V_max tek başına kuvvet modelini değiştirmediğinden taranmaz; giriş sınırı u_max sabittir.",
              "- Kontrolcü yeniden tasarlanmaz; fiziksel parametre uyumsuzluğu ölçülür. Ölçüm ölçek hataları, gecikme ve birleşik sapmalar bu deneyin dışında kalır.",
              "- Çalıştır: `python tools/hassasiyet_analizi.py`; makine tarafından okunabilir sonuçlar JSON dosyasına da yazılır."]
    return "\n".join(lines) + "\n"
