"""#50 Aşama 2: mekatronik ara raporunun (23.11.2026) tasarım değerleri parametre dosyalarında ve firmware'de."""

import json
import math
import re
from pathlib import Path

import pytest

from pendulum_lab.control.supervisor import CATCH_DEG, RELEASE_DEG
from pendulum_lab.control.swingup import EnergySwingUp
from pendulum_lab.filters import ENC_CPR_MOTOR, encoder_resolution
from pendulum_lab.model.motor import max_cart_force
from pendulum_lab.model.params import load_params

ROOT = Path(__file__).resolve().parents[1]
R_PULLEY = 0.009549
GEAR = 19.0
# dosya: (m, l, I_pivot) — Tablo 15
TABLO_15 = {"params_ucsuz.json": (0.085, 0.0637, 0.000986),
            "params.json": (0.105, 0.0992, 0.002236),
            "params_50g.json": (0.135, 0.1327, 0.004111)}


@pytest.mark.parametrize("dosya", list(TABLO_15))
def test_parameter_files_follow_report(dosya):
    p = load_params(ROOT / "config" / dosya)
    m, l, I_pivot = TABLO_15[dosya]
    assert (p.m, p.l) == (m, l)
    assert p.I == pytest.approx(I_pivot - m * l * l, abs=2e-6)            # ağırlık merkezi etrafında
    assert p.M == pytest.approx(1.10 - m + 0.26 + 0.025, abs=1e-9)        # 1.10 kg sarkaç dahil
    assert p.placeholder is True                                         # tasarım değeri, ölçüm değil
    assert p.rail_half == 0.4 and p.input_mode == "voltage" and p.u_max == 12.0
    mo = p.motor
    assert (mo.gear, mo.r_pulley, mo.V_max) == (GEAR, R_PULLEY, 12.0)
    assert mo.alpha == pytest.approx(2.26, rel=1e-4)                     # Tablo 16
    assert mo.beta == pytest.approx(48.41, rel=1e-4)
    assert mo.reflected_mass == pytest.approx(1.346, rel=1e-3)           # Bölüm 14.1
    assert p.F_coulomb == pytest.approx(3.0, rel=1e-3)
    assert max_cart_force(p) == pytest.approx(27.1, abs=0.1)             # Tablo 16: 27.07 N
    assert mo.alpha * mo.V_max / mo.beta == pytest.approx(0.56, abs=0.005)  # sürtünmesiz en yüksek hız
    assert "23.11" in json.loads((ROOT / "config" / dosya).read_text(encoding="utf-8"))["note"]


def test_default_file_is_20g_and_matches_preliminary_study():
    a = load_params(ROOT / "config" / "params.json").to_dict()
    b = load_params(ROOT / "config" / "params_mekatronik_tasarim.json").to_dict()
    for k in ("note", "placeholder"):
        a.pop(k), b.pop(k)
    assert a["I"] == pytest.approx(b["I"], abs=1e-6)
    a.pop("I"), b.pop("I")
    assert a == b


def test_encoder_44_cpr_everywhere():
    assert ENC_CPR_MOTOR == 44.0 == 836 / 19
    ino = (ROOT / "firmware" / "esp32_pendulum" / "esp32_pendulum.ino").read_text(encoding="utf-8")
    assert float(re.search(r"ENC_CPR_MOTOR\s*=\s*([\d.]+)f", ino).group(1)) == 44.0
    p = load_params(ROOT / "config" / "params.json")
    assert encoder_resolution(p) == pytest.approx(0.060 / 836, rel=1e-3)     # ≈ 0.072 mm
    gains = (ROOT / "firmware" / "esp32_pendulum" / "gains.h").read_text(encoding="utf-8")
    assert "#define MOTOR_GEAR         19.0000f" in gains


def test_swingup_energy_ratio_default_105_everywhere():
    assert EnergySwingUp().E_ratio == 1.05
    gains = (ROOT / "firmware" / "esp32_pendulum" / "gains.h").read_text(encoding="utf-8")
    assert "#define SU_E_RATIO         1.0500f" in gains
    ino = (ROOT / "firmware" / "esp32_pendulum" / "esp32_pendulum.ino").read_text(encoding="utf-8")
    assert "#define SU_E_RATIO 1.05f" in ino
    # mekatroniğin E_ref = 1.02·mgl'si (dikte +mgl) bizim tanımımızda 1.01
    p = load_params(ROOT / "config" / "params.json")
    mgl = p.m * p.g * p.l
    assert EnergySwingUp(E_ratio=1.01).target_energy(p) == pytest.approx(1.02 * mgl - mgl)


def test_catch_release_defaults_have_single_source():
    import tools_path  # noqa: F401  (tools/ dizinini içe aktarılabilir yapar)
    import rapor_sekilleri
    from gui.state import AppState
    from pendulum_lab.export import gains_header
    from pendulum_lab.model.params import PendulumParams
    st = AppState(ROOT / "config" / "params.json")
    assert (st.catch_deg, st.release_deg) == (CATCH_DEG, RELEASE_DEG)
    assert rapor_sekilleri.CATCH_DEG == CATCH_DEG and rapor_sekilleri.RELEASE_DEG == RELEASE_DEG
    assert rapor_sekilleri.DEFAULT_E_RATIO == EnergySwingUp().E_ratio
    h = gains_header(PendulumParams(), [0, 0, 0, 0], 0.002)
    assert f"#define CATCH_RAD          {math.radians(CATCH_DEG):.6f}f" in h
    assert f"#define RELEASE_RAD        {math.radians(RELEASE_DEG):.6f}f" in h
