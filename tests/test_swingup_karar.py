"""#50 Aşama 3: swing-up karar çalışması aracı (tools/swingup_karar.py) küçük bir alt kümeyle hatasız çalışır."""

import json

import pytest

import tools_path  # noqa: F401
import swingup_karar as SK
from pendulum_lab.control.supervisor import Supervisor
from pendulum_lab.model.params import load_params


def test_profiles_are_stage2_parameter_files():
    for prof, path in SK.PROFILES.items():
        p = load_params(SK.ROOT_DIR / path)
        assert p.placeholder and p.motor.gear == 19.0
    assert load_params(SK.ROOT_DIR / SK.PROFILES["+20 g"]).m == 0.105
    assert SK.E_RATIOS == (1.00, 1.01, 1.02, 1.05, 1.10) and SK.RAILS == (0.40, 0.60, 0.80)
    assert SK.RULES["mekatronik"] == dict(catch_deg=10.0, release_deg=25.0, catch_dth=1.5, catch_x=0.25,
                                         catch_dx=0.70)


def test_rules_reach_the_supervisor():
    p = load_params(SK.ROOT_DIR / SK.PROFILES["+20 g"])
    sup = Supervisor(p, [0, 0, 0, 0], **SK.RULES["mekatronik"])
    assert (round(sup.catch, 6), sup.catch_dth, sup.catch_x, sup.catch_dx) == (0.174533, 1.5, 0.25, 0.70)


def test_small_subset_runs_and_report_is_generated():
    jobs = list(SK.all_jobs(T_sim=6.0, profiles=["+20 g"], rules=["bizim"], e_ratios=[1.05], rails=[0.40],
                            comps=[True]))
    assert len(jobs) == len(SK.VARIANTS)
    res = [SK.run_one(j) for j in jobs[:2]] + [dict(SK.run_one(jobs[0]), variant=v) for v, *_ in SK.VARIANTS[2:]]
    r0 = res[0]
    assert r0["up"] and r0["t_catch"] == pytest.approx(4.32, abs=0.2) and r0["max_x"] < 0.4
    md = SK.markdown(res, {"date": "test", "n_runs": len(res), "elapsed_s": 0.0, "T": 6.0})
    assert "## +20 g" in md and "✅" in md and "## Sonuç" in md and "1.01 (= mekatronik 1.02·mgl)" in md
    json.dumps(res)                                   # ham sonuçlar JSON'a yazılabilir
