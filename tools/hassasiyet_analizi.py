#!/usr/bin/env python3
"""#13 deneylerini yeniden üretir; raporu ve ham JSON sonuçlarını kaydeder."""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT_DIR))

from pendulum_lab.analysis.sensitivity import generate_markdown_report, run_sensitivity_sweep
from pendulum_lab.model.params import load_params


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--params", type=Path, default=ROOT_DIR / "config/params.json")
    parser.add_argument("--duration", type=float, default=12.)
    parser.add_argument("--dt", type=float, default=.002)
    parser.add_argument("--output", type=Path, default=ROOT_DIR / "docs/hassasiyet_raporu.md")
    args = parser.parse_args()
    start = time.perf_counter()
    def progress(path, var, status):
        print(f"{path:18} {var:+.0%}: {status}", flush=True)
    results = run_sensitivity_sweep(load_params(args.params), sim_time=args.duration,
                                    dt=args.dt, progress=progress)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(generate_markdown_report(results), encoding="utf-8")
    # Iraksama: geçersiz JSON NaN yerine null ve açık bir hata/durum
    def clean(value):
        import math
        if isinstance(value, float) and not math.isfinite(value):
            return None
        if isinstance(value, dict):
            return {k: clean(v) for k, v in value.items()}
        if isinstance(value, (list, tuple)):
            return [clean(v) for v in value]
        return value
    args.output.with_suffix(".json").write_text(
        json.dumps(clean(results.to_dict()), ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    print(f"Tamamlandı: {time.perf_counter() - start:.1f} s; rapor: {args.output}")


if __name__ == "__main__":
    main()
