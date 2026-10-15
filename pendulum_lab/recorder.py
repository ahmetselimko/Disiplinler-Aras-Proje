"""Telemetri oturumunu CSV'ye kaydeder (Excel / Matlab / pandas ile açılabilir).

TelemetryReader.sinks listesine eklenir; satırlar okuyucu thread'inden gelir, dosyaya tamponlu yazılır.
"""

from __future__ import annotations

import csv
import threading
import time
from pathlib import Path

from .comm.reader import COLUMNS, mode_name


class Recorder:
    def __init__(self, path: str | Path, meta: dict | None = None):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._f = open(self.path, "x", newline="", encoding="utf-8")   # "x": varsa üzerine yazma
        for k, v in (meta or {}).items():
            self._f.write(f"# {k}: {v}\n")
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

    @staticmethod
    def default_path(folder: str | Path = "kayitlar") -> Path:
        """Zaman damgalı, var olan bir dosyanın üzerine asla yazmayan yol."""
        base = Path(folder) / time.strftime("oturum_%Y%m%d_%H%M%S")
        path, i = base.with_suffix(".csv"), 1
        while path.exists():
            path, i = Path(f"{base}_{i}.csv"), i + 1
        return path
