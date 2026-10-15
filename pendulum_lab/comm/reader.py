"""Seri veriyi ayrı bir thread'de okuyan sınıf (PDF'teki "kesinlikle ayrı Thread" uyarısı).

Akış:  Transport.read() → satırlara böl → parse_line → numpy halka tampona toplu yaz
GUI thread'i yalnızca tampondan kopya alır (latest) ve kuyruktaki olay/yanıtları boşaltır;
hiçbir zaman seri portu beklemez. Satır başına maliyet ~5 µs → 1000+ satır/s rahat.
"""

from __future__ import annotations

import queue
import threading
import time
import numpy as np

from ..control.supervisor import MODE_CODES, Mode
from . import protocol as P
from .transport import Transport

COLUMNS = ("t", "x", "dx", "th", "dth", "u", "mode")
_MODE_TO_CODE = {m.value: float(c) for m, c in MODE_CODES.items()}


class RingBuffer:
    """Sabit boyutlu, thread-güvenli satır tamponu (n_satır × n_sütun float64)."""

    def __init__(self, capacity: int, ncols: int):
        self._data = np.zeros((capacity, ncols))
        self._cap = capacity
        self._idx = 0          # bir sonraki yazma yeri
        self._count = 0
        self._lock = threading.Lock()

    def extend(self, rows: np.ndarray) -> None:
        rows = np.atleast_2d(rows)
        n = len(rows)
        if n == 0:
            return
        if n >= self._cap:
            rows, n = rows[-self._cap:], self._cap
        with self._lock:
            end = self._idx + n
            if end <= self._cap:
                self._data[self._idx:end] = rows
            else:
                k = self._cap - self._idx
                self._data[self._idx:] = rows[:k]
                self._data[:n - k] = rows[k:]
            self._idx = end % self._cap
            self._count = min(self._cap, self._count + n)

    def latest(self, n: int | None = None) -> np.ndarray:
        """Son n satır, eskiden yeniye sıralı KOPYA."""
        with self._lock:
            n = self._count if n is None else min(n, self._count)
            start = (self._idx - n) % self._cap
            if start + n <= self._cap:
                return self._data[start:start + n].copy()
            return np.concatenate([self._data[start:], self._data[:self._idx]])

    def clear(self) -> None:
        with self._lock:
            self._idx = self._count = 0

    def __len__(self) -> int:
        return self._count


class TelemetryReader:
    def __init__(self, transport: Transport, capacity: int = 120_000):
        self.transport = transport
        self.buffer = RingBuffer(capacity, len(COLUMNS))
        self.messages: "queue.Queue[P.Event | P.Ack]" = queue.Queue()
        self.sinks: list = []              # rows(list[tuple]) alan nesneler, ör. Recorder
        self.last: P.Telemetry | None = None
        self.lines = 0
        self.parse_errors = 0
        self.dropped = 0
        self.error: Exception | None = None
        self._last_n = None
        self.rows_total = 0
        self._rate = 0.0
        self._rate_t0 = time.perf_counter()
        self._rate_n0 = 0
        self._running = False
        self._thread: threading.Thread | None = None

    # ---------------------------------------------------------------- kontrol
    def start(self):
        if not self.transport.is_open:
            self.transport.open()
        self._running = True
        self._thread = threading.Thread(target=self._run, name="TelemetryReader", daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        self.transport.close()

    @property
    def alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def send(self, cmd: str, **fields) -> None:
        """GUI'den komut gönder (yazma thread-güvenli)."""
        self.transport.write(P.encode_command(cmd, **fields))

    @property
    def rate_hz(self) -> float:
        """Telemetri hızı (okuyucu thread'i yarım saniyede bir günceller)."""
        return self._rate

    def _update_rate(self):
        now = time.perf_counter()
        if now - self._rate_t0 >= 0.5:
            self._rate = (self.rows_total - self._rate_n0) / (now - self._rate_t0)
            self._rate_t0, self._rate_n0 = now, self.rows_total

    # ---------------------------------------------------------------- thread
    def _run(self):
        pending = b""
        try:
            while self._running:
                data = self.transport.read(0.05)
                self._update_rate()
                if not data:
                    continue
                pending += data
                *lines, pending = pending.split(b"\n")
                if len(pending) > 4096:      # satır sonu hiç gelmiyor → çöp, at
                    pending = b""
                    self.parse_errors += 1
                self._handle_lines(lines)
        except Exception as e:  # port koptu vb. — GUI'de gösterilir
            self.error = e
            self._running = False

    def _handle_lines(self, lines: list[bytes]):
        rows = []
        for line in lines:
            line = line.strip()
            if not line:
                continue
            self.lines += 1
            try:
                msg = P.parse_line(line)
            except P.ProtocolError:
                self.parse_errors += 1
                continue
            if isinstance(msg, P.Telemetry):
                if msg.n >= 0:
                    if self._last_n is not None and msg.n > self._last_n + 1:
                        self.dropped += msg.n - self._last_n - 1
                    self._last_n = msg.n
                rows.append((msg.t, msg.x, msg.dx, msg.th, msg.dth, msg.u, _MODE_TO_CODE.get(msg.mode, -1.0)))
                self.last = msg
            else:
                if isinstance(msg, P.Event) and msg.name == "boot":
                    # cihaz yeniden başladı: zaman ekseni sıfırlandı → eski veriyle karıştırma
                    self._flush(rows)
                    rows = []
                    self.buffer.clear()
                    self._last_n = None
                self.messages.put(msg)
        self._flush(rows)

    def _flush(self, rows):
        if not rows:
            return
        self.rows_total += len(rows)
        self.buffer.extend(np.array(rows))
        for sink in list(self.sinks):
            try:
                sink.write_rows(rows)
            except Exception as e:  # ör. disk dolu: kaydı durdur ama okumaya devam et
                if sink in self.sinks:
                    self.sinks.remove(sink)
                self.messages.put(P.Event("recorder_error", {"msg": str(e)}))


def mode_name(code: float) -> str:
    try:
        i = int(code)
    except (TypeError, ValueError):
        return "?"
    modes = list(Mode)
    return modes[i].value if 0 <= i < len(modes) else "?"
