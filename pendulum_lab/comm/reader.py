"""Seri veriyi ayrı bir thread'de okuyan sınıf (PDF'teki "kesinlikle ayrı Thread" uyarısı).

Akış:  Transport.read() → satırlara böl → parse_line → numpy halka tampona toplu yaz
GUI thread'i yalnızca tampondan kopya alır (latest) ve kuyruktaki olay/yanıtları boşaltır;
hiçbir zaman seri portu beklemez. Satır başına maliyet ~5 µs → 1000+ satır/s rahat.
"""

from __future__ import annotations

import queue
import threading
import time
from collections import deque
import numpy as np

from ..control.supervisor import MODE_CODES, Mode
from ..analysis.summary import LiveTrialTracker
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
    def __init__(self, transport: Transport, capacity: int = 120_000, u_max: float = 12.0):
        self.transport = transport
        self.trials = LiveTrialTracker(u_max=u_max)
        self._command_lock = threading.Lock()
        self._pending_commands = deque(maxlen=512)
        # Replace atomically so GUI metadata reads a stable accepted-settings snapshot.
        self.applied_settings = {}
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
        self.trials.finish()

    @property
    def alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def send(self, cmd: str, **fields) -> None:
        """GUI'den komut gönder (yazma thread-güvenli)."""
        encoded = P.encode_command(cmd, **fields)
        with self._command_lock:
            self.transport.write(encoded)
            self._pending_commands.append((cmd, fields))
        for sink in list(self.sinks):
            if hasattr(sink, "write_event"):
                try:
                    sink.write_event("command_sent", {"cmd": cmd, "fields": fields,
                                     "t": self.last.t if self.last is not None else None})
                except Exception as exc:
                    self._sink_failed(sink, exc)

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
                self.trials.update(msg)
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
                    self.trials.finish("Cihaz yeniden başladı")
                    with self._command_lock:
                        self._pending_commands.clear()
                    self.applied_settings = {}
                if isinstance(msg, P.Ack):
                    with self._command_lock:
                        request = next((r for r in self._pending_commands if r[0] == msg.cmd), None)
                        if request is not None:
                            self._pending_commands.remove(request)
                    if request is not None and msg.ok:
                        fields = request[1]
                        if msg.cmd in ("set_param", "set_gain"):
                            self.applied_settings = {**self.applied_settings, **fields}
                        if msg.cmd == "set_param" and "u_max" in fields:
                            self.trials.set_u_max(float(fields["u_max"]))
                        elif msg.cmd == "pause":
                            self.trials.set_paused(bool(fields.get("on", 1)))
                        elif msg.cmd == "reset":
                            self.trials.finish("Simülasyon sıfırlandı")
                # Record ACKs/events in stream order, after preceding telemetry.
                self._flush(rows)
                rows = []
                for sink in list(self.sinks):
                    if hasattr(sink, "record_message"):
                        try:
                            sink.record_message(msg)
                        except Exception as exc:
                            self._sink_failed(sink, exc)
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
                self._sink_failed(sink, e)

    def _sink_failed(self, sink, exc):
        if sink in self.sinks:
            self.sinks.remove(sink)
        self.messages.put(P.Event("recorder_error", {"msg": str(exc)}))


def mode_name(code: float) -> str:
    try:
        i = int(code)
    except (TypeError, ValueError):
        return "?"
    modes = list(Mode)
    return modes[i].value if 0 <= i < len(modes) else "?"
