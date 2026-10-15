"""Bayt taşıyıcıları: gerçek seri port ve sanal ESP32. Okuyucu (reader.py) ikisini aynı şekilde kullanır."""

from __future__ import annotations

import queue
import threading
from abc import ABC, abstractmethod


class Transport(ABC):
    name = "transport"

    @abstractmethod
    def open(self) -> None: ...

    @abstractmethod
    def close(self) -> None: ...

    @abstractmethod
    def read(self, timeout: float = 0.05) -> bytes:
        """Gelen baytlar (yoksa b''). En fazla `timeout` saniye bekler."""

    @abstractmethod
    def write(self, data: bytes) -> None: ...

    @property
    @abstractmethod
    def is_open(self) -> bool: ...


class SerialTransport(Transport):
    """pySerial. ESP32 açılışta DTR/RTS ile resetlenmesin diye ikisi de kapalı tutulur.

    Okuma zaman aşımı açılışta BİR KEZ ayarlanır: pySerial'de `ser.timeout = ...` Windows'ta her
    seferinde portu yeniden yapılandırır (SetCommState, DTR/RTS) — her okumada yapılmamalı.
    """

    READ_TIMEOUT = 0.05

    def __init__(self, port: str, baudrate: int = 921600):
        self.port, self.baudrate = port, baudrate
        self.name = f"{port} @ {baudrate}"
        self._ser = None
        self._wlock = threading.Lock()

    @staticmethod
    def list_ports() -> list[tuple[str, str]]:
        from serial.tools import list_ports
        return [(p.device, p.description) for p in list_ports.comports()]

    def open(self) -> None:
        import serial
        ser = serial.Serial()
        ser.port, ser.baudrate, ser.timeout, ser.write_timeout = self.port, self.baudrate, self.READ_TIMEOUT, 0.5
        ser.dtr = ser.rts = False
        ser.open()
        ser.reset_input_buffer()
        self._ser = ser

    def close(self) -> None:
        if self._ser is not None:
            self._ser.close()
            self._ser = None

    def read(self, timeout: float = 0.05) -> bytes:
        ser = self._ser
        if ser is None:
            return b""
        n = ser.in_waiting
        return ser.read(n if n else 1)  # n>0: hemen döner; yoksa 1 bayt için READ_TIMEOUT kadar bekler

    def write(self, data: bytes) -> None:
        with self._wlock:
            self._ser.write(data)

    @property
    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open


class SimTransport(Transport):
    """Sanal ESP32'ye (mock_device.MockDevice) bağlanır — donanım yokken geliştirme ve test için."""

    def __init__(self, device=None):
        from .mock_device import MockDevice
        self.device = device or MockDevice()
        self.name = "Sanal ESP32 (simülasyon)"
        self._rx: "queue.Queue[bytes]" = queue.Queue()
        self._open = False

    def open(self) -> None:
        self.device.output = self._rx.put
        self.device.start()
        self._open = True

    def close(self) -> None:
        self.device.stop()
        self._open = False

    def read(self, timeout: float = 0.05) -> bytes:
        try:
            chunks = [self._rx.get(timeout=timeout)]
        except queue.Empty:
            return b""
        while True:
            try:
                chunks.append(self._rx.get_nowait())
            except queue.Empty:
                return b"".join(chunks)

    def write(self, data: bytes) -> None:
        self.device.feed(data)

    @property
    def is_open(self) -> bool:
        return self._open
