"""Bağlantı çubuğu: seri port / sanal ESP32 seçimi."""

from __future__ import annotations

import copy

from PyQt5 import QtWidgets

from pendulum_lab.comm.mock_device import MockDevice
from pendulum_lab.comm.reader import TelemetryReader
from pendulum_lab.comm.transport import SerialTransport, SimTransport

SIM_LABEL = "★ Sanal ESP32 (simülasyon)"
BAUDS = ["115200", "230400", "460800", "921600", "2000000"]


class ConnectionPanel(QtWidgets.QWidget):
    def __init__(self, state, parent=None):
        super().__init__(parent)
        self.state = state
        lay = QtWidgets.QHBoxLayout(self)
        lay.setContentsMargins(6, 4, 6, 4)
        self.port = QtWidgets.QComboBox()
        self.port.setMinimumWidth(260)
        self.refresh_btn = QtWidgets.QPushButton("⟳")
        self.refresh_btn.setToolTip("Portları yenile")
        self.refresh_btn.setFixedWidth(32)
        self.baud = QtWidgets.QComboBox()
        self.baud.addItems(BAUDS)
        self.baud.setCurrentText("921600")
        self.connect_btn = QtWidgets.QPushButton("Bağlan")
        self.connect_btn.setMinimumWidth(110)
        for w in (QtWidgets.QLabel("Port:"), self.port, self.refresh_btn, QtWidgets.QLabel("Baud:"),
                  self.baud, self.connect_btn):
            lay.addWidget(w)
        lay.addStretch(1)
        self.refresh_btn.clicked.connect(self.refresh)
        self.connect_btn.clicked.connect(self.toggle)
        state.connection_changed.connect(self._on_conn)
        self.refresh()

    def refresh(self):
        current = self.port.currentData()
        self.port.clear()
        self.port.addItem(SIM_LABEL, "SIM")
        try:
            for dev, desc in SerialTransport.list_ports():
                self.port.addItem(f"{dev} — {desc}", dev)
        except Exception as e:
            self.state.log.emit(f"Port listesi alınamadı: {e}")
        idx = self.port.findData(current)
        self.port.setCurrentIndex(max(0, idx))

    def toggle(self):
        if self.state.connected:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        dev = self.port.currentData()
        if dev == "SIM":
            transport = SimTransport(MockDevice(params=copy.deepcopy(self.state.p), Ts=self.state.Ts))
        else:
            transport = SerialTransport(dev, int(self.baud.currentText()))
        reader = TelemetryReader(transport)
        try:
            reader.start()
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Bağlantı hatası", f"{transport.name} açılamadı:\n{e}")
            return
        self.state.set_reader(reader)
        self.state.log.emit(f"Bağlandı: {transport.name}")
        # cihaz bilgisini iste; yanıt (Ts) gelince K ve ayarlar otomatik yüklenir (AppState.on_device_info)
        self.state.send("ping")

    def disconnect(self):
        reader = self.state.reader
        if reader is None:
            return
        try:
            if reader.transport.is_open and reader.error is None:
                reader.send("stop")   # bağlantı keserken motoru durdur
        except Exception:
            pass
        reader.stop()
        self.state.set_reader(None)
        self.state.log.emit("Bağlantı kesildi.")

    def _on_conn(self, connected: bool):
        self.connect_btn.setText("Bağlantıyı Kes" if connected else "Bağlan")
        for w in (self.port, self.baud, self.refresh_btn):
            w.setEnabled(not connected)
