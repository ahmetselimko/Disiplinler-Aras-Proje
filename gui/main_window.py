"""Ana pencere: bağlantı çubuğu + sekmeler + durum çubuğu + 60 FPS yenileme zamanlayıcısı."""

from __future__ import annotations

import queue
import time

from PyQt5 import QtCore, QtWidgets

from pendulum_lab.comm import protocol as P

from .analysis_tab import AnalysisTab
from .connection_panel import ConnectionPanel
from .design_tab import DesignTab
from .live_tab import LiveTab


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, state):
        super().__init__()
        self.state = state
        self.setWindowTitle("Ters Sarkaç Kontrol Arayüzü")
        self.resize(1450, 880)

        central = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(central)
        lay.setContentsMargins(4, 4, 4, 4)
        self.conn = ConnectionPanel(state)
        lay.addWidget(self.conn)
        self.tabs = QtWidgets.QTabWidget()
        self.live = LiveTab(state)
        self.design = DesignTab(state)
        self.analysis = AnalysisTab(state)
        self.tabs.addTab(self.live, "Canlı İzleme")
        self.tabs.addTab(self.design, "Model ve LQR Tasarımı")
        self.tabs.addTab(self.analysis, "Analiz ve Simülasyon")
        lay.addWidget(self.tabs)
        self.setCentralWidget(central)

        sb = self.statusBar()
        self.s_conn = QtWidgets.QLabel("Bağlı değil")
        self.s_rate = QtWidgets.QLabel("")
        self.s_fps = QtWidgets.QLabel("")
        self.s_K = QtWidgets.QLabel("")
        for w in (self.s_conn, self.s_rate, self.s_fps):
            sb.addWidget(w)
        sb.addPermanentWidget(self.s_K)
        state.gains_changed.connect(self._on_gains)
        state.connection_changed.connect(self._on_conn)
        self._on_gains()

        # Esc her yerde acil stop — uygulama düzeyinde olay filtresi, açık bir diyalog varken de çalışır
        self._esc = EscFilter(self._esc_pressed)
        QtWidgets.QApplication.instance().installEventFilter(self._esc)

        if state.startup_warning:
            state.log.emit("UYARI: " + state.startup_warning)

        self._frames, self._fps_t0, self._fps = 0, time.perf_counter(), 0.0
        self.timer = QtCore.QTimer(self)
        self.timer.setTimerType(QtCore.Qt.PreciseTimer)
        self.timer.setInterval(16)          # ~60 FPS; veri okuma bundan bağımsız, ayrı thread'de
        self.timer.timeout.connect(self._tick)
        self.timer.start()

    def _tick(self):
        reader = self.state.reader
        if reader is not None:
            if reader.error is not None:
                err = reader.error
                self.conn.disconnect()
                QtWidgets.QMessageBox.critical(self, "Bağlantı koptu", str(err))
                return
            for _ in range(200):
                try:
                    msg = reader.messages.get_nowait()
                except queue.Empty:
                    break
                self.live.handle_message(msg)
                if (isinstance(msg, P.Ack) and msg.cmd == "ping" and msg.ok) or \
                        (isinstance(msg, P.Event) and msg.name == "boot"):
                    self.state.on_device_info(msg.data)
            if self.tabs.currentWidget() is self.live:
                self.live.refresh(reader)
                self._frames += 1
            self.s_rate.setText(f"  Veri: {reader.rate_hz:6.0f} Hz   kayıp: {reader.dropped}   "
                                f"hatalı satır: {reader.parse_errors}")
        now = time.perf_counter()
        if now - self._fps_t0 >= 1.0:
            self._fps = self._frames / (now - self._fps_t0)
            self._frames, self._fps_t0 = 0, now
            self.s_fps.setText(f"  Çizim: {self._fps:4.0f} FPS" if reader is not None else "")

    def _esc_pressed(self):
        if self.state.connected:
            self.live.estop()

    def _on_gains(self):
        self.s_K.setText("K = [" + ", ".join(f"{k:.2f}" for k in self.state.K) + "]  ")

    def _on_conn(self, connected: bool):
        r = self.state.reader
        self.s_conn.setText(f"Bağlı: {r.transport.name}" if connected and r else "Bağlı değil")
        if not connected:
            self.s_rate.setText("")
            self.s_fps.setText("")

    def closeEvent(self, ev):
        self.conn.disconnect()
        super().closeEvent(ev)


class EscFilter(QtCore.QObject):
    """Esc tuşunu hangi pencere/diyalog odakta olursa olsun yakalar (olayı tüketmez)."""

    def __init__(self, callback):
        super().__init__()
        self.callback = callback
        self._last = 0.0

    def eventFilter(self, obj, ev):
        if ev.type() == QtCore.QEvent.KeyPress and ev.key() == QtCore.Qt.Key_Escape and not ev.isAutoRepeat():
            now = time.monotonic()
            if now - self._last > 0.3:      # olay ebeveynlere yayılırken birden çok kez görülür
                self._last = now
                self.callback()
        return False
