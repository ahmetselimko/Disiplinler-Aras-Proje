"""Ters Sarkaç Kontrol Arayüzü — çalıştırma:  python main.py  [--params config/params.json]"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--params", default=str(ROOT / "config" / "params.json"), help="fiziksel parametre dosyası")
    ap.add_argument("--screenshot", help="(test) pencereyi açıp verilen dosyaya ekran görüntüsü kaydeder ve kapanır")
    ap.add_argument("--demo", action="store_true", help="(test) sanal cihaza bağlan, kalibre et ve başlat")
    args = ap.parse_args(argv)

    import pyqtgraph as pg
    from PyQt5 import QtCore, QtWidgets

    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    app = QtWidgets.QApplication(sys.argv[:1])
    app.setStyle("Fusion")
    pg.setConfigOptions(antialias=False, background="w", foreground="k")

    from gui.main_window import MainWindow
    from gui.state import AppState

    state = AppState(args.params)
    win = MainWindow(state)
    win.show()

    if args.demo or args.screenshot:
        def demo():
            win.conn.port.setCurrentIndex(0)   # sanal ESP32
            win.conn.connect()
            state.send("home")
        QtCore.QTimer.singleShot(300, demo)
        # homing bittiğinde (HOMING → IDLE) başlat; sanal cihazda ~5–9 s sürer
        seen_homing = []

        def try_start():
            last = state.reader.last if state.reader else None
            if last and last.mode == "HOMING":
                seen_homing.append(True)
            if seen_homing and last and last.mode == "IDLE":
                state.send("start")
            else:
                QtCore.QTimer.singleShot(100, try_start)
        QtCore.QTimer.singleShot(1500, try_start)
    if args.screenshot:
        def shot():
            win.grab().save(args.screenshot)
            print(f"ekran görüntüsü: {args.screenshot}  veri hızı: {state.reader.rate_hz:.0f} Hz  "
                  f"FPS: {win._fps:.0f}  kayıp: {state.reader.dropped}  mod: {state.reader.last.mode}")
            win.close()
        QtCore.QTimer.singleShot(22000, shot)

    return app.exec_()


if __name__ == "__main__":
    sys.path.insert(0, str(ROOT))
    sys.exit(main())
