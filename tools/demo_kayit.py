"""Arayüzün tanıtım videosunu, GIF'ini ve ekran görüntülerini otomatik üretir.

    python tools/demo_kayit.py

Sanal ESP32'ye bağlanır, adım adım kullanımı oynatır (bağlan → kalibre et → başlat → dürt →
tasarım → simülasyon → acil stop), her kareye açıklama yazısı ve basılan butona çerçeve ekler.
Çıktılar docs/media/ altına yazılır:
    demo.mp4 (veya demo.webm)  — tam tur, 10 kare/s
    demo.gif                    — README için kısa canlı izleme bölümü
    *.png                       — sekme ekran görüntüleri
Arayüz değiştiğinde bu betik yeniden çalıştırılarak medya güncellenir.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import pyqtgraph as pg  # noqa: E402
from PIL import Image  # noqa: E402
from PyQt5 import QtCore, QtGui, QtWidgets  # noqa: E402

OUT = ROOT / "docs" / "media"
FPS = 10                 # video kare hızı (zamanlayıcı 100 ms)
GIF_EVERY = 2            # GIF'e her 2. kare (5 kare/s)
GIF_WIDTH = 820
WIN_SIZE = (1440, 880)


class Recorder:
    def __init__(self, win: QtWidgets.QWidget):
        self.win = win
        self.writer = None
        self.video_path = None
        self.gif_frames: list[Image.Image] = []
        self.gif_on = False
        self.n = 0
        self.font = QtGui.QFont("Segoe UI", 15)
        self.font.setBold(True)

    def _open(self, w: int, h: int):
        OUT.mkdir(parents=True, exist_ok=True)
        # tarayıcıda oynayan biçimler öncelikli: H.264 MP4 → VP8 WebM → MPEG-4 MP4
        for fourcc, name in (("avc1", "demo.mp4"), ("VP80", "demo.webm"), ("mp4v", "demo.mp4")):
            path = OUT / name
            wr = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*fourcc), FPS, (w, h))
            if wr.isOpened():
                self.writer, self.video_path, self.codec = wr, path, fourcc
                return
        raise RuntimeError("Hiçbir video kodlayıcı açılamadı")

    def _qimage_to_rgb(self, img: QtGui.QImage) -> np.ndarray:
        img = img.convertToFormat(QtGui.QImage.Format_RGB888)
        w, h, bpl = img.width(), img.height(), img.bytesPerLine()
        ptr = img.bits()
        ptr.setsize(h * bpl)
        return np.frombuffer(ptr, np.uint8).reshape(h, bpl)[:, :w * 3].reshape(h, w, 3).copy()

    def _overlay(self, img: QtGui.QImage, caption: str, hl):
        dpr = img.devicePixelRatio()
        p = QtGui.QPainter(img)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        if hl is not None and hl.isVisible():
            pos = hl.mapTo(self.win, QtCore.QPoint(0, 0))
            r = QtCore.QRectF(pos.x() - 5, pos.y() - 5, hl.width() + 10, hl.height() + 10)
            p.setPen(QtGui.QPen(QtGui.QColor("#f39c12"), 5))
            p.setBrush(QtCore.Qt.NoBrush)
            p.drawRoundedRect(r, 8, 8)
        if caption:
            w, h = img.width() / dpr, img.height() / dpr
            band = QtCore.QRectF(0, h - 60, w, 60)
            p.fillRect(band, QtGui.QColor(25, 25, 25, 215))
            p.setPen(QtGui.QColor("white"))
            p.setFont(self.font)
            p.drawText(band.adjusted(24, 0, -24, 0), QtCore.Qt.AlignVCenter | QtCore.Qt.AlignLeft, caption)
        p.end()

    def frame(self, caption: str = "", hl=None):
        img = self.win.grab().toImage()
        self._overlay(img, caption, hl)
        rgb = self._qimage_to_rgb(img)
        h, w = rgb.shape[:2]
        h, w = h - h % 2, w - w % 2                     # kodlayıcılar çift boyut ister
        rgb = rgb[:h, :w]
        if self.writer is None:
            self._open(w, h)
        self.writer.write(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        if self.gif_on and self.n % GIF_EVERY == 0:
            im = Image.fromarray(rgb)
            im = im.resize((GIF_WIDTH, round(h * GIF_WIDTH / w)), Image.LANCZOS)
            self.gif_frames.append(im.quantize(colors=128, method=Image.Quantize.FASTOCTREE))
        self.n += 1

    def still(self, name: str):
        OUT.mkdir(parents=True, exist_ok=True)
        self.win.grab().save(str(OUT / name))

    def close(self):
        if self.writer is not None:
            self.writer.release()
        if self.gif_frames:
            self.gif_frames[0].save(OUT / "demo.gif", save_all=True, append_images=self.gif_frames[1:],
                                    duration=1000 * GIF_EVERY // FPS, loop=0, optimize=True)


def scenario(win, st, rec: Recorder):
    """Her yield bir video karesi (100 ms). Adımlar arayüzün gerçek durumuna göre ilerler."""
    live = win.live

    def hold(sec, caption, hl=None):
        for _ in range(round(sec * FPS)):
            yield caption, hl

    def until(cond, timeout, caption, hl=None):
        t0 = time.monotonic()
        while not cond() and time.monotonic() - t0 < timeout:
            yield caption, hl

    def mode():
        return st.reader.last.mode if st.reader and st.reader.last else ""

    rec.gif_on = True
    yield from hold(2.0, "Ters Sarkaç Kontrol Arayüzü — donanım yokken sanal ESP32 ile çalışır")
    yield from hold(2.0, "1) Port: ★ Sanal ESP32 (gerçek kartta ESP32'nin COM portu) → Bağlan", win.conn.port)
    yield from hold(1.0, "1) Port: ★ Sanal ESP32 (gerçek kartta ESP32'nin COM portu) → Bağlan", win.conn.connect_btn)
    win.conn.connect()
    yield from hold(2.5, "Cihaz kontrol periyodunu (Ts) bildirir; eşleşirse K ve ayarlar otomatik yüklenir", live.log)
    yield from hold(1.0, "2) KALİBRE ET: araba sol limit switch'e gider, sıfırlanır, ortaya döner", live.b_home)
    live.b_home.click()
    yield from until(lambda: mode() == "HOMING", 3, "2) KALİBRE ET: araba sol limit switch'e gider, sıfırlanır, ortaya döner", live.b_home)
    yield from until(lambda: mode() == "IDLE", 20, "Kalibrasyon sürüyor… (gerçek kartta homing yapılmadan BAŞLAT reddedilir)", live.view)
    yield from hold(1.0, "3) BAŞLAT: enerji bazlı swing-up sarkacı aşağıdan kaldırır", live.b_start)
    live.b_start.click()
    yield from until(lambda: mode() == "BALANCE", 15, "Swing-up: araba ileri geri giderek sarkaca enerji pompalar", live.view)
    yield from hold(4.0, "4) |θ| < 15° olunca LQR devreye girer ve sarkacı dik tutar", live.mode_lbl)
    rec.still("canli_izleme.png")
    yield from hold(1.0, "5) Sarkacı Dürt (yalnız simülasyonda): LQR bozulan dengeyi toparlar", live.b_kick)
    live.b_kick.click()
    yield from hold(4.0, "5) Sarkacı Dürt (yalnız simülasyonda): LQR bozulan dengeyi toparlar", live.glw)
    rec.gif_on = False

    win.tabs.setCurrentIndex(1)
    rec.still("tasarim.png")
    yield from hold(3.5, "6) Model ve LQR Tasarımı: parametreler + Q/R → K, kutup haritası, gains.h dışa aktarma", win.design.b_calc)

    win.tabs.setCurrentIndex(2)
    sim = win.analysis.widget(0)
    win.analysis.setCurrentIndex(0)
    yield from hold(1.0, "7) Analiz: aşağıdan başlayan lineer olmayan simülasyon (12-bit sensör, gecikme, gürültü)", sim.run_btn)
    sim.run()
    rec.still("simulasyon.png")
    sim.toggle_play()
    yield from until(lambda: not sim.timer.isActive(), 8, "Simülasyon sonucu animasyonla oynatılır; θ, x, u ve mod grafikleri", sim.view)
    if sim.timer.isActive():
        sim.toggle_play()
    win.analysis.setCurrentIndex(1)
    rec.still("basamak_yaniti.png")
    yield from hold(2.5, "Basamak yanıtı: araba hedefe giderken önce ters yöne kayar (minimum faz dışı)")
    win.analysis.setCurrentIndex(2)
    rec.still("kok_egrisi.png")
    yield from hold(2.5, "Kök eğrisi: yalnız θ geri beslemesi arabayı kontrol edemez → LQR seçildi")

    win.tabs.setCurrentIndex(0)
    yield from hold(1.0, "8) ACİL STOP (veya her pencerede Esc): motor hemen durur", live.b_estop)
    live.estop()
    yield from hold(2.5, "8) ACİL STOP (veya her pencerede Esc): motor hemen durur", live.mode_lbl)
    yield from hold(1.5, "Ayrıntılar: README.md ve docs/kullanim_kilavuzu.md")


def main() -> int:
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    app = QtWidgets.QApplication(sys.argv[:1])
    app.setStyle("Fusion")
    pg.setConfigOptions(antialias=False, background="w", foreground="k")
    from gui.main_window import MainWindow
    from gui.state import AppState

    st = AppState(ROOT / "config" / "params.json")
    win = MainWindow(st)
    win.resize(*WIN_SIZE)
    win.show()
    rec = Recorder(win)
    steps = scenario(win, st, rec)

    def tick():
        try:
            caption, hl = next(steps)
        except StopIteration:
            timer.stop()
            win.conn.disconnect()
            rec.close()
            print(f"video: {rec.video_path} ({rec.codec}, {rec.n} kare)  gif: {len(rec.gif_frames)} kare")
            win.close()
            return
        rec.frame(caption, hl)

    timer = QtCore.QTimer()
    timer.setTimerType(QtCore.Qt.PreciseTimer)
    timer.setInterval(1000 // FPS)
    timer.timeout.connect(tick)
    QtCore.QTimer.singleShot(800, timer.start)      # pencere tam çizilsin
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
