# Yazılım Mimarisi

## En önemli karar: denge döngüsü ESP32'de çalışır

```
┌──────────────── ESP32 (500 Hz, gerçek zamanlı) ─────────────────┐        ┌────────────── PC (Python) ───────────────┐
│ AS5600 (I2C) ─┐                                                  │        │                                           │
│ Enkoder (ISR) ┼─▶ durum kestirimi ─▶ Swing-up / LQR ─▶ BTS7960   │ ─JSON─▶│ okuyucu thread ─▶ halka tampon ─▶ grafik  │
│ Limit switch ─┘     (türev+filtre)    (mod makinesi)    PWM      │  200Hz │                         (60 FPS timer)    │
│                                                                  │◀─JSON──│ K tasarımı (LQR), komutlar, kayıt, sim.   │
└──────────────────────────────────────────────────────────────────┘ komut  └───────────────────────────────────────────┘
```

**Neden?** Ters sarkaçta dengeyi kaybetmemek için kontrol kararının her 2 ms'de *gecikmesiz ve düzenli* verilmesi gerekir. Döngü PC'de çalışsaydı:
* USB-seri gidiş-dönüşü 2–10 ms gecikme ekler,
* Windows gerçek zamanlı bir işletim sistemi değildir; arada 10–20 ms'lik takılmalar olur,
* bu gecikmeler, hangi dil kullanılırsa kullanılsın (C++ dahil) sarkacı düşürür.

Bu yüzden PC'nin görevi **zaman açısından kritik olmayan** işlerdir: K'yı hesaplamak, göndermek, izlemek, kaydetmek, simüle etmek. Bu işler için Python yeterince hızlıdır. Ölçülen değerler: sanal cihazdan 500 Hz kayıpsız okuma ve 57–60 FPS çizim.

## Katmanlar

| Katman | Klasör | Bağımlılık |
|---|---|---|
| Model (denklemler, doğrusallaştırma, motor) | `pendulum_lab/model` | NumPy |
| Kontrol (LQR, swing-up, mod makinesi, PID) | `pendulum_lab/control` | SciPy |
| Analiz (ss, tf, step, rlocus) | `pendulum_lab/analysis` | SciPy |
| Simülasyon (ode45 karşılığı + ESP32 taklidi) | `pendulum_lab/sim` | SciPy |
| Haberleşme (protokol, transport, okuyucu, sanal cihaz) | `pendulum_lab/comm` | pySerial |
| Arayüz | `gui/` | PyQt5, PyQtGraph |

Kütüphane (`pendulum_lab`) arayüzden bağımsızdır. Arayüz olmadan, örneğin bir Jupyter defterinde de kullanılabilir.

## Thread yapısı (PC)

| Thread | İş | Not |
|---|---|---|
| Ana (Qt) | Çizim (16 ms QTimer), butonlar | Seri portu **hiç** beklemez |
| `TelemetryReader` | `transport.read()`, satır bölme, JSON çözme, tampona toplu yazma | PDF'teki "ayrı thread" şartı |
| `MockDevice` (yalnız simülasyonda) | 500 Hz fizik + firmware mantığı | Windows'taki uyku sapması telafi edilir |

Paylaşılan veri `RingBuffer`'dır: kilitli, önceden ayrılmış numpy dizisi. GUI her karede yalnızca son N satırın kopyasını alır. Komut yazma (`transport.write`) kilitlidir, bu yüzden GUI'den güvenle çağrılır.

## Sanal ESP32 neden var?
Donanım (motor, sensör, kart) henüz seçilmedi. `MockDevice`, firmware'in yapacağı her şeyi aynı protokolle yapar: homing, limit switch'ler, swing-up, LQR, sensör kuantalama. Böylece arayüz ve protokol, kart gelmeden uçtan uca geliştirilip test edilebilir. Gerçek karta geçişte arayüzde yalnızca port seçimi değişir.

## Gerçek parametreler gelince
1. `config/params.json` dosyasını doldurun (bkz. `parametre_tablosu.md`) veya arayüzün Tasarım sekmesinden girip kaydedin.
2. Tasarım sekmesinde **K Hesapla**: açık çevrim kutupları, K ve kapalı çevrim kutupları görünür.
3. Analiz sekmesinde θ₀ = 180° ile simülasyon yapın: swing-up ve yakalama çalışmalı, max |x| ray sınırının altında kalmalı. Gerekirse swing-up kazançlarını ayarlayın.
4. **gains.h Dışa Aktar** ile firmware'e gömün, ya da bağlıyken **Cihaza Gönder**. Bağlanınca arayüz önce `ping` gönderir; cihazın Ts'si tasarımdakiyle aynıysa K ve tüm ayarlar (swing-up açık/kapalı, x_ref dahil) otomatik yüklenir. Cihaz resetlenirse (`boot` olayı) yeniden yüklenir.
