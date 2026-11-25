# Kalman Filtresi ile Titreşim Azaltma (#14) ve Kayıt Oynatma Sekmesi (#15)

> **Not (22.11.2026):** Bu belgedeki sayılar eski kasnakla (GT2 40 diş, r = 12.7 mm) üretildi. `config/params.json` artık BOM v3'teki HTD3M 20 dişi (r = 9.55 mm) kullanıyor; sonuçlar yeniden üretilirse değişebilir.

Bu belge #14 ve #15 görevleri kapsamında yapılan değişiklikleri, deneysel ölçümleri ve doğrulama yöntemlerini özetler.

---

## #14 — Kalman Filtresiyle Titreşimi Azaltmak

### Sorun
12-bit AS5600 açı sensörünün kuantalama adımı ($2\pi / 4096 \approx 0.088^\circ$) sayısal türev + 25 Hz 1. derece alçak geçiren filtre (`VelocityEstimator`) üzerinden LQR kazancıyla çarpıldığında dengede motor geriliminde $\pm 1\text{--}1.5\text{ V}$ mertebesinde titreşime yol açıyordu. Filtre kesim frekansını 15 Hz'in altına çekmek faz gecikmesini artırdığı için swing-up yakalamayı bozuyordu.

### Çözüm
1. **Ayrık Doğrusal Kalman Tasarımı ([`pendulum_lab/filters.py`](../pendulum_lab/filters.py)):**
   - Dik konum ($\theta = 0$) etrafında doğrusallaştırılmış ayrık model $(A_d, B_d)$ (`c2d(*linearize(p), dt)`) ve ölçüm matrisi $C = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 0 & 1 & 0 \end{bmatrix}$ için `design_kalman` eklendi.
   - Ölçüm gürültüsü $R_n$, enkoder ve 12-bit AS5600 kuantalama varyanslarından ($\Delta^2 / 12$) türetildi; süreç gürültüsü $Q_n = \operatorname{diag}(10^{-9}, 10^{-5}, 10^{-9}, 10^{-4})$ ile DARE çözülerek kararlı hal Kalman kazancı $L$ elde edildi.
   - `KalmanFilter.from_params`, `KalmanFilter.reset` ve `step` içinde $\pm\pi$ sınırında açı sarma (`_wrap_angle`) desteği eklendi.
2. **Mod Geçişi ve Çalışma Bölgesi ([`simulator.py`](../pendulum_lab/sim/simulator.py), [`mock_device.py`](../pendulum_lab/comm/mock_device.py)):**
   - $(A_d, B_d)$ dik konum etrafında tanımlı olduğu için geniş açı swing-up fazında ($|\theta| \ge \text{release\_deg}$) `VelocityEstimator` (25 Hz) korunur.
   - Sarkaç dik bölgeye ($|\theta| < \text{release\_deg}$) girdiğinde Kalman kestiricisi mevcut hız kestirimiyle başlatılır (`reset`) ve $\dot{x}, \dot{\theta}$ değerleri Kalman durumundan alınır.
   - `Analiz ve Simülasyon` sekmesine (`SimulationPage`) Kalman filtresi aç/kapa seçeneği eklendi.
3. **ESP32 Firmware Entegrasyonu ([`export.py`](../pendulum_lab/export.py), [`gains.h`](../firmware/esp32_pendulum/gains.h), [`esp32_pendulum.ino`](../firmware/esp32_pendulum/esp32_pendulum.ino)):**
   - `gains_header` artık `KALMAN_AD[4][4]`, `KALMAN_BD[4]` ve `KALMAN_L[4][2]` sabitlerini üretir.
   - `esp32_pendulum.ino` `ctrlTask` döngüsüne `KalmanEst` yapısı eklendi; `calib` ve `homing` geçişlerinde `kf.reset()` çağrılır.
   - Gerçek kartta model tutmazsa Kalman, derlemede `#define USE_KALMAN 0` ile kapatılıp eski 25 Hz türev filtresine dönülebilir.
   - `simulate_closed_loop` varsayılan olarak (`use_kalman=True`) firmware ile aynı kestiriciyi kullanır; eski davranış için `use_kalman=False`.

### Simülasyon Karşılaştırması ($\theta_0 = 180^\circ$, 15 s, $T_s = 2\text{ ms}$, `SensorModel(seed=0)`, 48 CPR enkoder; 18.11.2026'da yeniden ölçüldü)

| Yöntem | Dengeye Giriş | Dengede $\sigma(u)$ ($t \ge 10\text{ s}$) | Dengede $\max\|u\|$ | Tepeden Tepeye ($u_{\text{ptp}}$) |
|---|---|---|---|---|
| 25 Hz `VelocityEstimator` | 3.650 s | 0.6012 V | 1.6805 V | 3.3563 V |
| `KalmanFilter` (dik bölgede) | 3.644 s | **0.2024 V** (**-%66**) | **0.6252 V** (**-%63**) | **1.2457 V** (**-%63**) |

---

## #15 — Kayıt Oynatma Sekmesi

### Yapılanlar
1. **CSV Okuyucu ve Model Doğrulama ([`pendulum_lab/recorder.py`](../pendulum_lab/recorder.py)):**
   - `load_recording`: `Recorder` tarafından üretilen `# meta`, `# event` ve `t,x,dx,th,dth,u,mode` veri satırlarını ayrıştırıp `RecordedSession` döndürür.
   - `compare_recording_with_simulation`: Kaydın aktif deneme aralığını (`WAIT`/`SWING_UP`/`BALANCE`) hizalar, başlangıç durumunu $[x_0, \dot{x}_0, \theta_0, \dot{\theta}_0]$ ve BAŞLAT anında geçerli ayarları (başlıktaki `params`, `K`, `Ts`, `swingup`, `device_acknowledged_settings` + o ana kadar kayda düşen `command_sent`/`Ack`/`pc_design_snapshot` olayları) kullanarak `simulate_closed_loop` çalıştırır; enterpole edilmiş `RMSE(θ)`, `RMSE(x)`, `RMSE(u)`, `max|Δθ|`, `max|Δx|` metriklerini ve her iki özet istatistiği (`TrialSummary`) hesaplar.
2. **Kayıt Oynatma Sekmesi ([`gui/playback_tab.py`](../gui/playback_tab.py), [`gui/main_window.py`](../gui/main_window.py)):**
   - Ana pencereye `"Kayıt Oynatma"` sekmesi eklendi.
   - `kayitlar/` klasöründeki kayıtları listeleme veya dosya seçiciden CSV yükleme,
   - $\theta$, $x$, $u$, $E$ ve mod grafiklerinde Kayıt (düz çizgi) ve Simülasyon (kesikli çizgi) üst üste çizimi,
   - `PendulumView` üzerinde kayıt veya simülasyon kaynağından hız ayarlı (`0.25x`–`2x`) animasyon oynatma ve dikey zaman imleci.

### Doğrulama
- Yerel bir sanal ESP32 kaydı (`kayitlar/oturum_20261101_233904.csv`; `kayitlar/` git'e eklenmez) ile karşılaştırma: `RMSE(θ) = 0.99°`, `RMSE(x) = 0.0120 m` (cihazın onayladığı K kullanılarak; yalnız başlıktaki PC tasarımıyla 1.10° / 0.0185 m çıkıyordu).
- Testler depodaki bir kayda bağlı değildir: sanal ESP32 (`MockDevice`) ile homing → swing-up → denge kaydını geçici klasöre üretip hem `compare_recording_with_simulation`'ı hem de `Kayıt Oynatma` sekmesini bu kayıtla dener.
- Model uyumsuzluğu denemesi (gerçek sistem m ±%30, M +%50, b ×3, Kt −%20, 1–3 adım gecikme; Kalman nominal parametrelerle): dengeye girilen her durumda Kalman σ(u)'yu ~0.6 V'tan ~0.2–0.26 V'a indiriyor.
- Otomatik testler: `tests/test_kalman_and_playback.py` içindeki tüm birim ve GUI testleri ile tüm proje test paketi çalıştırılıp doğrulanmıştır.

### İnceleme sonrası düzeltmeler
- **Başlangıç hızı:** gerçekçi sensörde hız kestiricileri (ve Kalman) ilk adımda başlangıç hızını sıfırlıyordu; `VelocityEstimator.prime` ile simülasyon kayıttaki $\dot{x}_0, \dot{\theta}_0$ ile başlıyor.
- **Kayıt sırasında onaylanan ayarlar:** karşılaştırma yalnız CSV başlığını değil, BAŞLAT anına kadarki `command_sent` + `Ack` olaylarını (ve `pc_design_snapshot`) da işliyor; deneme sırasında değişen ayarlar sekmede uyarı olarak gösteriliyor.
- **Tam kayıt modu:** simülasyon kaydın başında değil, kayıttaki BAŞLAT anında, o anki durumla başlıyor ve zaman ekseninde bu ana kaydırılıyor. Metrikler her iki modda da yalnız aktif denemede hesaplanıyor (önceden tam modda RMSE(θ) 127°'ye çıkıyordu).
- **Oynatma hızı:** kare, geçen gerçek süre × hız ile CSV zaman damgalarından bulunuyor (örnek aralıkları 4/6 ms düzensiz olduğundan sabit satır adımı 0.25x ile 0.5x'i aynı hızda oynatıyordu).
- **Firmware:** `gains.h` `KALMAN_AVAILABLE` tanımlıyor; Kalman matrisleri olmayan bir `gains.h` ile Kalman kodu derleme dışı kalıyor ve derleme uyarısı veriliyor. (#50'den beri #50 öncesi dışa aktarılmış her `gains.h` `THETA_POSITIVE_LEFT` eksik olduğu için `#error` ile durur; bu yedek yalnız elle yazılmış bir `gains.h` için geçerli.)
- Her bulgu için düzeltme öncesi kodda düşen bir regresyon testi eklendi (`tests/test_kalman_and_playback.py`).
