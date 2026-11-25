# ESP32 Referans Firmware

**Durum:** Bilgisayar ekibi tarafından protokolü ve kontrol mantığını tanımlamak için yazılmış bir iskelettir. **Hatasız derleniyor, ama kartta henüz denenmedi.** Mekatronik ekibi kendi donanımına göre uyarlamalıdır.

## Gereken
* Arduino IDE 2.x + "esp32 by Espressif" kart paketi (core 2.x ve 3.x desteklenir)
* Kütüphane: **ArduinoJson** (v7)
* Kart: **ESP32-S3-DevKitC-1 N16R8** (BOM v3) ya da klasik ESP32-WROOM-32. İkisi de çift çekirdek; pinler kartın türüne göre `board_pins.h`'den otomatik seçilir

## Derleme
Hatasız ve uyarısız (`--warnings all`) derlendiği doğrulanan sürümler:

| Araç | Sürüm |
|---|---|
| arduino-cli | 1.5.1 |
| esp32 by Espressif | **3.3.12** ve **2.0.17** |
| ArduinoJson | 7.4.3 |
| Kart seçimi | ESP32S3 Dev Module (`esp32:esp32:esp32s3`) ya da ESP32 Dev Module (`esp32:esp32:esp32`) |

Komut satırından:
```bash
arduino-cli core install esp32:esp32 --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli lib install ArduinoJson
arduino-cli compile --fqbn esp32:esp32:esp32s3 firmware/esp32_pendulum   # ESP32-S3
arduino-cli compile --fqbn esp32:esp32:esp32 firmware/esp32_pendulum     # klasik ESP32
```
Aynı derleme her push ve PR'da GitHub Actions'ta iki kart × iki core sürümüyle tekrarlanır (`.github/workflows/test.yml`).

**θ işaret kuralı (#50, firmware `ref-1.2`):** θ > 0 sarkaç **sola** yatık (mekatronik raporu 8.1.3). Firmware boot olayında ve ping yanıtında `"theta":"sol"` bildirir; arayüz bunu bildirmeyen (eski) karta K göndermez. #50'den önce dışa aktarılmış bir `gains.h` (`THETA_POSITIVE_LEFT` makrosu yok) **derlenmez** (`#error`): onda K'nın θ, θ̇ kazançları ve Kalman matrisleri ters işaretlidir. Arayüzden yeniden dışa aktarın.

**ESP32-S3-DevKitC-1'de iki USB portu var:** bilgisayara **"COM" (UART)** yazan port bağlanmalı; `Serial` (921600 baud) oradan çıkar. Arduino IDE'de "USB CDC On Boot" **Disabled** (varsayılan) kalmalı.

## Dosyalar
| Dosya | İçerik |
|---|---|
| `esp32_pendulum.ino` | 500 Hz kontrol görevi (çekirdek 1) + iletişim görevi (çekirdek 0) |
| `gains.h` | LQR kazancı, eşikler ve LQR'a geçiş şartları (`CATCH_*`), swing-up kazançları ve hedef enerji oranı (`SU_*`), model sabitleri, redüktör oranı ve kasnak yarıçapı, Kalman matrisleri (`KALMAN_AD/BD/L`). PC arayüzünden üretilir (**gains.h Dışa Aktar**). Eski bir `gains.h`'de olmayan değerler için `.ino`'daki `#ifndef` varsayılanları kullanılır; Kalman matrisleri yoksa Kalman uyarıyla kapalı derlenir |

Araba konumu enkoder sayımından `x = sayım · 2π·PULLEY_R_M / (ENC_CPR_MOTOR · MOTOR_GEAR)` ile hesaplanır. Redüktör (`MOTOR_GEAR`) ve kasnak (`PULLEY_R_M`) `gains.h`'den gelir: motor veya kasnak değişince arayüzde girip yeniden dışa aktarmak yeter. Motor milindeki enkoder çözünürlüğü (`ENC_CPR_MOTOR` = **44**: JGB37-520, mekatronik raporunda 836 sayım / çıkış turu ÷ 19; motorda doğrulanacak) ise motora özgüdür. Yanlış CPR/redüktör x ölçeğini doğrudan bozar: ör. eski 48 CPR × 10:1 (480) varsayımı 836 sayımlık motorda x'i ~%43 küçük okurdu; motor değişirse `.ino`'da elle düzenlenir.

## Pinler (`board_pins.h` — kart türüne göre otomatik)
| İşlev | ESP32-S3-DevKitC-1 N16R8 | Klasik ESP32 |
|---|---|---|
| I2C SDA / SCL (AS5600, ileride MPU6050) | 8 / 9 (400 kHz) | 21 / 22 |
| Enkoder A / B (harici 10k pull-up, **3.3 V'a**) | 4 / 5 | 32 / 33 |
| BTS7960 RPWM / LPWM / EN | 6 / 7 / 15 | 25 / 26 / 27 |
| Uç sensörü sol / sağ (INPUT_PULLUP) | 16 / 17 | 18 / 19 |

**Enkoder gerilimi:** ESP32 / ESP32-S3 pinleri 5 V'a dayanıklı değildir. Motor enkoderi 5 V ile besleniyorsa çıkışları 5 V'a çekilmemeli: pull-up'lar 3.3 V'a bağlanmalı (açık kollektör çıkışta) ya da seviye dönüştürücü / gerilim bölücü kullanılmalı. Hall enkoder 3.3 V ile çalışabiliyorsa en basiti 3.3 V'tan beslemektir (datasheet'e bakın).

**S3 N16R8'de neden eski pinler olmuyor:** 22–25 S3'te yok; 26–32 SPI flash'a, 33–37 8 MB oktal PSRAM'e bağlı. Ayrıca 0/3/45/46 (açılış), 19/20 (yerel USB), 43/44 (`Serial`), 38/48 (RGB LED) kullanılmadı. `board_pins.h` S3 pinlerini derleme anında denetler; yasak bir pin seçilirse derleme durur.

**Uç sensörü** (`LIMIT_SWITCHES 1`, varsayılan): BOM'da iki seçenek var, ikisi de aynı pinlere ve aynı ayarla bağlanır:
* **Limit switch (DC163-3):** **NC (normalde kapalı) bağlantı önerilir**: basılı veya kablo kopuk → HIGH → güvenli taraf. NO kullanılırsa `LIM_PRESSED = LOW` yapın; o zaman kopuk kablo fark edilmez.
* **Yarık optik sensör (TCST2103):** IR LED'e akım sınırlama direnci (3.3 V'ta ~100 Ω, 5 V'ta ~220 Ω → ~15–20 mA), fototransistör çıkışına 3.3 V'a **harici 4.7–10 kΩ pull-up**. Arabadaki bayrak yarığa girince çıkış HIGH olur; kablo koparsa da HIGH → `LIM_PRESSED = HIGH` değişmez. Ortam ışığından etkilenmemesi için yarık kapalı tarafa bakmalı.

### Limit switch'siz çalışma (`LIMIT_SWITCHES 0`, isteğe bağlı)
Donanımda switch var (varsayılan `LIMIT_SWITCHES 1`). Switch'ler henüz takılmamışken kartı denemek gerekirse `0` yapılır; yoksa NC ayarındaki boş pinler "basılı" okunur ve kart hemen `limit` hatasına düşer. Bu modda:
* **KALİBRE ET = elle ortalama:** araba elle ray ortasına konur, durağanken `home` gönderilir; enkoderin o anki okuması x = 0 olur. Motor hareket etmez. Araba hareket ediyorsa (> 0.01 m/s) reddedilir.
* Ray güvenliğini **yalnız enkoder** sağlar: |x| > rail_half − 3 cm → FAULT `rail`. Araba ortaya yanlış konursa sınırlar o kadar kayar; enkoder sayım kaçırırsa araba uca çarpabilir. **Ray uçlarına mekanik durdurucu/tampon konmalıdır.**
* Switch'ler takılınca `.ino`'nun başındaki ayar yeniden `1` yapılır; otomatik homing ve switch korumaları geri gelir.

**Donanımsal acil stop** (motor beslemesini kesen mantar buton) yazılımdan bağımsız olarak eklenmelidir; yazılımdaki `estop` ve limit switch'ler bunun yerine geçmez (switch'ler yalnız kart çalışırken motoru durdurur). Mekatronik şimdilik buton koymayı planlamıyor (16.11.2026); en azından motor beslemesi elle kolayca kesilebilmeli.

## Güvenlik davranışları (inceleme sonrası eklendi)
| Durum | Davranış |
|---|---|
| Hareketli modda (HOMING/SWING_UP/WAIT/BALANCE/FRICTION) 1 s boyunca PC'den satır gelmedi (USB koptu, arayüz çöktü) | FAULT `link`, motor kesilir. Arayüz bağlıyken 0.25 s'de bir `{"cmd":"hb"}` gönderir (`LINK_TIMEOUT_MS`, protokol.md §4) |
| Uç sensöründe tek bir parazitli okuma | Dikkate alınmaz: art arda `LIM_DEBOUNCE` = 3 okuma (6 ms) basılıysa basılı sayılır |
| Yeni `home` başladı | Önceki kalibrasyon geçersiz; homing yarıda `stop` edilirse `start` reddedilir |
| Homing'de sağ switch'e basıldı | FAULT `homing_wrong_side` (motor veya enkoder yönü ters; ya da sağ sensör kablosu kopuk/sürekli basılı) |
| Homing'de sola dayandıktan 5 cm sonra sol sensör hâlâ basılı | FAULT `homing_switch` (sol sensör kablosu kopuk ya da takılı; sıfır yanlış olurdu) |
| Homing'de \|ẋ\| > 0.30 m/s ya da gerilim 0.3 s'den uzun doyumda | FAULT `homing_runaway` (hız döngüsü kontrolden çıktı; ek güvenlik) |
| Homing 20 s'yi aştı / rayın 2 katından fazla yol gidildi | FAULT `homing_timeout` / `homing_travel` |
| Homing başlarken sol switch zaten basılı | Hemen sıfırlanır (switch seviyesine bakılır) |
| Yalnız `LIMIT_SWITCHES 0`: `home` gelirken araba hareket ediyor | `home` reddedilir; araba durunca tekrar |
| AS5600 art arda 3 kez okunamadı | FAULT `sensor`; I2C zaman aşımı 2 ms (döngü takılmaz) |
| Açılış kalibrasyonu: mıknatıs yok/zayıf ya da sarkaç hareket ediyor | `calib ok:false`, `start` reddedilir; sarkaç durunca `calib` komutu |
| Aynı anda estop ve start/home | estop kazanır |
| Araba ray ucuna 0.10 m'den yakın veya 0.2 m/s'den hızlı | `start` sebebi yazılarak reddedilir |
| IDLE / FAULT | Sürücü EN = LOW (motor serbest) |
| `gains.h` eski veya Ts kesirli ms / giriş türü voltage değil | Derleme hatası (`#error`) |

## Python ile birebir eşleşen parçalar
| Firmware | Python |
|---|---|
| `ctrlTask` mod makinesi | `pendulum_lab/control/supervisor.py` |
| `swingUpAccel` | `pendulum_lab/control/swingup.py` |
| `FrictionTest`, `friction` komutu (sürtünme ölçümü) | `pendulum_lab/control/friction.py`, `mock_device.py` (test: `test_firmware_friction_matches_python`) |
| `accelToVolt` (Coulomb telafisi dahil) | `pendulum_lab/model/dynamics.py::accel_to_input`, `coulomb_compensation` (test: `test_firmware_coulomb_compensation_matches_python`) |
| `VelEst` (25 Hz) | `pendulum_lab/filters.py::VelocityEstimator` |
| `KalmanEst` (dik bölgede, `USE_KALMAN 1`; `0` ile kapatılır) | `pendulum_lab/filters.py::KalmanFilter`, `design_kalman` (test: `test_firmware_kalman_matches_python`) |
| `ENC_CPR_MOTOR` (44), `X_PER_COUNT` | `pendulum_lab/filters.py::ENC_CPR_MOTOR`, `encoder_resolution` (test: `test_simulated_encoder_resolution_matches_firmware`) |
| telemetri `snprintf` biçimi | `pendulum_lab/comm/protocol.py::encode_telemetry` |
| `PARAM_LIMITS`, komut kabul/ret kuralları | `protocol.py::SET_PARAM_LIMITS`, `mock_device.py::_handle` (test: `test_firmware_param_limits_match_python`) |

Birinde yapılan değişiklik diğerine de yansıtılmalıdır. Python tarafı testlerle doğrulanır (`tests/test_control.py`).

## Coulomb sürtünme telafisi (#12, firmware'e eklendi — derleyen/düzenleyen için not)
"Modele Coulomb sürtünmesi eklemek" görevinde (#12) firmware'e bilerek eklenen parçalar. Derleme hatalarını düzeltirken (#8) **kaldırılmamalı veya değiştirilmemelidir**; Python'daki karşılığıyla birebir aynı olmalı:

| Firmware | Ne yapar | Python karşılığı |
|---|---|---|
| `#ifndef P_FC … #define P_FC 0.0f` (derleme anı denetimlerinde) | Eski `gains.h`'de `P_FC` yoksa telafiyi kapatır, derleme bozulmaz | — |
| `FC_V_COMP = 0.01f` | Bu hızın [m/s] altında araba duruyor sayılır | `dynamics.V_COMP` |
| `accelToVolt` içindeki `F += / -= fc_N` | Kuvvete motorun kuru sürtünmesi kadar ek: araba hareket ediyorsa hareket yönünde, duruyorsa istenen ivme yönünde. `fc_N` başta `P_FC`'dir; `set_param fc_volt` gelince `fc_volt·MOTOR_ALPHA` olur | `dynamics.coulomb_compensation` |
| `gains.h` → `P_FC` | Sürtünme kuvveti T_f·N/r [N]; arayüzden üretilir (şu anki `gains.h`'de rapordaki 3 N tasarım değeri; ölçülmedi) | `export.gains_header` |
| `gains.h` → `P_B` | Artık motor mili viskoz sürtünmesini de (B_m·N²/r²) içerir | `PendulumParams.b_cart` |

**Neden:** Kuru sürtünme swing-up'ın küçük itmelerini yutar. Mekatronik tasarım değerleriyle (3 N) telafisiz +20 g sarkaç ancak ~7 s'de, uçsuz sarkaç hiç kalkmıyor; telafiyle +20 g ~4.3 s'de kalkıyor (`docs/mekatronik_on_degerlendirme.md`). Telafi yalnız swing-up'ta çalışır; LQR'da ve homing'de yoktur. Homing sürtünmeyi hız döngüsünün integraliyle (PI, `HOME_KI`) yener; böylece T_f yanlış tahmin edilse ya da hiç ölçülmese de KALİBRE ET biter (#30). `params.json`'da T_f şimdilik rapordaki 3 N'dan (tasarım değeri). Parametreler yer tutucu sayıldığı sürece arayüz bunu gerçek karta onaysız göndermez; kart `gains.h`'deki `P_FC` (3 N) ile çalışır. Simülasyonda gerçek sürtünme 0–4.5 N iken de bu telafiyle kalkış sürüyor; yine de **Sürtünme Ölç** ile gerçek değer bulunmalı (aşağıdaki kontrol listesi, 4. adım).

## İlk açılış kontrol listesi
1. Motor sürücüsünün EN pini bağlı değilken yükleyin, Seri Monitör'de (921600) telemetrinin geldiğini görün.
2. `docs/protokol.md` §2'deki işaret kontrolünü yapın (sarkacı **sola** yatırınca `th` pozitif olmalı; değilse `ANGLE_SIGN = -1`).
3. Arayüzden **KALİBRE ET** ile homing'i deneyin (elinizi acil stop'ta tutun).
4. Sarkaç aşağıda dururken **Sürtünme Ölç**: sonucu kabul edin (telafi karta gider) ve **Parametre Kaydet**.
5. Önce **Swing-up açık** kutusunu kaldırıp sarkacı elle dik tutun, **BAŞLAT** ile yalnızca LQR'ı deneyin. Sonra swing-up'ı açın.
