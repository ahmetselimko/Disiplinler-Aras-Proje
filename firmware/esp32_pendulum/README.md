# ESP32 Referans Firmware

**Durum:** Bilgisayar ekibi tarafından protokolü ve kontrol mantığını tanımlamak için yazılmış bir iskelettir. **Hatasız derleniyor, ama kartta henüz denenmedi.** Mekatronik ekibi kendi donanımına göre uyarlamalıdır.

## Gereken
* Arduino IDE 2.x + "esp32 by Espressif" kart paketi (core 2.x ve 3.x desteklenir)
* Kütüphane: **ArduinoJson** (v7)
* Kart: ESP32-WROOM-32 (çift çekirdek)

## Derleme
Hatasız ve uyarısız (`--warnings all`) derlendiği doğrulanan sürümler:

| Araç | Sürüm |
|---|---|
| arduino-cli | 1.5.1 |
| esp32 by Espressif | **3.3.12** ve **2.0.17** |
| ArduinoJson | 7.4.3 |
| Kart seçimi | ESP32 Dev Module (`esp32:esp32:esp32`) |

Komut satırından:
```bash
arduino-cli core install esp32:esp32 --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli lib install ArduinoJson
arduino-cli compile --fqbn esp32:esp32:esp32 firmware/esp32_pendulum
```
Aynı derleme her push ve PR'da GitHub Actions'ta iki core sürümüyle tekrarlanır (`.github/workflows/test.yml`).

## Dosyalar
| Dosya | İçerik |
|---|---|
| `esp32_pendulum.ino` | 500 Hz kontrol görevi (çekirdek 1) + iletişim görevi (çekirdek 0) |
| `gains.h` | LQR kazancı, eşikler, model sabitleri, redüktör oranı ve kasnak yarıçapı. PC arayüzünden üretilir (**gains.h Dışa Aktar**) |

Araba konumu enkoder sayımından `x = sayım · 2π·PULLEY_R_M / (ENC_CPR_MOTOR · MOTOR_GEAR)` ile hesaplanır. Redüktör (`MOTOR_GEAR`) ve kasnak (`PULLEY_R_M`) `gains.h`'den gelir: motor veya kasnak değişince arayüzde girip yeniden dışa aktarmak yeter. Motor milindeki enkoder çözünürlüğü (`ENC_CPR_MOTOR`, 37D için 64) ise motora özgüdür; motor değişirse `.ino`'da elle düzenlenir.

## Pinler (varsayılan — düzenleyin)
| İşlev | Pin |
|---|---|
| AS5600 SDA / SCL | 21 / 22 (400 kHz) |
| Enkoder A / B | 32 / 33 (harici 10k pull-up) |
| BTS7960 RPWM / LPWM / EN | 25 / 26 / 27 |
| Limit switch sol / sağ | 18 / 19 (INPUT_PULLUP). **NC (normalde kapalı) switch önerilir**: basılı veya kablo kopuk → HIGH → güvenli taraf. NO kullanılırsa `LIM_PRESSED = LOW` yapın; o zaman kopuk kablo fark edilmez |

**Donanımsal acil stop** (motor beslemesini kesen mantar buton) yazılımdan bağımsız olarak mutlaka eklenmelidir. Yazılımdaki `estop` bunun yerine geçmez.

## Güvenlik davranışları (inceleme sonrası eklendi)
| Durum | Davranış |
|---|---|
| Homing'de sağ switch'e basıldı | FAULT `homing_wrong_side` (motor veya enkoder yönü ters) |
| Homing 20 s'yi aştı / rayın 2 katından fazla yol gidildi | FAULT `homing_timeout` / `homing_travel` |
| Homing başlarken sol switch zaten basılı | Hemen sıfırlanır (switch seviyesine bakılır) |
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
| `gains.h` → `P_FC` | Sürtünme kuvveti T_f·N/r [N]; arayüzden üretilir (`motor.T_f` = 0 iken 0) | `export.gains_header` |
| `gains.h` → `P_B` | Artık motor mili viskoz sürtünmesini de (B_m·N²/r²) içerir | `PendulumParams.b_cart` |

**Neden:** Simülasyonda Pololu 37D'nin tahmini kuru sürtünmesi (arabada ≈1.3 N) swing-up'ın ilk dürtmesini (≈1.4 N) yutuyor, sarkaç aşağıdan kalkamıyordu; telafiyle sürtünmesiz durumdaki gibi ~3.6 s'de kalkıyor. Telafi yalnız swing-up'ta çalışır; LQR'da ve homing'de yoktur. Homing sürtünmeyi hız döngüsünün integraliyle (PI, `HOME_KI`) yener; böylece T_f yanlış tahmin edilse ya da hiç ölçülmese de KALİBRE ET biter (#30). `motor.T_f` ölçülene kadar 0 olduğundan davranış değişmez.

## İlk açılış kontrol listesi
1. Motor sürücüsünün EN pini bağlı değilken yükleyin, Seri Monitör'de (921600) telemetrinin geldiğini görün.
2. `docs/protokol.md` §2'deki işaret kontrolünü yapın.
3. Arayüzden **KALİBRE ET** ile homing'i deneyin (elinizi acil stop'ta tutun).
4. Önce **Swing-up açık** kutusunu kaldırıp sarkacı elle dik tutun, **BAŞLAT** ile yalnızca LQR'ı deneyin. Sonra swing-up'ı açın.
