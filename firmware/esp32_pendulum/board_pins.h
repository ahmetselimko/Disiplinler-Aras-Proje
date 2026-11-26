// ============================================================ KART / PİN KATMANI (HAL)
// Kontrol, protokol ve güvenlik kodu (.ino) karttan bağımsızdır; karta özgü olan yalnız bu dosyadır.
// Kart, derleme sırasında seçilen hedeften OTOMATİK belirlenir:
//   Arduino IDE  "ESP32S3 Dev Module"  / arduino-cli --fqbn esp32:esp32:esp32s3  → ESP32-S3 pinleri
//   Arduino IDE  "ESP32 Dev Module"    / arduino-cli --fqbn esp32:esp32:esp32    → klasik ESP32 pinleri
// Yeni bir kart eklemek: aşağıya bir #elif bloğu açıp aynı adlı sabitleri tanımlayın.
#pragma once

#if defined(CONFIG_IDF_TARGET_ESP32S3)
// ---- ESP32-S3-DevKitC-1 N16R8 (BOM v3, 13.11.2026) ----
// N16R8 modülünde KULLANILAMAYAN pinler: 22–25 (S3'te yok), 26–32 (SPI flash), 33–37 (8 MB oktal PSRAM).
// Ayrıca kaçınılanlar: 0/3/45/46 (açılış/strapping), 19/20 (yerel USB), 43/44 (UART0 = Serial, "COM" USB portu),
// 38/48 (kart üstü RGB LED, kart sürümüne göre), 47 (bazı modüllerde 1.8 V alanı).
#define BOARD_NAME "esp32s3-devkitc1-n16r8"
static constexpr int PIN_SDA = 8, PIN_SCL = 9;          // I2C: AS5600 (0x36) [+ ileride MPU6050 (0x68) aynı hatta]
static constexpr int PIN_ENC_A = 4, PIN_ENC_B = 5;      // motor enkoderi (harici 10k pull-up)
static constexpr int PIN_RPWM = 6, PIN_LPWM = 7;        // BTS7960
static constexpr int PIN_EN = 15;                       // BTS7960 R_EN + L_EN birlikte
static constexpr int PIN_LIM_L = 16, PIN_LIM_R = 17;    // uç sensörleri (END_SENSORS 1: tek sensör; 2: mikro switch)
static constexpr int PIN_OPT_L = 10, PIN_OPT_R = 11;    // END_SENSORS 2: iç kademe optik (TCST2103)

static constexpr bool s3PinUsable(int p) {
  return p >= 1 && p <= 21 && p != 3 && p != 19 && p != 20;   // yalnız 1–21 aralığı (3, 19, 20 hariç) kullanılıyor
}
static_assert(s3PinUsable(PIN_SDA) && s3PinUsable(PIN_SCL) && s3PinUsable(PIN_ENC_A) && s3PinUsable(PIN_ENC_B) &&
              s3PinUsable(PIN_RPWM) && s3PinUsable(PIN_LPWM) && s3PinUsable(PIN_EN) &&
              s3PinUsable(PIN_LIM_L) && s3PinUsable(PIN_LIM_R) && s3PinUsable(PIN_OPT_L) && s3PinUsable(PIN_OPT_R),
              "ESP32-S3 N16R8: flash/PSRAM/USB/strapping pini secildi");

#elif defined(CONFIG_IDF_TARGET_ESP32)
// ---- Klasik ESP32 (ESP32-WROOM-32 DevKit) — önceki varsayılan pinler ----
#define BOARD_NAME "esp32-wroom32"
static constexpr int PIN_SDA = 21, PIN_SCL = 22;        // AS5600
static constexpr int PIN_ENC_A = 32, PIN_ENC_B = 33;    // motor enkoderi (harici 10k pull-up)
static constexpr int PIN_RPWM = 25, PIN_LPWM = 26;      // BTS7960
static constexpr int PIN_EN = 27;                       // BTS7960 R_EN + L_EN birlikte
static constexpr int PIN_LIM_L = 18, PIN_LIM_R = 19;    // uç sensörleri (END_SENSORS 1: tek sensör; 2: mikro switch)
static constexpr int PIN_OPT_L = 13, PIN_OPT_R = 14;    // END_SENSORS 2: iç kademe optik (TCST2103)

#else
#error "Bu kart için pin tanımı yok: board_pins.h'e bir #elif bloğu ekleyin (ESP32 ve ESP32-S3 destekleniyor)."
#endif
