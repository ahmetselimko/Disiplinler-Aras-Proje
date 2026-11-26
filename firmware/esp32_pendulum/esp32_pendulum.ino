/*
 * Ters Sarkaç — ESP32 REFERANS firmware iskeleti (Arduino-ESP32 core 2.x / 3.x)
 * ---------------------------------------------------------------------------
 * Bilgisayar ekibi tarafından protokolü ve kontrol mantığını tanımlamak için yazıldı.
 * Mekatronik ekibi pinleri, motor sürücüsünü ve sensör yönlerini kendi donanımına göre
 * düzenlemelidir. BU DOSYA HATASIZ DERLENİYOR (sürümler: README) AMA HENÜZ GERÇEK KARTTA DENENMEDİ.
 *
 * Mantık, Python tarafındaki pendulum_lab/control/supervisor.py, swingup.py ve
 * comm/mock_device.py (komut kabul/ret kuralları) ile BİREBİR aynıdır — birinde değişiklik
 * yapılırsa diğeri de güncellenmelidir.
 *
 * Görevler (FreeRTOS):
 *   ctrlTask  (çekirdek 1, en yüksek öncelik, CTRL_PERIOD_MS): sensör → durum kestirimi → kontrol → motor
 *   commTask  (çekirdek 0): seri porttan komut oku, telemetri ve olayları gönder
 * Seri port yazmak kontrol döngüsünü HİÇ bekletmez (ayrı çekirdek).
 *
 * Gerekli kütüphane: ArduinoJson (v7). Kazançlar: gains.h (PC arayüzünden "gains.h Dışa Aktar").
 */

#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <math.h>
#include "gains.h"

// ============================================================ DERLEME ANI DENETİMLERİ
#if !defined(CTRL_PERIOD_MS) || !defined(MOTOR_V_MAX) || !defined(MOTOR_GEAR) || !defined(PULLEY_R_M)
#error "gains.h eski: CTRL_PERIOD_MS / MOTOR_V_MAX / MOTOR_GEAR / PULLEY_R_M yok. Arayüzden yeniden dışa aktarın."
#endif
#ifndef THETA_POSITIVE_LEFT
#error "gains.h eski θ kuralıyla (sağa +) üretilmiş: LQR_K ve KALMAN_* ters işaretli, sarkaç düşer. Arayüzden yeniden dışa aktarın (#50)."
#endif
#ifndef P_FC
#define P_FC 0.0f   // elle yazılmış/eksik gains.h için: Coulomb telafisi kapalı (#50 öncesi gains.h zaten derlenmez)
#endif
// elle yazılmış/eksik gains.h için supervisor.py varsayılanları (#50 öncesi dışa aktarılan gains.h yukarıda #error verir)
#ifndef CATCH_DTH
#define CATCH_DTH 5.0f     // LQR'a girişte |θ̇| sınırı [rad/s]
#endif
#ifndef CATCH_X
#define CATCH_X 0.40f      // LQR'a girişte |x| sınırı [m]
#endif
#ifndef CATCH_DX
#define CATCH_DX 2.0f      // LQR'a girişte |ẋ| sınırı [m/s]
#endif
#ifndef SU_E_RATIO
#define SU_E_RATIO 1.05f   // swing-up hedef enerji oranı (1 = dik konum; swingup.EnergySwingUp varsayılanı)
#endif
#ifndef INPUT_MODE_VOLTAGE
#error "Bu firmware yalnız DC motor (input_mode = voltage) için yazıldı. Step motor için motor sürme kısmı yeniden yazılmalı."
#endif
#if configTICK_RATE_HZ != 1000
#error "FreeRTOS tik hızı 1000 Hz olmalı (Arduino-ESP32 varsayılanı)."
#endif
#if CTRL_PERIOD_MS < 1 || CTRL_PERIOD_MS > 20
#error "CTRL_PERIOD_MS 1–20 ms olmalı (protokol.md §7). gains.h'i arayüzden yeniden dışa aktarın."
#endif

// ============================================================ PİNLER
// Pin numaraları karta göre board_pins.h'de (ESP32-S3-DevKitC-1 N16R8 ya da klasik ESP32; derleme hedefinden seçilir).
#include "board_pins.h"
// Uç sensörleri (#50 Aşama 4; mekatronik raporu 8.4 iki kademe öneriyor). END_SENSORS:
//   0: sensör yok. Homing yerine "elle ortalama": araba elle ray ortasına konur, KALİBRE ET (home) o noktayı
//      x = 0 yapar; ray güvenliğini yalnız enkoderle yazılımsal sınır (SOFT_MARGIN) sağlar.
//   1: her uçta tek sensör (PIN_LIM_L/R; limit switch ya da TCST2103): homing referansı + "limit" FAULT'u.
//   2: her uçta iki kademe. İç kademe optik (PIN_OPT_L/R, TCST2103; mikro switch'ten OPT_FROM_END_M içeride):
//      homing referansı, çalışırken "limit" FAULT'u. Dış kademe mikro switch (PIN_LIM_L/R, ray sonunda):
//      çalışırken "hard_limit", homing'de "homing_overrun"; basılıyken start/home/friction her modda
//      reddedilir; aynı taraftaki optik görmüyorsa "sensor_mismatch" olayı (optik arızalı ya da kaymış).
// Takılı olmayan bir sensör 1/2 bırakılırsa NC/TCST2103 ayarındaki boş pin "etkin" okunur, kart hemen hataya düşer.
#ifdef LIMIT_SWITCHES                                  // eski ad (#50 öncesi): END_SENSORS = LIMIT_SWITCHES
#ifndef END_SENSORS
#define END_SENSORS LIMIT_SWITCHES
#endif
#warning "LIMIT_SWITCHES eski ad: END_SENSORS kullanın (0: yok, 1: tek sensör, 2: optik + mikro switch)"
#endif
#ifndef END_SENSORS
#define END_SENSORS 1
#endif
#if END_SENSORS < 0 || END_SENSORS > 2
#error "END_SENSORS 0, 1 ya da 2 olmalı"
#endif
// Uç sensörü (PIN_LIM_L/R; END_SENSORS 1'de tek sensör, 2'de mikro switch): limit switch (DC163-3) ya da TCST2103.
static const int LIM_PRESSED = HIGH;                  //   NC switch + INPUT_PULLUP → basılı/kopuk kablo = HIGH (güvenli)
// Optik (PIN_OPT_L/R, END_SENSORS 2): TCST2103, pull-up'lı fototransistör çıkışı → yarığa bayrak girince ışık
// kesilir, çıkış HIGH; kopuk kablo da HIGH okunur (güvenli tarafta).
static const int OPT_ACTIVE = HIGH;
static const float OPT_FROM_END_M = 0.03f;            // optik, mikro switch'ten (ray sonundan) bu kadar içeride [m] (mock_device.OPT_FROM_END)
static const float HOME_BACKOFF_M = 0.01f;             // homing optik etkinken başlarsa: optik bırakıldıktan sonra bu kadar daha sağa (mock_device.HOME_BACKOFF)
#if END_SENSORS == 2                                   // homing referansı ve "limit" sensörü: iç kademe optik
#define PIN_REF_L PIN_OPT_L
#define PIN_REF_R PIN_OPT_R
static const int REF_ACTIVE = OPT_ACTIVE;
static const float REF_FROM_END_M = OPT_FROM_END_M;
#else                                                  // tek sensör (ray sonunda)
#define PIN_REF_L PIN_LIM_L
#define PIN_REF_R PIN_LIM_R
static const int REF_ACTIVE = LIM_PRESSED;
static const float REF_FROM_END_M = 0.0f;
#endif
                                                      //   TCST2103 (pull-up'lı çıkış): yarığa bayrak girince = HIGH,
                                                      //     kopuk kablo da HIGH (güvenli) → ayar değişmez
                                                      //   NO switch kullanılırsa LOW yapın (kopuk kablo fark edilmez!)

// ============================================================ DONANIM SABİTLERİ
static const char *FW_NAME = "ref-1.2";                // 1.2: θ sola + (#50); boot/ping "theta":"sol" bildirir
static const uint8_t AS5600_ADDR = 0x36;
static const float ANGLE_SIGN = +1.0f;                // θ > 0: sarkaç SOLA yatık (mekatronik raporu 8.1.3, #50). Sola yatınca θ artmıyorsa -1 yapın
static const float X_SIGN = +1.0f;                    // araba sağa giderken x artmıyorsa -1 yapın
static const float ENC_CPR_MOTOR = 44.0f;             // motor milindeki enkoder, 4x sayım: JGB37-520 (rapor: 836 sayım/çıkış turu ÷ 19 = 44; motorda doğrulanacak)
// redüktör ve kasnak gains.h'den gelir: motor/kasnak değişince arayüzden yeniden dışa aktarmak yeter
static const float ENC_COUNTS_PER_REV = ENC_CPR_MOTOR * MOTOR_GEAR;
static const float X_PER_COUNT = 2.0f * PI * PULLEY_R_M / ENC_COUNTS_PER_REV;
static const float V_SUPPLY = MOTOR_V_MAX;            // arayüzdeki motor.V_max ile aynı (gains.h)
static const int   PWM_FREQ = 20000, PWM_BITS = 10;   // 20 kHz (duyulmaz), 0..1023
static const float VEL_FC_HZ = 25.0f;                 // hız kestirim filtresi (Python ile aynı)
#ifndef USE_KALMAN
#define USE_KALMAN 1                                   // 0: dik bölgede de yalnız 25 Hz türev filtresi (Kalman kapalı)
#endif
#if USE_KALMAN && !defined(KALMAN_AVAILABLE)          // KALMAN_* matrisleri olmayan (elle yazılmış) gains.h → filtresiz derle
#warning "gains.h Kalman matrislerini icermiyor: Kalman KAPALI derleniyor. Arayuzden gains.h'yi yeniden uretin."
#undef USE_KALMAN
#define USE_KALMAN 0
#endif
// MPU6050 jiroskobu (#50 Aşama 5; mekatronik raporu 8.1.4, Denklem 8.5). Varsayılan KAPALI: 0 iken kod ve
// telemetri birebir eskisi gibi. 1: AS5600 ile aynı I²C hattı (PIN_SDA/PIN_SCL), adres 0x68 (AD0 = GND).
// θ̇ = γ·ω_gyro + (1 − γ)·θ̇_türev, γ = set_param gyro_mix (varsayılan 0) — YALNIZ türev filtresinin kullanıldığı
// bölgede (|θ| > release ya da Kalman kapalı); Kalman bölgesinde Kalman aynen kalır (jiroskobu Kalman'a ölçüm
// olarak eklemek ileride). MPU okunamazsa FAULT YOK: yedek sensör, karışım 0'a düşer, "gyro" olayı bir kez.
#ifndef USE_MPU6050
#define USE_MPU6050 0
#endif
#if USE_MPU6050
static const uint8_t MPU_ADDR = 0x68;
static const int   MPU_AXIS = 0;                       // sarkaç eksenine paralel jiroskop ekseni: 0 = X, 1 = Y, 2 = Z
static const float MPU_SIGN = +1.0f;                   // sarkaç sola (+θ) dönerken gy pozitif değilse −1
static const float MPU_LSB_RAD = (1.0f / 16.4f) * PI / 180.0f;   // ±2000 °/s aralığında 1 LSB [rad/s]
static const int   MPU_FAIL_LIMIT = 3;                 // art arda bu kadar okunamazsa jiroskop devre dışı
// Tutarlılık denetimi: okuma başarılı ama değer yanlış olabilir (MPU güç kesintisinde resetlenip uykuya döner:
// ACK verir, yazmaçlar 0, aralık ±250 °/s). |gy − θ̇_türev| art arda GYRO_CHECK_N adım GYRO_CHECK_RAD_S'yi
// aşarsa jiroskop devre dışı (I²C arızasıyla aynı: FAULT yok, gyro olayı). AS5600 türev gürültüsü ~0.8 rad/s.
static const float GYRO_CHECK_RAD_S = 3.0f;
static const int   GYRO_CHECK_N = 25;                  // 50 ms
// Hareketsizken (ya da γ = 1'de donmuş jiroskop swing-up'ı durdurunca) tutarlılık denetimi tetiklenmez; bu yüzden
// ayrıca her MPU_PWR_CHECK_N adımda PWR_MGMT_1 okunur: resetlenen MPU uykuda (0x40) ve saat 0 → devre dışı.
static const int   MPU_PWR_CHECK_N = 50;               // 100 ms, 1 bayt ek okuma
static const int   MPU_BIAS_SAMPLES = 200;             // calib: sarkaç hareketsizken bias ortalaması
#endif
static const float SOFT_MARGIN = 0.03f;               // rayın ucundan bu kadar önce FAULT [m]
static const float START_X_MARGIN = 0.10f;            // BAŞLAT için rayın ucuna en az bu kadar uzak [m]
static const float START_MAX_SPEED = 0.2f;            // BAŞLAT için araba hızı bundan düşük [m/s]
static const float HOME_SPEED = 0.10f;                // homing hızı [m/s]
static const float HOME_TIMEOUT_S = 20.0f;
// homing hız döngüsü PI (mock_device.HOME_KP/KI/I_MAX ile aynı): integral sürtünmeyi yener, telafi kullanılmaz (#30)
static const float HOME_KP = 20.0f;                   // [1/s]
static const float HOME_KI = 30.0f;                   // [1/s²]
static const float HOME_I_MAX = 10.0f;                // integral sınırı [m/s²]
static const float HOME_RUNAWAY_V = 0.30f;            // homing'de |ẋ| bunu aşarsa (normal ≤ 0.15) → FAULT (yön ters)
static const float HOME_SAT_S = 0.30f;                // homing'de gerilim bu süreden uzun doyumda → FAULT
static const float HOME_SW_CLEAR_M = 0.05f;            // sol switch'ten bu kadar sağa gidildiği hâlde hâlâ basılıysa
                                                       // sensör takılı/kablo kopuk (NC ve TCST2103'te kopuk = basılı) → FAULT
static const float HOME_STILL_DX = 0.01f;             // elle ortalamada araba hızı bundan düşük olmalı [m/s] (mock ile aynı)
static const int   SENSOR_FAIL_LIMIT = 3;             // art arda bu kadar okuma hatası → FAULT
// bağlantı bekçisi: hareketli modda PC'den bu süre satır gelmezse FAULT "link" (protocol.py LINK_TIMEOUT_S ile aynı)
static const uint32_t LINK_TIMEOUT_MS = 1000;
static const int   LIM_DEBOUNCE = 3;                  // uç sensörü bu kadar art arda okumada basılıysa basılı, bu kadar art arda boşsa bırakılmış (6 ms)
static const float CALIB_MAX_SPREAD = 0.02f;          // kalibrasyonda izin verilen sapma [rad] (~1.1°)

// sürtünme ölçümü — pendulum_lab/control/friction.py ile AYNI (test denetler)
static const float FR_RATE = 1.0f;                    // gerilim rampası [V/s]
static const float FR_STOP_RATE = 2.0f;               // yön sonunda gerilimin indirilme hızı [V/s]
static const float FR_WINDOW = 0.04f;                 // hızın yer değiştirmeden hesaplandığı pencere [s]
static const float FR_V_LO = 0.04f;                   // doğru oturtmada en düşük hız [m/s]
static const float FR_V_HI = 0.10f;                   // bu hızda o yöndeki ölçüm biter [m/s]
static const float FR_MAX_VOLT = 6.0f;                // [V]
static const float FR_TRAVEL = 0.08f;                 // yön başına ölçüm yolu [m]
static const float FR_X_MARGIN = 0.15f;               // başlarken ray ucuna FR_TRAVEL + bu kadar uzaklık [m]
static const float FR_TIMEOUT = 8.0f;                 // yön başına [s]
static const float FR_SETTLE = 0.3f;                  // yön değiştirmeden önce durma süresi [s]
static const float FR_STILL_DIST = 0.002f;            // [m]
static const float FR_SETTLE_TIMEOUT = 3.0f;          // [s]
static const float FR_MIN_SAMPLES = 5.0f;             // en az pencere
static const float FR_MIN_SPAN = 0.03f;               // pencerelerin kapsaması gereken hız aralığı [m/s]
static const float FR_SWING_MAX = 0.3f;               // sarkaç aşağıdan bu kadar saparsa iptal [rad]
static const float FR_STILL_DTH = 0.3f;               // başlarken sarkaç açısal hızı sınırı [rad/s]
static const float FR_STILL_DX = 0.01f;               // başlarken araba hızı sınırı [m/s]

// ============================================================ set_param TABLOSU
// pendulum_lab/comm/protocol.py::SET_PARAM_LIMITS ile AYNI olmalı (test bunu denetler).
struct ParamLimit { const char *key; float lo, hi; };
static const ParamLimit PARAM_LIMITS[] = {
  {"catch_deg", 1.0f, 45.0f},
  {"release_deg", 2.0f, 90.0f},
  {"catch_dth", 0.5f, 50.0f},
  {"catch_x", 0.01f, 1.0f},
  {"catch_dx", 0.01f, 5.0f},
  {"swingup", 0.0f, 1.0f},
  {"k_E", 0.0f, 10000.0f},
  {"a_max", 0.0f, 50.0f},
  {"k_x", 0.0f, 1000.0f},
  {"k_v", 0.0f, 1000.0f},
  {"E_ratio", 0.5f, 1.5f},
  {"x_ref", -0.3f, 0.3f},
  {"u_max", 0.0f, 48.0f},
  {"telemetry_hz", 1.0f, 500.0f},
  {"fc_volt", 0.0f, 6.0f},
  {"gyro_mix", 0.0f, 1.0f},
};

// ============================================================ MODLAR (Python Mode ile aynı adlar)
enum Mode : uint8_t { IDLE, HOMING, WAIT, SWING_UP, BALANCE, FAULT, FRICTION };
static const char *MODE_NAMES[] = {"IDLE", "HOMING", "WAIT", "SWING_UP", "BALANCE", "FAULT", "FRICTION"};

// ============================================================ PAYLAŞILAN DURUM (her erişim mux içinde)
struct Shared {
  // kontrol → iletişim
  uint32_t n; float t, x, dx, th, dth, u; Mode mode;   // telemetri örneği (telemetry_hz'de)
  bool tel_valid;                                    // ilk gerçek örnek yazıldı mı (açılışta sıfırlar gönderilmesin)
  float lx, ldx, lth, ldth;                          // HER adımda güncel durum (komut kabul kuralları için)
  bool homed, angle_ok, calib_done;                  // calib_done: açılış kalibrasyonu bitti (önce angle_ok anlamsız)
  // iletişim → kontrol
  float K[4]; float catch_rad, release_rad, x_ref, u_max;
  float catch_dth, catch_x, catch_dx;                // LQR'a giriş sınırları: |θ̇| [rad/s], |x| [m], |ẋ| [m/s]
  float su_kE, su_amax, su_kx, su_kv, su_akick, su_eratio; bool swingup_on;
  float fc_volt;                                     // sürtünme telafisi [V] (set_param fc_volt)
  bool req_start, req_stop, req_estop, req_home, req_calib, req_friction;
  uint32_t last_rx_ms;                               // son komut satırının geldiği an (bağlantı bekçisi)
  // olaylar (kontrol üretir, iletişim gönderir)
  bool ev_homed, ev_fault, ev_limitL, ev_limitR, ev_calib, ev_friction;
  bool ev_hardL, ev_hardR, ev_mismatchL, ev_mismatchR;   // END_SENSORS 2: mikro switch ve tutarsızlık olayları
  float gyro_mix;                                    // MPU6050 karışımı γ (set_param gyro_mix)
  float gy, tel_gy, gyro_bias; bool mpu_ok, ev_gyro_fail;   // USE_MPU6050: son ölçüm [rad/s], telemetri örneği, bias, durum, olay
  uint32_t i2c_us;                                   // USE_MPU6050: AS5600 + MPU okuma süresi [µs] (en büyük)
  bool hardL, hardR;                                 // mikro switch şu an basılı mı (komut reddi için, her adımda)
  bool fr_ok; const char *fr_reason; float fr_v_pos, fr_v_neg, fr_v_c, fr_slope;   // sürtünme ölçümü sonucu
  const char *fault_reason;
};
static Shared S;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
#define SET_SHARED(field, val) do { portENTER_CRITICAL(&mux); S.field = (val); portEXIT_CRITICAL(&mux); } while (0)
static volatile int telemetry_hz = 200;

// ============================================================ ENKODER (ISR, dörtlü kod çözme)
static volatile int32_t enc_count = 0;
static volatile uint8_t enc_prev = 0;
static const int8_t QDEC[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
void IRAM_ATTR encISR() {
  uint8_t s = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  enc_count += QDEC[(enc_prev << 2) | s];
  enc_prev = s;
}

// ============================================================ YARDIMCILAR
static inline float wrapAngle(float a) { return atan2f(sinf(a), cosf(a)); }
static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

struct VelEst {               // geri fark + 1. derece alçak geçiren (filters.VelocityEstimator)
  float a = 0, y = 0, prev = 0; bool init = false; bool angular = false;
  void setup(float dt, float fc, bool ang) { float tau = 1.0f / (2 * PI * fc); a = dt / (tau + dt); angular = ang; }
  float update(float pos, float dt) {
    if (!init) { prev = pos; init = true; return y; }
    float d = pos - prev; if (angular) d = wrapAngle(d); prev = pos;
    y += a * (d / dt - y); return y;
  }
  void reset() { init = false; y = 0; }
};

struct KalmanEst {            // ayrık kararlı hal Kalman filtresi (filters.KalmanFilter ile aynı)
  float s[4] = {0, 0, 0, 0}; bool active = false;
  void reset() { active = false; s[0] = s[1] = s[2] = s[3] = 0; }
  void update(float xm, float dx_lp, float thm, float dth_lp, float u_prev, bool enable, float &dx, float &dth) {
    if (!enable) { active = false; dx = dx_lp; dth = dth_lp; return; }
#if !USE_KALMAN                                        // KALMAN_* sabitlerine hiç başvurma (Kalman'sız gains.h derlensin)
    (void)xm; (void)thm; (void)u_prev; dx = dx_lp; dth = dth_lp;
#else
    if (!active) {
      s[0] = xm; s[1] = dx_lp; s[2] = wrapAngle(thm); s[3] = dth_lp;
      active = true; dx = dx_lp; dth = dth_lp; return;
    }
    float sp[4];
    for (int i = 0; i < 4; i++) {
      sp[i] = KALMAN_BD[i] * u_prev;
      for (int j = 0; j < 4; j++) sp[i] += KALMAN_AD[i][j] * s[j];
    }
    sp[2] = wrapAngle(sp[2]);
    float ex = xm - sp[0], eth = wrapAngle(thm - sp[2]);
    for (int i = 0; i < 4; i++) s[i] = sp[i] + KALMAN_L[i][0] * ex + KALMAN_L[i][1] * eth;
    s[2] = wrapAngle(s[2]);
    dx = s[1]; dth = s[3];
#endif
  }
};

static bool i2cReadDev(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

#if USE_MPU6050
static bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}
// Uykudan çıkar, saat = X ekseni PLL; DLPF_CFG = 2 → jiroskop bant genişliği ≈ 98 Hz (gecikme ≈ 2.8 ms; 500 Hz
// döngüde örtüşmeyi sınırlar, θ̇'yi fazla geciktirmez); jiroskop aralığı ±2000 °/s (swing-up'ta θ̇ 15–20 rad/s).
static bool mpuInit() {
  uint8_t who = 0;
  if (!i2cReadDev(MPU_ADDR, 0x75, &who, 1) || who != 0x68) return false;   // WHO_AM_I
  return mpuWrite(0x6B, 0x01) && mpuWrite(0x1A, 0x02) && mpuWrite(0x1B, 0x18);
}
static bool mpuReadGyro(float &w) {            // yalnız gereken eksenin 2 baytı
  uint8_t b[2];
  if (!i2cReadDev(MPU_ADDR, 0x43 + 2 * MPU_AXIS, b, 2)) return false;
  w = MPU_SIGN * (int16_t)((b[0] << 8) | b[1]) * MPU_LSB_RAD;
  return true;
}
#endif

static bool i2cRead(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(AS5600_ADDR, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();   // sıra garanti: önce yüksek bayt
  return true;
}

static bool readAS5600(uint16_t &raw) {
  uint8_t b[2];
  if (!i2cRead(0x0C, b, 2)) return false;             // RAW ANGLE (12 bit)
  raw = ((uint16_t)(b[0] & 0x0F) << 8) | b[1];
  return true;
}

static bool magnetOk() {
  uint8_t st;
  if (!i2cRead(0x0B, &st, 1)) return false;            // STATUS: MD=bit5, ML=bit4, MH=bit3
  return (st & 0x20) && !(st & 0x18);
}

static inline float rawToRad(uint16_t raw) { return raw * (2 * PI / 4096.0f); }

// Sarkaç aşağıda ve hareketsizken açı sıfırını bulur. Dairesel ortalama: 4095↔0 sınırında da doğru.
static bool calibrateAngle(float &hang_rad) {
  if (!magnetOk()) return false;
  float sx = 0, sy = 0; int ok = 0;
  float a[64];
  for (int i = 0; i < 64; i++) {
    uint16_t raw;
    if (readAS5600(raw)) { a[ok] = rawToRad(raw); sx += cosf(a[ok]); sy += sinf(a[ok]); ok++; }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  if (ok < 48) return false;
  float mean = atan2f(sy, sx);
  for (int i = 0; i < ok; i++)
    if (fabsf(wrapAngle(a[i] - mean)) > CALIB_MAX_SPREAD) return false;   // sarkaç hareket ediyor
  hang_rad = mean;
  return true;
}

#if USE_MPU6050
// Jiroskop sıfırı: sarkaç aşağıda ve hareketsizken (calib ile aynı koşul) MPU_BIAS_SAMPLES örneğin ortalaması.
static bool calibrateGyro(float &bias) {
  float sum = 0; int ok = 0;
  for (int i = 0; i < MPU_BIAS_SAMPLES; i++) {
    float w;
    if (mpuReadGyro(w)) { sum += w; ok++; }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  if (ok < MPU_BIAS_SAMPLES * 3 / 4) return false;
  bias = sum / ok;
  return true;
}
#endif

static void motorWrite(float volts) {
  float duty = clampf(fabsf(volts) / V_SUPPLY, 0, 1) * ((1 << PWM_BITS) - 1);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_RPWM, volts > 0 ? (uint32_t)duty : 0);
  ledcWrite(PIN_LPWM, volts < 0 ? (uint32_t)duty : 0);
#else
  ledcWrite(0, volts > 0 ? (uint32_t)duty : 0);
  ledcWrite(1, volts < 0 ? (uint32_t)duty : 0);
#endif
}

// Swing-up ivmesini motor gerilimine çevir (dynamics.accel_to_input ile aynı)
static const float FC_V_COMP = 0.01f;   // [m/s] altında araba duruyor sayılır (dynamics.V_COMP)
// Coulomb telafisi [N]: başta gains.h'deki P_FC; set_param fc_volt (sürtünme ölçümü) ile değişir: fc_N = fc_volt·α
static float fc_N = P_FC;
static float accelToVolt(float dx, float th, float dth, float a, bool coulomb) {
  float s = sinf(th), c = cosf(th), ml = P_M_POLE * P_L;
  float ddth = (ml * P_G * s - P_C * dth + ml * c * a) / P_J;            // θ sola +: J·θ̈ = mgl·sinθ + ml·cosθ·ẍ − c·θ̇
  float F = (P_M_CART + P_M_POLE) * a - ml * c * ddth + ml * s * dth * dth + P_B * dx;
  // Coulomb sürtünme telafisi (dynamics.coulomb_compensation): hareket yönünde, duruyorsa istenen ivme yönünde.
  // Yalnız swing-up'ta; homing'de kapalı (coulomb = false)
  if (coulomb) {
    float d = fabsf(dx) > FC_V_COMP ? dx : a;
    if (d > 0.0f) F += fc_N; else if (d < 0.0f) F -= fc_N;
  }
  return (F + MOTOR_BETA * dx) / MOTOR_ALPHA;
}

// Enerji bazlı swing-up (swingup.EnergySwingUp ile aynı). θ sola + iken dE/dt = m·l·a·θ̇·cosθ → a = −k_E·Ẽ·θ̇·cosθ
static float swingUpAccel(const Shared &p, float x, float dx, float th, float dth) {
  float E = 0.5f * P_J * dth * dth + P_M_POLE * P_G * P_L * (cosf(th) - 1.0f);
  float E_target = (p.su_eratio - 1.0f) * 2 * P_M_POLE * P_G * P_L;   // EnergySwingUp.target_energy
  float a;
  if (fabsf(dth) < 0.05f && fabsf(wrapAngle(th)) > 2.8f && E < -0.99f * 2 * P_M_POLE * P_G * P_L)
    a = p.su_akick * (x > 1e-3f ? -1.0f : 1.0f);
  else
    a = -p.su_kE * (E - E_target) * dth * cosf(th);
  a = clampf(a, -p.su_amax, p.su_amax);
  a += -p.su_kx * x - p.su_kv * dx;
  return clampf(a, -p.su_amax, p.su_amax);
}

// ============================================================ KONTROL GÖREVİ
// ============================================================ SÜRTÜNME ÖLÇÜMÜ (control/friction.py::FrictionTest ile aynı)
static const char *FR_ERR_SWING = "sarkaç sallandı";
static const char *FR_ERR_SETTLE = "araba durmadı";
static const char *FR_ERR_SPEED = "yeterli hıza ulaşılamadı (gerilim/yol/süre sınırı)";
struct FrictionTest {
  int dir = 1, phase = 0, nd = 0;                      // phase: 0 rampa, 1 yumuşak duruş, 2 bekleme
  bool started = false, ok = false;
  const char *reason = nullptr;
  float t = 0, volt = 0, x0 = 0, w_t = 0, w_V = 0, w_x0 = 0, vmin = 0, vmax = 0;
  float n = 0, sv = 0, sV = 0, svv = 0, svV = 0, sums[2][5] = {};
  float v_pos = 0, v_neg = 0, v_c = 0, slope = 0;

  void reset() { dir = 1; nd = 0; started = false; ok = false; reason = nullptr; }
  void window(float x) { w_t = 0; w_V = 0; w_x0 = x; }
  void beginRamp(float x) {
    phase = 0; t = 0; volt = 0; x0 = x; n = sv = sV = svv = svV = 0; vmin = 1e9f; vmax = -1e9f; window(x);
  }
  bool fail(const char *r, float &u) { ok = false; reason = r; u = 0; return true; }
  bool finish(float &u) {                              // ortak eğimli en küçük kareler: V = V_c(yön) + k·v
    float Svv = 0, SvV = 0;
    for (int i = 0; i < 2; i++) {
      Svv += sums[i][3] - sums[i][1] * sums[i][1] / sums[i][0];
      SvV += sums[i][4] - sums[i][1] * sums[i][2] / sums[i][0];
    }
    if (Svv <= 0) return fail(FR_ERR_SPEED, u);
    slope = SvV / Svv;
    v_pos = (sums[0][2] - slope * sums[0][1]) / sums[0][0];
    v_neg = (sums[1][2] - slope * sums[1][1]) / sums[1][0];
    v_c = (v_pos + v_neg) / 2; ok = true; u = 0; return true;
  }
  // bir kontrol adımı; ölçüm bitince true döner (sonuç ok/reason/v_* alanlarında).
  // u_lim: giriş sınırı (u_max); rampa bunu aşmaz, yoksa kaydedilen gerilim uygulanandan yüksek olurdu.
  // (Adı vmax olmamalı: o ad, ölçülen en yüksek hızı tutan üye değişkeni gölgeler.)
  bool step(float x, float th, float dt, float u_lim, float &u) {
    const float cap = fminf(FR_MAX_VOLT, u_lim);
    if (!started) { started = true; beginRamp(x); }
    if (fabsf(PI - fabsf(wrapAngle(th))) > FR_SWING_MAX) return fail(FR_ERR_SWING, u);
    t += dt;
    if (phase == 1) {                                  // gerilimi yavaşça indir
      volt = fmaxf(0.0f, volt - FR_STOP_RATE * dt);
      if (volt > 0.0f) { u = dir * volt; return false; }
      if (nd == 2) return finish(u);
      phase = 2; t = 0; window(x); u = 0; return false;
    }
    if (phase == 2) {                                  // FR_SETTLE boyunca FR_STILL_DIST'ten az hareket
      w_t += dt; u = 0;
      if (fabsf(x - w_x0) > FR_STILL_DIST) window(x);
      else if (w_t >= FR_SETTLE) { dir = -1; beginRamp(x); }
      else if (t > FR_SETTLE_TIMEOUT) return fail(FR_ERR_SETTLE, u);
      return false;
    }
    bool done_w = w_t >= FR_WINDOW - 1e-6f;
    float speed = 0;
    if (done_w) {                                      // pencere: (ortalama gerilim, yer değiştirmeden hız)
      speed = dir * (x - w_x0) / w_t;
      float v = w_V / w_t;
      if (speed >= FR_V_LO && speed <= FR_V_HI) {
        n += 1; sv += speed; sV += v; svv += speed * speed; svV += speed * v;
        vmin = fminf(vmin, speed); vmax = fmaxf(vmax, speed);
      }
      window(x);
    }
    if ((done_w && speed >= FR_V_HI) || fabsf(x - x0) > FR_TRAVEL || volt >= cap || t > FR_TIMEOUT) {
      if (n < FR_MIN_SAMPLES || vmax - vmin < FR_MIN_SPAN) return fail(FR_ERR_SPEED, u);
      float *s = sums[nd++];
      s[0] = n; s[1] = sv; s[2] = sV; s[3] = svv; s[4] = svV;
      phase = 1; u = dir * volt; return false;
    }
    volt = fminf(volt + FR_RATE * dt, cap);
    w_t += dt; w_V += volt * dt;                       // bu adımda uygulanan gerilim penceredeki ortalamaya girer
    u = dir * volt; return false;
  }
};

static Mode raiseFault(Mode m, const char *reason) {
  if (m != FAULT) {
    portENTER_CRITICAL(&mux); S.ev_fault = true; S.fault_reason = reason; portEXIT_CRITICAL(&mux);
  }
  return FAULT;
}

// İki yönlü parazit filtresi: durum ancak LIM_DEBOUNCE art arda aynı okumayla değişir (etkinleşme ve bırakma).
struct DebouncedInput {
  bool state = false; int cnt = 0;
  void seed(bool s) { state = s; cnt = 0; }
  bool update(bool raw) {
    if (raw == state) cnt = 0;
    else if (++cnt >= LIM_DEBOUNCE) { state = raw; cnt = 0; }
    return state;
  }
};

void ctrlTask(void *) {
  const float dt = CTRL_PERIOD_MS / 1000.0f;
  const TickType_t period = pdMS_TO_TICKS(CTRL_PERIOD_MS);
  VelEst vx, vth; vx.setup(dt, VEL_FC_HZ, false); vth.setup(dt, VEL_FC_HZ, true);
  KalmanEst kf; float u_prev = 0.0f;
  Mode mode = IDLE; bool homed = false; int home_phase = 0;
  float x_offset = 0, home_t0 = 0, home_x0 = 0, home_i = 0; uint32_t k = 0, n = 0; float tel_acc = 0;
  bool home_r0 = false;                                // homing başlarken sağ switch basılı mıydı
  float home_rel = NAN;                                // faz 3 (END_SENSORS 2): sol optiğin bırakıldığı enkoder konumu
  float home_sat = 0;                                  // homing'de gerilimin doyumda kaldığı süre [s]
  FrictionTest fr;
  bool limL_prev = false, limR_prev = false, hardL_prev = false, hardR_prev = false;
  // uç sensörü parazit filtresi: hem etkinleşme hem bırakma LIM_DEBOUNCE art arda okuma ister (DebouncedInput)
  DebouncedInput fL, fR;
#if END_SENSORS == 2
  DebouncedInput fHL, fHR;
#endif
  int sensor_fails = 0; uint16_t raw = 0;

  // açılış kalibrasyonu: sarkaç aşağıda ve hareketsiz olmalı → o konum θ = π
  vTaskDelay(pdMS_TO_TICKS(1000));
  float raw_hang = 0;
  bool angle_ok = calibrateAngle(raw_hang);
  // uç sensörü filtrelerini gerçek durumla başlat (sıfırdan başlasa ilk 3 adım "boş" sayılır: homing faz 3'ü
  // atlar, mikro switch basılıyken komut reddi gecikir). LIM_DEBOUNCE okumadan biri etkinse etkin: güvenli taraf.
#if END_SENSORS
  { bool aL = false, aR = false;
#if END_SENSORS == 2
    bool hL = false, hR = false;
#endif
    for (int i = 0; i < LIM_DEBOUNCE; i++) {
      aL |= digitalRead(PIN_REF_L) == REF_ACTIVE; aR |= digitalRead(PIN_REF_R) == REF_ACTIVE;
#if END_SENSORS == 2
      hL |= digitalRead(PIN_LIM_L) == LIM_PRESSED; hR |= digitalRead(PIN_LIM_R) == LIM_PRESSED;
#endif
      delayMicroseconds(200);
    }
    // *_prev da başlatılır (homing faz 3 kararı ilk adımda doğru olsun); açılışta etkin sensörün olayları
    // (limit, sensor_mismatch) burada bir kez gönderilir, sanal ESP32'deki gibi
    fL.seed(aL); fR.seed(aR); limL_prev = aL; limR_prev = aR;
    if (aL) SET_SHARED(ev_limitL, true);
    if (aR) SET_SHARED(ev_limitR, true);
#if END_SENSORS == 2
    fHL.seed(hL); fHR.seed(hR); hardL_prev = hL; hardR_prev = hR;
    if (hL) { SET_SHARED(ev_hardL, true); if (!aL) SET_SHARED(ev_mismatchL, true); }
    if (hR) { SET_SHARED(ev_hardR, true); if (!aR) SET_SHARED(ev_mismatchR, true); }
    portENTER_CRITICAL(&mux); S.hardL = hL; S.hardR = hR; portEXIT_CRITICAL(&mux);
#endif
  }
#endif
#if USE_MPU6050
  bool mpu_ok = mpuInit();
  int mpu_fails = 0; float gyro_bias = 0.0f, gy = 0.0f;
  int mpu_bad = 0;                                     // tutarlılık denetimi sayacı
  // bias yalnız açı kalibrasyonu başarılıyken (sarkaç hareketsiz) alınır: sallanırken ölçülen bias birkaç rad/s
  // yanlış çıkar ve tutarlılık denetimi sağlam MPU'yu kapatırdı. Okuma başarısızsa yine devre dışı.
  if (mpu_ok) {
    vTaskDelay(pdMS_TO_TICKS(50));                     // uykudan çıkış ~30 ms
    float b; mpu_ok = calibrateGyro(b); if (mpu_ok && angle_ok) gyro_bias = b;
  }
  if (!mpu_ok) SET_SHARED(ev_gyro_fail, true);         // WHO_AM_I yanıtsız ya da bias okunamadı: MPU'suz devam
  portENTER_CRITICAL(&mux); S.mpu_ok = mpu_ok; S.gyro_bias = gyro_bias; portEXIT_CRITICAL(&mux);
#endif
  portENTER_CRITICAL(&mux); S.angle_ok = angle_ok; S.calib_done = true; S.ev_calib = true; portEXIT_CRITICAL(&mux);

  TickType_t last = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last, period);
    // ---------- komutları ve ayarları al (kısa kritik bölge)
    Shared p;
    portENTER_CRITICAL(&mux);
    p = S;
    S.req_start = S.req_stop = S.req_estop = S.req_home = S.req_calib = S.req_friction = false;
    portEXIT_CRITICAL(&mux);

    // sıra önemli: estop EN SON işlenir ve her şeyi ezer
    if (p.req_start && mode == IDLE && homed && angle_ok) mode = p.swingup_on ? SWING_UP : WAIT;
    if (p.req_home && (mode == IDLE || mode == FAULT)) {
#if END_SENSORS
      mode = HOMING; home_phase = 1; home_t0 = k * dt; home_x0 = X_SIGN * enc_count * X_PER_COUNT; home_i = 0;
      homed = false; SET_SHARED(homed, false);           // eski sıfır geçersiz: homing yarıda kalırsa start reddedilir
      home_sat = 0;
      home_r0 = limR_prev;                               // referans sensörün filtrelenmiş durumu (1'de switch, 2'de optik)
#if END_SENSORS == 2
      // optik zaten etkinse araba optik ile uç arasında, sıfır 3 cm'ye kadar kayardı: önce sağa çekilip optiği
      // bırak (HOME_BACKOFF_M daha), sonra kenara yeniden yaklaş (faz 3)
      if (limL_prev) { home_phase = 3; home_rel = NAN; }   // filtrelenmiş durum: tek boş okuma atlatmaz
#endif
#else
      // elle ortalama: araba elle ortaya konmuş ve durağan (iletişim görevi denetledi) → bu nokta x = 0
      int32_t c0; portENTER_CRITICAL(&mux); c0 = enc_count; portEXIT_CRITICAL(&mux);
      x_offset = X_SIGN * c0 * X_PER_COUNT; vx.reset(); mode = IDLE; homed = true;
      portENTER_CRITICAL(&mux); S.ev_homed = true; S.homed = true; portEXIT_CRITICAL(&mux);
#endif
    }
    if (p.req_calib && mode == IDLE) {
      angle_ok = calibrateAngle(raw_hang);              // motor kapalıyken ~130 ms bloklar
#if USE_MPU6050
      if (mpu_ok) {                                    // +~0.2 s; bias yalnız angle_ok iken güncellenir (yukarıdaki not)
        float b;
        if (!calibrateGyro(b)) { mpu_ok = false; SET_SHARED(ev_gyro_fail, true); }
        else if (angle_ok) gyro_bias = b;
      }
      portENTER_CRITICAL(&mux); S.mpu_ok = mpu_ok; S.gyro_bias = gyro_bias; portEXIT_CRITICAL(&mux);
#endif
      vth.reset(); kf.reset(); last = xTaskGetTickCount();
      portENTER_CRITICAL(&mux); S.angle_ok = angle_ok; S.ev_calib = true; portEXIT_CRITICAL(&mux);
    }
    if (p.req_friction && mode == IDLE && homed) { mode = FRICTION; fr.reset(); }
    if (p.req_stop) { mode = IDLE; home_phase = 0; }
    fc_N = p.fc_volt * MOTOR_ALPHA;
    if ((mode == HOMING || mode == SWING_UP || mode == WAIT || mode == BALANCE || mode == FRICTION)
        && millis() - p.last_rx_ms > LINK_TIMEOUT_MS) mode = raiseFault(mode, "link");   // PC sustu: USB koptu / arayüz çöktü
    if (p.req_estop) mode = raiseFault(mode, "estop");

    // ---------- sensörler
#if USE_MPU6050
    const uint32_t t_i2c = micros();
#endif
    if (readAS5600(raw)) sensor_fails = 0;
    else if (++sensor_fails >= SENSOR_FAIL_LIMIT && mode != IDLE) mode = raiseFault(mode, "sensor");
    float th = wrapAngle(ANGLE_SIGN * wrapAngle(rawToRad(raw) - raw_hang) + PI);
    int32_t cnt; portENTER_CRITICAL(&mux); cnt = enc_count; portEXIT_CRITICAL(&mux);
    float x_enc = X_SIGN * cnt * X_PER_COUNT;
    float x = x_enc - x_offset;
    float dx = vx.update(x, dt), dth = vth.update(th, dt);
#if USE_MPU6050
    if (mpu_ok) {
      float w;
      if (mpuReadGyro(w)) {
        mpu_fails = 0; gy = w - gyro_bias;
        mpu_bad = fabsf(gy - dth) > GYRO_CHECK_RAD_S ? mpu_bad + 1 : 0;   // dth: henüz karışımsız AS5600 türevi
        if (mpu_bad >= GYRO_CHECK_N) { mpu_ok = false; SET_SHARED(ev_gyro_fail, true); }
        uint8_t pwr = 0x01;
        if (mpu_ok && k % MPU_PWR_CHECK_N == 0 && (!i2cReadDev(MPU_ADDR, 0x6B, &pwr, 1) || pwr != 0x01)) {
          mpu_ok = false; SET_SHARED(ev_gyro_fail, true);   // resetlenmiş (uyku, ±250 °/s) ya da yanıtsız
        }
      }
      else if (++mpu_fails >= MPU_FAIL_LIMIT) { mpu_ok = false; SET_SHARED(ev_gyro_fail, true); }  // FAULT değil
    }
    {
      const uint32_t us = micros() - t_i2c;
      portENTER_CRITICAL(&mux); if (us > S.i2c_us) S.i2c_us = us; S.mpu_ok = mpu_ok; S.gy = gy; portEXIT_CRITICAL(&mux);
    }
    // türev bölgesinde karışım (Denklem 8.5); Kalman bölgesinde kf.update bunu kendi kestirimiyle değiştirir
    if (mpu_ok && p.gyro_mix > 0.0f) dth = p.gyro_mix * gy + (1.0f - p.gyro_mix) * dth;
#endif
    bool kf_en = USE_KALMAN && homed && (mode == SWING_UP || mode == WAIT || mode == BALANCE) && (fabsf(th) < p.release_rad);
    kf.update(x, dx, th, dth, u_prev, kf_en, dx, dth);
    // limL/limR: REFERANS sensör (1'de switch, 2'de optik): homing ve çalışırken "limit".
    // hardL/hardR: yalnız END_SENSORS 2'de dış kademe mikro switch.
#if END_SENSORS
    bool limL = fL.update(digitalRead(PIN_REF_L) == REF_ACTIVE);   // tek parazitli okuma ne tetikler ne bırakır
    bool limR = fR.update(digitalRead(PIN_REF_R) == REF_ACTIVE);
#else
    const bool limL = false, limR = false;               // switch yok: takılı olmayan pin "basılı" okunurdu
#endif
#if END_SENSORS == 2
    bool hardL = fHL.update(digitalRead(PIN_LIM_L) == LIM_PRESSED);
    bool hardR = fHR.update(digitalRead(PIN_LIM_R) == LIM_PRESSED);
#else
    const bool hardL = false, hardR = false;
#endif
    if (limL && !limL_prev) SET_SHARED(ev_limitL, true);
    if (limR && !limR_prev) SET_SHARED(ev_limitR, true);
    if (hardL && !hardL_prev) { SET_SHARED(ev_hardL, true); if (!limL) SET_SHARED(ev_mismatchL, true); }
    if (hardR && !hardR_prev) { SET_SHARED(ev_hardR, true); if (!limR) SET_SHARED(ev_mismatchR, true); }
    if (!limR) home_r0 = false;                          // sağ referans bırakıldı; artık basılması yön hatasıdır
    limL_prev = limL; limR_prev = limR; hardL_prev = hardL; hardR_prev = hardR;
    portENTER_CRITICAL(&mux); S.hardL = hardL; S.hardR = hardR; portEXIT_CRITICAL(&mux);

    // ---------- mod makinesi (supervisor.Supervisor.update + mock_device homing ile aynı)
    float u = 0;
    if (mode == HOMING) {
      if (k * dt - home_t0 > HOME_TIMEOUT_S)                   mode = raiseFault(mode, "homing_timeout");
      // END_SENSORS 2: dış kademe basıldı → optik kaçırıldı ya da çalışmıyor, araba ray sonunda
      else if (hardL || hardR)                                 mode = raiseFault(mode, "homing_overrun");
      // sağ switch: yeni basıldıysa ya da başta basılıydı ama 1 s sola gidince hâlâ bırakmadıysa → yön ters
      else if (limR && (!home_r0 || k * dt - home_t0 > 1.0f))  mode = raiseFault(mode, "homing_wrong_side");
      else if (fabsf(x_enc - home_x0) > 2 * RAIL_HALF_M + 0.1f) mode = raiseFault(mode, "homing_travel");
      // motor kabloları ya da X_SIGN ters: hız döngüsü pozitif geri beslemeye döner ve araba tam gerilimle
      // uca koşar. Hız beklenenin 2 katını aşarsa ya da gerilim uzun süre doyumdaysa çarpmadan dur.
      else if ((k * dt - home_t0 > 0.2f && fabsf(dx) > HOME_RUNAWAY_V) || home_sat > HOME_SAT_S)
        mode = raiseFault(mode, "homing_runaway");   // ilk 0.2 s hız kestirimi oturuyor
      // sol sensör hep "basılı" okunuyor (kopuk kablo, NO/NC karışıklığı): sıfır yanlış olurdu
      else if (home_phase == 2 && limL && x > -RAIL_HALF_M + REF_FROM_END_M + HOME_SW_CLEAR_M) mode = raiseFault(mode, "homing_switch");
      // faz 3: optikten geri çekilirken HOME_SW_CLEAR_M gidildiği hâlde optik bırakmıyor → takılı/kopuk
      else if (home_phase == 3 && limL && x_enc - home_x0 > HOME_SW_CLEAR_M) mode = raiseFault(mode, "homing_switch");
      else if (home_phase == 3 && limL && !isnan(home_rel)) home_rel = NAN;            // optik yeniden etkin: baştan say
      else if (home_phase == 3 && !limL && isnan(home_rel)) home_rel = x_enc;
      else if (home_phase == 3 && !limL && x_enc - home_rel >= HOME_BACKOFF_M) { home_phase = 1; home_i = 0; }
      else if (home_phase == 1 && limL) {                       // switch SEVİYESİ: zaten basılıysa da çalışır
        // referans sensörün konumu: 1'de −RAIL_HALF_M, 2'de optik (OPT_FROM_END_M içeride)
        x_offset = x_enc + RAIL_HALF_M - REF_FROM_END_M; x = -RAIL_HALF_M + REF_FROM_END_M; vx.reset(); kf.reset(); home_phase = 2; home_i = 0;
      } else {
        float v_des = home_phase == 1 ? -HOME_SPEED : home_phase == 3 ? HOME_SPEED : clampf(-2.0f * x, -0.15f, 0.15f);
        if (home_phase == 2 && fabsf(x) < 0.002f && fabsf(dx) < 0.02f) {
          mode = IDLE; home_phase = 0; homed = true;
          portENTER_CRITICAL(&mux); S.ev_homed = true; S.homed = true; portEXIT_CRITICAL(&mux);
        } else {
          float e = v_des - dx;
          home_i = clampf(home_i + HOME_KI * e * dt, -HOME_I_MAX, HOME_I_MAX);
          u = accelToVolt(dx, th, dth, HOME_KP * e + home_i, false);   // sürtünmeyi integral karşılar (#30)
          home_sat = fabsf(u) >= 0.98f * p.u_max ? home_sat + dt : 0.0f;
        }
      }
    } else if (mode == FRICTION) {
      if (hardL || hardR)                            mode = raiseFault(mode, "hard_limit");
      else if (limL || limR)                         mode = raiseFault(mode, "limit");
      else if (fabsf(x) > RAIL_HALF_M - SOFT_MARGIN) mode = raiseFault(mode, "rail");
      else if (fr.step(x, th, dt, p.u_max, u)) {
        mode = IDLE;
        portENTER_CRITICAL(&mux);
        S.ev_friction = true; S.fr_ok = fr.ok; S.fr_reason = fr.reason;
        S.fr_v_pos = fr.v_pos; S.fr_v_neg = fr.v_neg; S.fr_v_c = fr.v_c; S.fr_slope = fr.slope;
        portEXIT_CRITICAL(&mux);
      }
    } else if (mode == SWING_UP || mode == BALANCE || mode == WAIT) {
      float ath = fabsf(th);
      if (hardL || hardR)                            mode = raiseFault(mode, "hard_limit");
      else if (limL || limR)                         mode = raiseFault(mode, "limit");
      else if (fabsf(x) > RAIL_HALF_M - SOFT_MARGIN) mode = raiseFault(mode, "rail");
      else {
        if (mode == BALANCE && ath > p.release_rad) mode = p.swingup_on ? SWING_UP : WAIT;
        else if (mode != BALANCE && ath < p.catch_rad && fabsf(dth) < p.catch_dth
                 && fabsf(x) < p.catch_x && fabsf(dx) < p.catch_dx) mode = BALANCE;   // Supervisor.can_catch
        else if (mode != BALANCE) mode = p.swingup_on ? SWING_UP : WAIT;      // swing-up açık/kapalı hemen uygulanır
        if (mode == BALANCE)
          u = -(p.K[0] * (x - p.x_ref) + p.K[1] * dx + p.K[2] * th + p.K[3] * dth);
        else if (mode == SWING_UP)
          u = accelToVolt(dx, th, dth, swingUpAccel(p, x, dx, th, dth), true);
      }
    }
    if (mode == IDLE || mode == FAULT) u = 0;
    u = clampf(u, -p.u_max, p.u_max);
    // sürücü yalnız hareket modlarında etkin; IDLE'da araba elle itilebilir (işaret kontrolü için)
    digitalWrite(PIN_EN, (mode == IDLE || mode == FAULT) ? LOW : HIGH);
    motorWrite(u);
    u_prev = u;

    // ---------- telemetri anlık görüntüsü
    k++;
    tel_acc += telemetry_hz * dt;
    portENTER_CRITICAL(&mux);
    S.mode = mode;                                     // komut doğrulaması için her adımda güncel
    S.lx = x; S.ldx = dx; S.lth = th; S.ldth = dth;
    if (tel_acc >= 1.0f) {
      tel_acc -= 1.0f;
      S.n = n++; S.t = k * dt; S.x = x; S.dx = dx; S.th = th; S.dth = dth; S.u = u; S.tel_valid = true;
      S.tel_gy = S.gy;                                 // telemetri örneğiyle aynı an (USE_MPU6050 0'da kullanılmaz)
    }
    portEXIT_CRITICAL(&mux);
  }
}

// ============================================================ İLETİŞİM GÖREVİ
// Yanıt: {"ack":"<cmd>","ok":..,"msg":".."}. cmd ve msg JSON'u bozmasın diye süzülür.
static void sendAck(const char *cmd, bool ok, const char *msg = nullptr, const char *extra = nullptr) {
  char c[20]; size_t j = 0;
  for (const char *s = cmd; *s && j < sizeof(c) - 1; s++)
    if ((*s >= 'a' && *s <= 'z') || *s == '_') c[j++] = *s;
  c[j] = 0;
  if (!j) strcpy(c, "?");
  char buf[192];
  int len = snprintf(buf, sizeof buf, "{\"ack\":\"%s\",\"ok\":%s%s%s%s%s}\n", c, ok ? "true" : "false",
                     msg ? ",\"msg\":\"" : "", msg ? msg : "", msg ? "\"" : "", extra ? extra : "");
  if (len < 0) return;
  if (len >= (int)sizeof buf)                          // olmamalı; yine de geçerli JSON gönder
    len = snprintf(buf, sizeof buf, "{\"ack\":\"%s\",\"ok\":%s}\n", c, ok ? "true" : "false");
  Serial.write((const uint8_t *)buf, len);
}

static const ParamLimit *findParam(const char *key) {
  for (const auto &pl : PARAM_LIMITS) if (!strcmp(pl.key, key)) return &pl;
  return nullptr;
}

static void handleSetParam(JsonObject d) {
  Shared cur; portENTER_CRITICAL(&mux); cur = S; portEXIT_CRITICAL(&mux);
  float catch_deg = degrees(cur.catch_rad), release_deg = degrees(cur.release_rad);
  // 1) hepsini doğrula — biri bile geçersizse hiçbiri uygulanmaz
  for (JsonPair kv : d) {
    const char *key = kv.key().c_str();
    if (!strcmp(key, "cmd")) continue;
    const ParamLimit *pl = findParam(key);
    if (!pl) { sendAck("set_param", false, "bilinmeyen parametre"); return; }
    if (!kv.value().is<float>()) { sendAck("set_param", false, "sayı değil"); return; }
    float v = kv.value().as<float>();
    if (!(v >= pl->lo && v <= pl->hi)) { sendAck("set_param", false, "aralık dışı"); return; }
    if (!strcmp(key, "catch_deg")) catch_deg = v;
    if (!strcmp(key, "release_deg")) release_deg = v;
    // 1e-4: PC sınıra kırptığı değeri gönderir; float32 yuvarlaması sınırı birkaç ulp aşabilir (ör. ray 0.302 m)
    if (!strcmp(key, "x_ref") && fabsf(v) > RAIL_HALF_M - 0.1f + 1e-4f) { sendAck("set_param", false, "x_ref ray sınırına çok yakın"); return; }
    if (!strcmp(key, "u_max") && v > V_SUPPLY) { sendAck("set_param", false, "u_max besleme gerilimini aşamaz"); return; }
  }
  if (catch_deg >= release_deg) { sendAck("set_param", false, "catch_deg < release_deg olmalı"); return; }
  // 2) uygula
  portENTER_CRITICAL(&mux);
  S.catch_rad = radians(catch_deg); S.release_rad = radians(release_deg);
  if (d["catch_dth"].is<float>()) S.catch_dth = d["catch_dth"].as<float>();
  if (d["catch_x"].is<float>())   S.catch_x   = d["catch_x"].as<float>();
  if (d["catch_dx"].is<float>())  S.catch_dx  = d["catch_dx"].as<float>();
  if (d["E_ratio"].is<float>())   S.su_eratio = d["E_ratio"].as<float>();
  if (d["swingup"].is<float>()) S.swingup_on = d["swingup"].as<float>() != 0;
  if (d["k_E"].is<float>())   S.su_kE   = d["k_E"].as<float>();
  if (d["a_max"].is<float>()) S.su_amax = d["a_max"].as<float>();
  if (d["k_x"].is<float>())   S.su_kx   = d["k_x"].as<float>();
  if (d["k_v"].is<float>())   S.su_kv   = d["k_v"].as<float>();
  if (d["x_ref"].is<float>()) S.x_ref   = d["x_ref"].as<float>();
  if (d["u_max"].is<float>()) S.u_max   = d["u_max"].as<float>();
  if (d["fc_volt"].is<float>()) S.fc_volt = d["fc_volt"].as<float>();
  if (d["gyro_mix"].is<float>()) S.gyro_mix = d["gyro_mix"].as<float>();   // USE_MPU6050 0'da etkisiz
  portEXIT_CRITICAL(&mux);
  if (d["telemetry_hz"].is<float>()) telemetry_hz = (int)d["telemetry_hz"].as<float>();
  sendAck("set_param", true);
}

static void handleCommand(const char *line) {
  JsonDocument d;
  if (deserializeJson(d, line)) { sendAck("?", false, "json"); return; }
  const char *cmd = d["cmd"] | "";
  Shared s; portENTER_CRITICAL(&mux); s = S; portEXIT_CRITICAL(&mux);
  bool moves = !strcmp(cmd, "start") || !strcmp(cmd, "home") || !strcmp(cmd, "friction");
  // açılış kalibrasyonu (~1.1 s) bitmeden kontrol döngüsü çalışmıyor, uç sensörleri henüz okunmadı
  if (moves && !s.calib_done) { sendAck(cmd, false, "açılış kalibrasyonu sürüyor: biraz bekleyin"); return; }
  // END_SENSORS 2: mikro switch basılıyken hareket başlatan komutlar her modda reddedilir (mock_device ile aynı)
  if ((s.hardL || s.hardR) && (!strcmp(cmd, "start") || !strcmp(cmd, "home") || !strcmp(cmd, "friction"))) {
    sendAck(cmd, false, s.hardL ? "sol mikro switch basılı: arabayı elle uçtan uzaklaştırın"
                                : "sağ mikro switch basılı: arabayı elle uçtan uzaklaştırın");
    return;
  }

  if (!strcmp(cmd, "hb")) return;                      // bağlantı bekçisi: yanıtsız (zamanı commTask kaydetti)
  if (!strcmp(cmd, "ping")) {
    char extra[144];                                   // angle_ok: açılıştaki calib olayını kaçıran PC de görsün
    // açılış kalibrasyonu (~1.1 s) bitmeden angle_ok gönderilmez (yanlış "başarısız" uyarısı olmasın)
    snprintf(extra, sizeof extra, ",\"Ts\":%.4f,\"fw\":\"%s\",\"theta\":\"sol\",\"homed\":%s%s", CTRL_PERIOD_MS / 1000.0f,
             FW_NAME, s.homed ? "true" : "false",
             !s.calib_done ? "" : s.angle_ok ? ",\"angle_ok\":true" : ",\"angle_ok\":false");
#if USE_MPU6050
    if (s.calib_done) {   // jiroskop durumu ve en büyük I²C okuma süresi (µs); açılış kalibrasyonundan önce yok
      size_t L = strlen(extra);
      snprintf(extra + L, sizeof extra - L, ",\"gyro\":%s,\"i2c_us\":%lu", s.mpu_ok ? "true" : "false",
               (unsigned long)s.i2c_us);
    }
#endif
    sendAck("ping", true, "pong", extra); return;
  }
  if (!strcmp(cmd, "set_gain")) {
    JsonArray K = d["K"];
    if (K.size() != 4) { sendAck("set_gain", false, "K 4 eleman olmalı"); return; }
    float k[4];
    for (int i = 0; i < 4; i++) { k[i] = K[i].as<float>(); if (!isfinite(k[i])) { sendAck("set_gain", false, "sonlu değil"); return; } }
    portENTER_CRITICAL(&mux); memcpy(S.K, k, sizeof k); portEXIT_CRITICAL(&mux);
    sendAck("set_gain", true); return;
  }
  if (!strcmp(cmd, "set_param")) { handleSetParam(d.as<JsonObject>()); return; }
  // kabul/ret kuralları mock_device.MockDevice._handle ile aynı
  if (!strcmp(cmd, "start")) {
    if (!s.homed)          { sendAck("start", false, "önce home (kalibrasyon) yapılmalı"); return; }
    if (!s.angle_ok)       { sendAck("start", false, "açı kalibrasyonu başarısız: sarkacı aşağıda sabit tutup calib gönderin"); return; }
    if (s.mode == FAULT)   { sendAck("start", false, "hata modunda: önce stop"); return; }
    if (s.mode != IDLE)    { sendAck("start", false, "IDLE dışında başlatılamaz"); return; }
    if (fabsf(s.lx) > RAIL_HALF_M - START_X_MARGIN) { sendAck("start", false, "araba ray ucuna çok yakın: KALİBRE ET ile ortaya alın"); return; }
    if (fabsf(s.ldx) > START_MAX_SPEED) { sendAck("start", false, "araba hâlâ hareket ediyor: durmasını bekleyin"); return; }
    SET_SHARED(req_start, true); sendAck("start", true); return;
  }
  if (!strcmp(cmd, "friction")) {                      // control/friction.py::start_block_reason ile aynı
    if (!s.homed)          { sendAck("friction", false, "önce home (kalibrasyon) yapılmalı"); return; }
    if (!s.angle_ok)       { sendAck("friction", false, "açı kalibrasyonu başarısız: sarkacı aşağıda sabit tutup calib gönderin"); return; }
    if (s.mode != IDLE)    { sendAck("friction", false, "yalnız IDLE modunda"); return; }
    if (fabsf(s.lx) > RAIL_HALF_M - FR_TRAVEL - FR_X_MARGIN) { sendAck("friction", false, "araba ortada değil: önce KALİBRE ET"); return; }
    if (fabsf(s.ldx) > FR_STILL_DX) { sendAck("friction", false, "araba hareket ediyor: durmasını bekleyin"); return; }
    if (fabsf(PI - fabsf(wrapAngle(s.lth))) > FR_SWING_MAX || fabsf(s.ldth) > FR_STILL_DTH) {
      sendAck("friction", false, "sarkaç aşağıda ve hareketsiz olmalı"); return;
    }
    SET_SHARED(req_friction, true); sendAck("friction", true); return;
  }
  if (!strcmp(cmd, "home")) {
    if (s.mode != IDLE && s.mode != FAULT) { sendAck("home", false, "önce stop"); return; }
#if !END_SENSORS
    if (fabsf(s.ldx) > HOME_STILL_DX) { sendAck("home", false, "araba hareket ediyor: elle ortaya koyup durunca tekrar deneyin"); return; }
#endif
    SET_SHARED(req_home, true); sendAck("home", true); return;
  }
  if (!strcmp(cmd, "calib")) {
    if (s.mode != IDLE) { sendAck("calib", false, "yalnız IDLE modunda, sarkaç aşağıda ve hareketsizken"); return; }
    SET_SHARED(req_calib, true); sendAck("calib", true); return;
  }
  if (!strcmp(cmd, "stop"))  { SET_SHARED(req_stop, true);  sendAck("stop", true);  return; }
  if (!strcmp(cmd, "estop")) { SET_SHARED(req_estop, true); sendAck("estop", true); return; }
  sendAck(cmd, false, "desteklenmiyor");
}

void commTask(void *) {
  char line[256]; size_t len = 0; uint32_t last_n = UINT32_MAX; bool overflow = false;
  char buf[200];
  for (;;) {
    while (Serial.available()) {                        // komutlar
      char ch = Serial.read();
      if (ch == '\n') {
        line[len] = 0;
        if (len || overflow) SET_SHARED(last_rx_ms, millis());   // bağlantı bekçisi: PC canlı
        if (overflow) sendAck("?", false, "satır çok uzun");
        else if (len) handleCommand(line);
        len = 0; overflow = false;
      } else if (len < sizeof(line) - 1) line[len++] = ch;
      else overflow = true;                             // taşma: satırın sonuna kadar at
    }
    Shared s;
    portENTER_CRITICAL(&mux);
    s = S; S.ev_homed = S.ev_fault = S.ev_limitL = S.ev_limitR = S.ev_calib = S.ev_friction = false;
    S.ev_hardL = S.ev_hardR = S.ev_mismatchL = S.ev_mismatchR = false; S.ev_gyro_fail = false;
    portEXIT_CRITICAL(&mux);
#if END_SENSORS == 2
    if (s.ev_limitL) Serial.print("{\"evt\":\"limit\",\"side\":\"L\",\"stage\":\"opt\"}\n");
    if (s.ev_limitR) Serial.print("{\"evt\":\"limit\",\"side\":\"R\",\"stage\":\"opt\"}\n");
    if (s.ev_hardL)  Serial.print("{\"evt\":\"limit\",\"side\":\"L\",\"stage\":\"hard\"}\n");
    if (s.ev_hardR)  Serial.print("{\"evt\":\"limit\",\"side\":\"R\",\"stage\":\"hard\"}\n");
    if (s.ev_mismatchL) Serial.print("{\"evt\":\"sensor_mismatch\",\"side\":\"L\"}\n");
    if (s.ev_mismatchR) Serial.print("{\"evt\":\"sensor_mismatch\",\"side\":\"R\"}\n");
#else
    if (s.ev_limitL) Serial.print("{\"evt\":\"limit\",\"side\":\"L\"}\n");
    if (s.ev_limitR) Serial.print("{\"evt\":\"limit\",\"side\":\"R\"}\n");
#endif
    if (s.ev_homed)  Serial.print("{\"evt\":\"homed\"}\n");
#if USE_MPU6050
    if (s.ev_calib) {
      if (s.mpu_ok) Serial.printf("{\"evt\":\"calib\",\"ok\":%s,\"gyro_bias\":%.5f}\n", s.angle_ok ? "true" : "false", s.gyro_bias);
      else          Serial.printf("{\"evt\":\"calib\",\"ok\":%s}\n", s.angle_ok ? "true" : "false");
    }
    if (s.ev_gyro_fail) Serial.print("{\"evt\":\"gyro\",\"ok\":false}\n");
#else
    if (s.ev_calib)  Serial.printf("{\"evt\":\"calib\",\"ok\":%s}\n", s.angle_ok ? "true" : "false");
#endif
    if (s.ev_fault)  Serial.printf("{\"evt\":\"fault\",\"reason\":\"%s\"}\n", s.fault_reason ? s.fault_reason : "?");
    if (s.ev_friction) {
      if (s.fr_ok) Serial.printf("{\"evt\":\"friction\",\"ok\":true,\"v_pos\":%.4f,\"v_neg\":%.4f,\"v_c\":%.4f,\"slope\":%.4f}\n",
                                 s.fr_v_pos, s.fr_v_neg, s.fr_v_c, s.fr_slope);
      else         Serial.printf("{\"evt\":\"friction\",\"ok\":false,\"reason\":\"%s\"}\n", s.fr_reason ? s.fr_reason : "?");
    }
    if (s.tel_valid && s.n != last_n) {                 // yeni telemetri (açılıştaki boş yapı gönderilmez)
      last_n = s.n;
#if USE_MPU6050
      int l = s.mpu_ok
        ? snprintf(buf, sizeof buf,
            "{\"n\":%lu,\"t\":%.4f,\"x\":%.5f,\"dx\":%.4f,\"th\":%.5f,\"dth\":%.4f,\"u\":%.3f,\"mode\":\"%s\",\"gy\":%.4f}\n",
            (unsigned long)s.n, s.t, s.x, s.dx, s.th, s.dth, s.u, MODE_NAMES[s.mode], s.tel_gy)
        : snprintf(buf, sizeof buf,
            "{\"n\":%lu,\"t\":%.4f,\"x\":%.5f,\"dx\":%.4f,\"th\":%.5f,\"dth\":%.4f,\"u\":%.3f,\"mode\":\"%s\"}\n",
            (unsigned long)s.n, s.t, s.x, s.dx, s.th, s.dth, s.u, MODE_NAMES[s.mode]);
#else
      int l = snprintf(buf, sizeof buf,
        "{\"n\":%lu,\"t\":%.4f,\"x\":%.5f,\"dx\":%.4f,\"th\":%.5f,\"dth\":%.4f,\"u\":%.3f,\"mode\":\"%s\"}\n",
        (unsigned long)s.n, s.t, s.x, s.dx, s.th, s.dth, s.u, MODE_NAMES[s.mode]);
#endif
      if (l > 0 && l < (int)sizeof buf) Serial.write((const uint8_t *)buf, l);
    }
    vTaskDelay(1);
  }
}

// ============================================================ KURULUM
void setup() {
  pinMode(PIN_EN, OUTPUT); digitalWrite(PIN_EN, LOW);   // önce sürücüyü kapat
  Serial.begin(921600);
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  Wire.setTimeOut(2);                                   // ms — takılan I2C döngüyü bekletmesin
  pinMode(PIN_ENC_A, INPUT); pinMode(PIN_ENC_B, INPUT);
#if END_SENSORS
  pinMode(PIN_LIM_L, INPUT_PULLUP); pinMode(PIN_LIM_R, INPUT_PULLUP);
#endif
#if END_SENSORS == 2
  pinMode(PIN_OPT_L, INPUT_PULLUP); pinMode(PIN_OPT_R, INPUT_PULLUP);   // TCST2103 çıkışı (harici pull-up önerilir)
#endif
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_RPWM, PWM_FREQ, PWM_BITS); ledcAttach(PIN_LPWM, PWM_FREQ, PWM_BITS);
#else
  ledcSetup(0, PWM_FREQ, PWM_BITS); ledcSetup(1, PWM_FREQ, PWM_BITS);
  ledcAttachPin(PIN_RPWM, 0); ledcAttachPin(PIN_LPWM, 1);
#endif
  motorWrite(0);
  enc_prev = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encISR, CHANGE);

  memset(&S, 0, sizeof S);
  memcpy(S.K, LQR_K, sizeof S.K);
  S.catch_rad = CATCH_RAD; S.release_rad = RELEASE_RAD; S.u_max = U_MAX; S.swingup_on = true;
  S.su_kE = SU_K_E; S.su_amax = SU_A_MAX; S.su_kx = SU_K_X; S.su_kv = SU_K_V; S.su_akick = SU_A_KICK;
  S.su_eratio = SU_E_RATIO; S.catch_dth = CATCH_DTH; S.catch_x = CATCH_X; S.catch_dx = CATCH_DX;
  S.fc_volt = P_FC / MOTOR_ALPHA;                     // gains.h'deki telafi, gerilim olarak
  S.mode = IDLE;

  Serial.print("\n");   // ROM'un 115200 baud açılış mesajı 921600'de çöp görünür: boot satırına yapışmasın
  Serial.printf("{\"evt\":\"boot\",\"fw\":\"%s\",\"theta\":\"sol\",\"Ts\":%.4f}\n", FW_NAME, CTRL_PERIOD_MS / 1000.0f);
  xTaskCreatePinnedToCore(ctrlTask, "ctrl", 4096, nullptr, configMAX_PRIORITIES - 1, nullptr, 1);
  xTaskCreatePinnedToCore(commTask, "comm", 6144, nullptr, 2, nullptr, 0);
}

void loop() { vTaskDelete(nullptr); }   // tüm iş görevlerde
