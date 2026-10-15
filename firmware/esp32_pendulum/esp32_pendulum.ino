/*
 * Ters Sarkaç — ESP32 REFERANS firmware iskeleti (Arduino-ESP32 core 2.x / 3.x)
 * ---------------------------------------------------------------------------
 * Bilgisayar ekibi tarafından protokolü ve kontrol mantığını tanımlamak için yazıldı.
 * Mekatronik ekibi pinleri, motor sürücüsünü ve sensör yönlerini kendi donanımına göre
 * düzenlemelidir. BU DOSYA HENÜZ GERÇEK KARTTA DERLENİP DENENMEDİ.
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
#if !defined(CTRL_PERIOD_MS) || !defined(MOTOR_V_MAX)
#error "gains.h eski: CTRL_PERIOD_MS / MOTOR_V_MAX yok. Arayüzden yeniden dışa aktarın."
#endif
#ifndef INPUT_MODE_VOLTAGE
#error "Bu firmware yalnız DC motor (input_mode = voltage) için yazıldı. Step motor için motor sürme kısmı yeniden yazılmalı."
#endif
#if configTICK_RATE_HZ != 1000
#error "FreeRTOS tik hızı 1000 Hz olmalı (Arduino-ESP32 varsayılanı)."
#endif

// ============================================================ PİNLER (donanıma göre düzenleyin)
static const int PIN_SDA = 21, PIN_SCL = 22;          // AS5600
static const int PIN_ENC_A = 32, PIN_ENC_B = 33;      // motor enkoderi (10k pull-up harici)
static const int PIN_RPWM = 25, PIN_LPWM = 26;        // BTS7960
static const int PIN_EN = 27;                         // BTS7960 R_EN + L_EN birlikte
static const int PIN_LIM_L = 18, PIN_LIM_R = 19;      // limit switch. NC (normalde kapalı) önerilir:
static const int LIM_PRESSED = HIGH;                  //   NC + INPUT_PULLUP → basılı/kopuk kablo = HIGH (güvenli)
                                                      //   NO kullanılırsa LOW yapın (kopuk kablo fark edilmez!)

// ============================================================ DONANIM SABİTLERİ
static const char *FW_NAME = "ref-1.1";
static const uint8_t AS5600_ADDR = 0x36;
static const float ANGLE_SIGN = +1.0f;                // sarkaç +x yönüne yatınca θ artmıyorsa -1 yapın
static const float X_SIGN = +1.0f;                    // araba sağa giderken x artmıyorsa -1 yapın
static const float ENC_COUNTS_PER_REV = 64.0f * 10.0f; // 37D: 64 CPR (4x) × redüktör
static const float PULLEY_R = 0.0127f;                // [m]
static const float X_PER_COUNT = 2.0f * PI * PULLEY_R / ENC_COUNTS_PER_REV;
static const float V_SUPPLY = MOTOR_V_MAX;            // arayüzdeki motor.V_max ile aynı (gains.h)
static const int   PWM_FREQ = 20000, PWM_BITS = 10;   // 20 kHz (duyulmaz), 0..1023
static const float VEL_FC_HZ = 25.0f;                 // hız kestirim filtresi (Python ile aynı)
static const float SOFT_MARGIN = 0.03f;               // rayın ucundan bu kadar önce FAULT [m]
static const float HOME_SPEED = 0.10f;                // homing hızı [m/s]
static const float HOME_TIMEOUT_S = 20.0f;
static const int   SENSOR_FAIL_LIMIT = 3;             // art arda bu kadar okuma hatası → FAULT
static const float CALIB_MAX_SPREAD = 0.02f;          // kalibrasyonda izin verilen sapma [rad] (~1.1°)

// ============================================================ set_param TABLOSU
// pendulum_lab/comm/protocol.py::SET_PARAM_LIMITS ile AYNI olmalı (test bunu denetler).
struct ParamLimit { const char *key; float lo, hi; };
static const ParamLimit PARAM_LIMITS[] = {
  {"catch_deg", 1.0f, 45.0f},
  {"release_deg", 2.0f, 90.0f},
  {"swingup", 0.0f, 1.0f},
  {"k_E", 0.0f, 10000.0f},
  {"a_max", 0.0f, 50.0f},
  {"k_x", 0.0f, 1000.0f},
  {"k_v", 0.0f, 1000.0f},
  {"x_ref", -0.3f, 0.3f},
  {"u_max", 0.0f, 48.0f},
  {"telemetry_hz", 1.0f, 500.0f},
};

// ============================================================ MODLAR (Python Mode ile aynı adlar)
enum Mode : uint8_t { IDLE, HOMING, WAIT, SWING_UP, BALANCE, FAULT };
static const char *MODE_NAMES[] = {"IDLE", "HOMING", "WAIT", "SWING_UP", "BALANCE", "FAULT"};

// ============================================================ PAYLAŞILAN DURUM (her erişim mux içinde)
struct Shared {
  // kontrol → iletişim
  uint32_t n; float t, x, dx, th, dth, u; Mode mode;
  bool homed, angle_ok;
  // iletişim → kontrol
  float K[4]; float catch_rad, release_rad, x_ref, u_max;
  float su_kE, su_amax, su_kx, su_kv, su_akick; bool swingup_on;
  bool req_start, req_stop, req_estop, req_home, req_calib;
  // olaylar (kontrol üretir, iletişim gönderir)
  bool ev_homed, ev_fault, ev_limitL, ev_limitR, ev_calib;
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
static float accelToVolt(float dx, float th, float dth, float a) {
  float s = sinf(th), c = cosf(th), ml = P_M_POLE * P_L;
  float ddth = (ml * P_G * s - P_C * dth - ml * c * a) / P_J;
  float F = (P_M_CART + P_M_POLE) * a + ml * c * ddth - ml * s * dth * dth + P_B * dx;
  return (F + MOTOR_BETA * dx) / MOTOR_ALPHA;
}

// Enerji bazlı swing-up (swingup.EnergySwingUp ile aynı)
static float swingUpAccel(const Shared &p, float x, float dx, float th, float dth) {
  float E = 0.5f * P_J * dth * dth + P_M_POLE * P_G * P_L * (cosf(th) - 1.0f);
  float a;
  if (fabsf(dth) < 0.05f && fabsf(wrapAngle(th)) > 2.8f && E < -0.99f * 2 * P_M_POLE * P_G * P_L)
    a = p.su_akick * (x > 1e-3f ? -1.0f : 1.0f);
  else
    a = p.su_kE * E * dth * cosf(th);
  a = clampf(a, -p.su_amax, p.su_amax);
  a += -p.su_kx * x - p.su_kv * dx;
  return clampf(a, -p.su_amax, p.su_amax);
}

// ============================================================ KONTROL GÖREVİ
static Mode raiseFault(Mode m, const char *reason) {
  if (m != FAULT) {
    portENTER_CRITICAL(&mux); S.ev_fault = true; S.fault_reason = reason; portEXIT_CRITICAL(&mux);
  }
  return FAULT;
}

void ctrlTask(void *) {
  const float dt = CTRL_PERIOD_MS / 1000.0f;
  const TickType_t period = pdMS_TO_TICKS(CTRL_PERIOD_MS);
  VelEst vx, vth; vx.setup(dt, VEL_FC_HZ, false); vth.setup(dt, VEL_FC_HZ, true);
  Mode mode = IDLE; bool homed = false; int home_phase = 0;
  float x_offset = 0, home_t0 = 0, home_x0 = 0; uint32_t k = 0, n = 0; float tel_acc = 0;
  bool home_r0 = false;                                // homing başlarken sağ switch basılı mıydı
  bool limL_prev = false, limR_prev = false;
  int sensor_fails = 0; uint16_t raw = 0;

  // açılış kalibrasyonu: sarkaç aşağıda ve hareketsiz olmalı → o konum θ = π
  vTaskDelay(pdMS_TO_TICKS(1000));
  float raw_hang = 0;
  bool angle_ok = calibrateAngle(raw_hang);
  portENTER_CRITICAL(&mux); S.angle_ok = angle_ok; S.ev_calib = true; portEXIT_CRITICAL(&mux);

  TickType_t last = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last, period);
    // ---------- komutları ve ayarları al (kısa kritik bölge)
    Shared p;
    portENTER_CRITICAL(&mux);
    p = S;
    S.req_start = S.req_stop = S.req_estop = S.req_home = S.req_calib = false;
    portEXIT_CRITICAL(&mux);

    // sıra önemli: estop EN SON işlenir ve her şeyi ezer
    if (p.req_start && mode == IDLE && homed && angle_ok) mode = p.swingup_on ? SWING_UP : WAIT;
    if (p.req_home && (mode == IDLE || mode == FAULT)) {
      mode = HOMING; home_phase = 1; home_t0 = k * dt; home_x0 = X_SIGN * enc_count * X_PER_COUNT;
      home_r0 = digitalRead(PIN_LIM_R) == LIM_PRESSED;
    }
    if (p.req_calib && mode == IDLE) {
      angle_ok = calibrateAngle(raw_hang);              // motor kapalıyken ~130 ms bloklar
      vth.reset(); last = xTaskGetTickCount();
      portENTER_CRITICAL(&mux); S.angle_ok = angle_ok; S.ev_calib = true; portEXIT_CRITICAL(&mux);
    }
    if (p.req_stop) { mode = IDLE; home_phase = 0; }
    if (p.req_estop) mode = raiseFault(mode, "estop");

    // ---------- sensörler
    if (readAS5600(raw)) sensor_fails = 0;
    else if (++sensor_fails >= SENSOR_FAIL_LIMIT && mode != IDLE) mode = raiseFault(mode, "sensor");
    float th = wrapAngle(ANGLE_SIGN * wrapAngle(rawToRad(raw) - raw_hang) + PI);
    int32_t cnt; portENTER_CRITICAL(&mux); cnt = enc_count; portEXIT_CRITICAL(&mux);
    float x_enc = X_SIGN * cnt * X_PER_COUNT;
    float x = x_enc - x_offset;
    float dx = vx.update(x, dt), dth = vth.update(th, dt);
    bool limL = digitalRead(PIN_LIM_L) == LIM_PRESSED, limR = digitalRead(PIN_LIM_R) == LIM_PRESSED;
    if (limL && !limL_prev) SET_SHARED(ev_limitL, true);
    if (limR && !limR_prev) SET_SHARED(ev_limitR, true);
    if (!limR) home_r0 = false;                          // sağ switch bırakıldı; artık basılması yön hatasıdır
    limL_prev = limL; limR_prev = limR;

    // ---------- mod makinesi (supervisor.Supervisor.update + mock_device homing ile aynı)
    float u = 0;
    if (mode == HOMING) {
      if (k * dt - home_t0 > HOME_TIMEOUT_S)                   mode = raiseFault(mode, "homing_timeout");
      // sağ switch: yeni basıldıysa ya da başta basılıydı ama 1 s sola gidince hâlâ bırakmadıysa → yön ters
      else if (limR && (!home_r0 || k * dt - home_t0 > 1.0f))  mode = raiseFault(mode, "homing_wrong_side");
      else if (fabsf(x_enc - home_x0) > 2 * RAIL_HALF_M + 0.1f) mode = raiseFault(mode, "homing_travel");
      else if (home_phase == 1 && limL) {                       // switch SEVİYESİ: zaten basılıysa da çalışır
        x_offset = x_enc + RAIL_HALF_M; x = -RAIL_HALF_M; vx.reset(); home_phase = 2;
      } else {
        float v_des = home_phase == 1 ? -HOME_SPEED : clampf(-2.0f * x, -0.15f, 0.15f);
        if (home_phase == 2 && fabsf(x) < 0.002f && fabsf(dx) < 0.02f) {
          mode = IDLE; home_phase = 0; homed = true;
          portENTER_CRITICAL(&mux); S.ev_homed = true; S.homed = true; portEXIT_CRITICAL(&mux);
        } else {
          u = accelToVolt(dx, th, dth, 20.0f * (v_des - dx));
        }
      }
    } else if (mode == SWING_UP || mode == BALANCE || mode == WAIT) {
      float ath = fabsf(th);
      if (limL || limR)                              mode = raiseFault(mode, "limit");
      else if (fabsf(x) > RAIL_HALF_M - SOFT_MARGIN) mode = raiseFault(mode, "rail");
      else {
        if (mode == BALANCE && ath > p.release_rad) mode = p.swingup_on ? SWING_UP : WAIT;
        else if (mode != BALANCE && ath < p.catch_rad && fabsf(dth) < 5.0f) mode = BALANCE;
        if (mode == BALANCE)
          u = -(p.K[0] * (x - p.x_ref) + p.K[1] * dx + p.K[2] * th + p.K[3] * dth);
        else if (mode == SWING_UP)
          u = accelToVolt(dx, th, dth, swingUpAccel(p, x, dx, th, dth));
      }
    }
    if (mode == IDLE || mode == FAULT) u = 0;
    u = clampf(u, -p.u_max, p.u_max);
    // sürücü yalnız hareket modlarında etkin; IDLE'da araba elle itilebilir (işaret kontrolü için)
    digitalWrite(PIN_EN, (mode == IDLE || mode == FAULT) ? LOW : HIGH);
    motorWrite(u);

    // ---------- telemetri anlık görüntüsü
    k++;
    tel_acc += telemetry_hz * dt;
    portENTER_CRITICAL(&mux);
    S.mode = mode;                                     // komut doğrulaması için her adımda güncel
    if (tel_acc >= 1.0f) {
      tel_acc -= 1.0f;
      S.n = n++; S.t = k * dt; S.x = x; S.dx = dx; S.th = th; S.dth = dth; S.u = u;
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
    if (!strcmp(key, "x_ref") && fabsf(v) > RAIL_HALF_M - 0.1f) { sendAck("set_param", false, "x_ref ray sınırına çok yakın"); return; }
    if (!strcmp(key, "u_max") && v > V_SUPPLY) { sendAck("set_param", false, "u_max besleme gerilimini aşamaz"); return; }
  }
  if (catch_deg >= release_deg) { sendAck("set_param", false, "catch_deg < release_deg olmalı"); return; }
  // 2) uygula
  portENTER_CRITICAL(&mux);
  S.catch_rad = radians(catch_deg); S.release_rad = radians(release_deg);
  if (d["swingup"].is<float>()) S.swingup_on = d["swingup"].as<float>() != 0;
  if (d["k_E"].is<float>())   S.su_kE   = d["k_E"].as<float>();
  if (d["a_max"].is<float>()) S.su_amax = d["a_max"].as<float>();
  if (d["k_x"].is<float>())   S.su_kx   = d["k_x"].as<float>();
  if (d["k_v"].is<float>())   S.su_kv   = d["k_v"].as<float>();
  if (d["x_ref"].is<float>()) S.x_ref   = d["x_ref"].as<float>();
  if (d["u_max"].is<float>()) S.u_max   = d["u_max"].as<float>();
  portEXIT_CRITICAL(&mux);
  if (d["telemetry_hz"].is<float>()) telemetry_hz = (int)d["telemetry_hz"].as<float>();
  sendAck("set_param", true);
}

static void handleCommand(const char *line) {
  JsonDocument d;
  if (deserializeJson(d, line)) { sendAck("?", false, "json"); return; }
  const char *cmd = d["cmd"] | "";
  Shared s; portENTER_CRITICAL(&mux); s = S; portEXIT_CRITICAL(&mux);

  if (!strcmp(cmd, "ping")) {
    char extra[64];
    snprintf(extra, sizeof extra, ",\"Ts\":%.4f,\"fw\":\"%s\",\"homed\":%s", CTRL_PERIOD_MS / 1000.0f, FW_NAME,
             s.homed ? "true" : "false");
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
    SET_SHARED(req_start, true); sendAck("start", true); return;
  }
  if (!strcmp(cmd, "home")) {
    if (s.mode != IDLE && s.mode != FAULT) { sendAck("home", false, "önce stop"); return; }
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
        if (overflow) sendAck("?", false, "satır çok uzun");
        else if (len) handleCommand(line);
        len = 0; overflow = false;
      } else if (len < sizeof(line) - 1) line[len++] = ch;
      else overflow = true;                             // taşma: satırın sonuna kadar at
    }
    Shared s;
    portENTER_CRITICAL(&mux);
    s = S; S.ev_homed = S.ev_fault = S.ev_limitL = S.ev_limitR = S.ev_calib = false;
    portEXIT_CRITICAL(&mux);
    if (s.ev_limitL) Serial.print("{\"evt\":\"limit\",\"side\":\"L\"}\n");
    if (s.ev_limitR) Serial.print("{\"evt\":\"limit\",\"side\":\"R\"}\n");
    if (s.ev_homed)  Serial.print("{\"evt\":\"homed\"}\n");
    if (s.ev_calib)  Serial.printf("{\"evt\":\"calib\",\"ok\":%s}\n", s.angle_ok ? "true" : "false");
    if (s.ev_fault)  Serial.printf("{\"evt\":\"fault\",\"reason\":\"%s\"}\n", s.fault_reason ? s.fault_reason : "?");
    if (s.n != last_n) {                                // yeni telemetri
      last_n = s.n;
      int l = snprintf(buf, sizeof buf,
        "{\"n\":%lu,\"t\":%.4f,\"x\":%.5f,\"dx\":%.4f,\"th\":%.5f,\"dth\":%.4f,\"u\":%.3f,\"mode\":\"%s\"}\n",
        (unsigned long)s.n, s.t, s.x, s.dx, s.th, s.dth, s.u, MODE_NAMES[s.mode]);
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
  pinMode(PIN_LIM_L, INPUT_PULLUP); pinMode(PIN_LIM_R, INPUT_PULLUP);
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
  S.mode = IDLE;

  Serial.printf("{\"evt\":\"boot\",\"fw\":\"%s\",\"Ts\":%.4f}\n", FW_NAME, CTRL_PERIOD_MS / 1000.0f);
  xTaskCreatePinnedToCore(ctrlTask, "ctrl", 4096, nullptr, configMAX_PRIORITIES - 1, nullptr, 1);
  xTaskCreatePinnedToCore(commTask, "comm", 6144, nullptr, 2, nullptr, 0);
}

void loop() { vTaskDelete(nullptr); }   // tüm iş görevlerde
