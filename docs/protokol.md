# ESP32 ↔ PC Haberleşme Protokolü (v1)

Bu belge mekatronik ve bilgisayar ekipleri arasındaki **sözleşmedir**. Firmware ve arayüz bu belgeye göre yazılır; bir taraf değişiklik yapacaksa önce bu belge güncellenir.
Referans uygulamalar: `pendulum_lab/comm/protocol.py` (PC) ve `firmware/esp32_pendulum/esp32_pendulum.ino` (ESP32).

## 1. Fiziksel katman

| Ayar | Değer |
|---|---|
| Bağlantı | USB-seri (ESP32 kartındaki CP2102/CH340) |
| Baud | **921600** (115200 de çalışır ama en fazla ~250 satır/s taşır) |
| Çerçeve | 8N1, akış kontrolü yok |
| Satır sonu | `\n` (tek bayt). `\r` gönderilmez |
| Karakter seti | UTF-8, satır başına en fazla 255 bayt |

PC portu açarken DTR/RTS'yi düşük tutar, böylece ESP32 resetlenmez.

## 2. Ortak kurallar

* **SI birimleri**: metre, radyan, saniye, volt (veya input_mode'a göre N / m/s²).
* **θ = 0 sarkacın DİK konumudur.** θ > 0 iken sarkaç +x yönüne yatıktır. θ, (−π, π] aralığına sarılmış gönderilir.
* **x = 0 rayın ortasıdır**, sağa doğru pozitif. Homing'den önce x'in anlamı yoktur.
* Durum vektörü sırası her yerde **[x, ẋ, θ, θ̇]** şeklindedir.
* Kontrol kanunu: `u = −(K0·(x − x_ref) + K1·ẋ + K2·θ + K3·θ̇)`.

### Montaj sonrası işaret kontrolü (mekatronik ekibi)
1. Kartı açın; sarkaç **aşağıda ve hareketsiz** olsun (açılışta açı sıfırı bu konumdan bulunur, `calib` olayı `ok:true` gelmeli). Telemetride `th` yaklaşık ±3.14 olmalıdır.
2. Sarkacı elle dik konuma getirin: `th` ≈ 0 olmalıdır.
3. Dik konumdan arabanın **sağına** doğru yatırın: `th` **pozitif** olmalıdır. Değilse firmware'de `ANGLE_SIGN = -1` yapın.
4. Arabayı elle sağa itin: `x` artmalıdır. Artmıyorsa `X_SIGN = -1` yapın.
5. Motor bağlantısını arabadan ayırıp `u > 0` verdiğinizde araba sağa gitmelidir. Gitmiyorsa motor kablolarını ters çevirin.

## 3. ESP32 → PC mesajları

### 3.1 Telemetri (varsayılan 200 Hz)
```json
{"n":1234,"t":12.3400,"x":0.01230,"dx":-0.0410,"th":0.00310,"dth":0.1200,"u":1.250,"mode":"BALANCE"}
```
| Alan | Tür | Açıklama |
|---|---|---|
| `n` | tamsayı | Sıra numarası; her telemetride 1 artar. PC kayıp satırları bununla sayar |
| `t` | s | Açılıştan beri geçen süre (kontrol döngüsü saati) |
| `x`, `dx` | m, m/s | Araba konumu ve kestirilen hızı |
| `th`, `dth` | rad, rad/s | Sarkaç açısı ve kestirilen açısal hızı |
| `u` | V | Motora uygulanan giriş (doyum sonrası) |
| `mode` | metin | `IDLE`, `HOMING`, `WAIT`, `SWING_UP`, `BALANCE`, `FAULT`, `FRICTION` (sürtünme ölçümü) |

Geriye uyumluluk: ilk taslaktaki `{"t":1.25,"theta":0.05,"x":0.12}` biçimi de kabul edilir. Eksik alanlar 0 sayılır.

### 3.2 Olaylar
| Mesaj | Ne zaman |
|---|---|
| `{"evt":"boot","fw":"ref-1.1","Ts":0.002}` | Açılışta. PC sıra sayacını ve grafiği sıfırlar, Ts'yi kontrol eder, K ve ayarları **yeniden yükler** (cihaz resetlenince PC'den gelen ayarlar kaybolur) |
| `{"evt":"friction","ok":true,"v_pos":0.51,"v_neg":0.47,"v_c":0.49,"slope":8.6}` | Sürtünme ölçümü bittiğinde. `v_c`: kayma sürtünmesini yenen gerilim [V] (iki yönün ortalaması), `slope`: (β + b)/α [V·s/m]. Başarısızsa `{"ok":false,"reason":"…"}`, mod IDLE'a döner |
| `{"evt":"calib","ok":true}` | Açı kalibrasyonu bittiğinde (açılışta ve `calib` komutundan sonra). `ok:false` → `start` reddedilir |
| `{"evt":"limit","side":"L"}` / `"R"` | Limit switch'e ilk basıldığında |
| `{"evt":"homed"}` | Homing tamamlandığında |
| `{"evt":"fault","reason":"…"}` | FAULT moduna geçildiğinde, **bir kez**. Sebepler: `estop`, `rail` (yazılımsal ray sınırı), `limit` (çalışırken limit switch), `sensor` (AS5600 art arda 3 kez okunamadı), `homing_timeout` (20 s), `homing_wrong_side` (homing'de sağ switch → motor/enkoder yönü ters), `homing_travel` (rayın 2 katından fazla yol → sol switch kablosu kopuk olabilir) |

### 3.3 Komut yanıtları
Her komut tam olarak bir yanıt alır: `{"ack":"start","ok":true}` veya `{"ack":"start","ok":false,"msg":"önce home (kalibrasyon) yapılmalı"}`.
`ok:true` yalnızca komut gerçekten uygulanacaksa döner; cihaz o anki modda komutu kabul etmiyorsa `ok:false` ve sebep gönderir. Yanıt ek alan taşıyabilir (ör. `ping`).

## 4. PC → ESP32 komutları

| Komut | Örnek | Etki |
|---|---|---|
| `ping` | `{"cmd":"ping"}` | `{"ack":"ping","ok":true,"msg":"pong","Ts":0.002,"fw":"ref-1.1","homed":false}`. PC bağlanınca gönderir; `Ts` tasarımdakiyle aynı değilse K **gönderilmez** |
| `home` | `{"cmd":"home"}` | Yalnız IDLE/FAULT modunda. Araba sola gider, sol limit switch'te (zaten basılıysa hemen) x = −rail_half kabul eder, sonra ortaya döner. Güvenlik: bkz. 3.2 `homing_*` hataları |
| `calib` | `{"cmd":"calib"}` | Yalnız IDLE modunda. Sarkaç aşağıda ve hareketsizken açı sıfırını yeniden bulur (açılıştaki kalibrasyon başarısız olduysa) |
| `friction` | `{"cmd":"friction"}` | Yalnız **IDLE**'da, homing ve açı kalibrasyonu yapılmışsa; araba ray ucundan en az 0.23 m uzakta ve duruyor, sarkaç aşağıda ve hareketsiz olmalı. Gerilim 1 V/s ile artırılır; araba hızı 4–10 cm/s iken (gerilim, hız) çiftlerine doğru oturtulur, önce +x sonra −x (yön başına ≤ 8 cm + yumuşak duruş). Sonuç `friction` olayıyla gelir. `stop` iptal eder, `estop` FAULT'a geçirir. Ayrıntı: `pendulum_lab/control/friction.py` |
| `start` | `{"cmd":"start"}` | Yalnız **IDLE** modunda, homing ve açı kalibrasyonu yapılmışsa. FAULT'tan doğrudan başlatılamaz (önce `stop`). Araba ray ucuna 0.10 m'den yakınsa veya 0.2 m/s'den hızlı gidiyorsa reddedilir. Swing-up açıksa SWING_UP, kapalıysa WAIT |
| `stop` | `{"cmd":"stop"}` | IDLE moduna geçer, sürücü kapatılır (araba elle itilebilir). Homing'i de iptal eder |
| `estop` | `{"cmd":"estop"}` | FAULT moduna geçer, sürücü EN pini düşürülür. Aynı anda gelen diğer komutları **ezer**. Arayüzde **Esc** tuşu (açık bir pencere varken de çalışır) |
| `set_gain` | `{"cmd":"set_gain","K":[-9.57,-22.13,-56.44,-8.71]}` | LQR kazancı. Çalışırken de değiştirilebilir |
| `set_param` | `{"cmd":"set_param","catch_deg":15,"release_deg":30}` | Aşağıdaki tablo |

**Yalnız sanal ESP32 (firmware'de yok, gerçek kart `ok:false` döner):** `{"cmd":"kick","dth":1.0}` sarkacı dürter; `{"cmd":"pause","on":1}` / `"on":0` fiziği ve kontrolü dondurur / sürdürür (telemetri son değerlerle akmaya devam eder); `{"cmd":"reset","th0_deg":180}` IDLE'a döner, duraklatmayı kaldırır ve sarkacı θ₀ açısında durağan koyar. Liste: `protocol.py::SIM_ONLY_COMMANDS`.

`set_param` alanları (bir mesajda birden fazla alan olabilir). **Ya hepsi uygulanır ya hiçbiri:** tek bir alan geçersizse mesajın tamamı reddedilir. Aralıkların tek kaynağı `pendulum_lab/comm/protocol.py::SET_PARAM_LIMITS`'tir; firmware'deki `PARAM_LIMITS` tablosunun aynı olduğunu bir test denetler.

| Alan | Aralık | Anlamı |
|---|---|---|
| `catch_deg` | 1–45 | \|θ\| bu değerin altına inince LQR devreye girer |
| `release_deg` | 2–90 | \|θ\| bu değeri aşınca LQR bırakılır. Her zaman `catch_deg < release_deg` olmalı (histerezis) |
| `swingup` | 0/1 | 0 ise sarkaç elle kaldırılana kadar beklenir (WAIT) |
| `k_E` | 0–10000 | Swing-up enerji kazancı |
| `a_max` | 0–50 | Swing-up ivme sınırı [m/s²] |
| `k_x`, `k_v` | 0–1000 | Swing-up araba merkezleme kazançları |
| `x_ref` | ±0.3 | Dengedeyken arabanın hedef konumu [m]. Cihaz ayrıca \|x_ref\| ≤ rail_half − 0.1 şartını arar |
| `u_max` | 0–48 | Giriş doyumu [V]. Cihaz besleme geriliminden büyük değeri reddeder |
| `telemetry_hz` | 1–500 | Telemetri hızı |
| `fc_volt` | 0–6 | Coulomb sürtünme telafisi, **gerilim** olarak [V] (yalnız swing-up'ta; homing telafi kullanmaz, sürtünmeyi integralle yener). Sürtünme ölçümünün sonucu (`v_c`). Cihaz kuvvete fc_volt·α ile çevirip yine α'ya bölerek uyguladığı için motor sabitlerindeki hata etkilemez. Bağlanınca PC, parametrelerdeki T_f'den hesaplayıp gönderir |

## 5. Mod makinesi

```
           home            homed
  IDLE ──────────▶ HOMING ───────▶ IDLE
   ▲  │ start (homed)
   │  ▼
   │ SWING_UP ── |θ|<catch & |θ̇|<5 ──▶ BALANCE
   │    ▲                                 │
   │    └────────── |θ|>release ──────────┘
   │
  stop        her mod ── |x| > rail_half − 3 cm, limit switch, sensör hatası, estop ──▶ FAULT (motor kapalı)

           friction (homed)        bitti / stop
  IDLE ───────────────────▶ FRICTION ─────────────▶ IDLE   (sonuç: friction olayı)
```
FAULT modundan `stop` (→ IDLE) veya `home` ile çıkılır; `start` FAULT'ta reddedilir.

## 6. Açık karar: Homing'i kim başlatır?
**Öneri (bu belgedeki ve kodlardaki davranış):** Güç gelince ESP32 IDLE modunda bekler. Homing yalnızca PC'den gelen `home` komutuyla ("KALİBRE ET" butonu) başlar. `start` komutu homing yapılmadan reddedilir.
Gerekçe: Araba açılışta kendiliğinden hareket etmez (güvenlik), ve kullanıcı rayın boş olduğunu görerek kalibrasyonu başlatır.
Bu öneri iki ekip tarafından onaylanmalıdır.

## 7. Hız bütçesi
| Kalem | Değer |
|---|---|
| Telemetri satırı | ~95 bayt |
| 921600 baud kapasitesi | ~92 000 bayt/s → ~950 satır/s |
| Varsayılan telemetri | 200 Hz → bant genişliğinin ~%21'i |
| PC tarafı çözümleme | satır başına ~5 µs (ayrı thread) |
| Kontrol döngüsü | 500 Hz (Ts = 2 ms), **ESP32 üzerinde**. Telemetri kontrolü hiç bekletmez |
| Ts kuralı | Tam sayı milisaniye, 1–20 ms (FreeRTOS 1 ms tik). `gains.h` başka değeri reddeder |
