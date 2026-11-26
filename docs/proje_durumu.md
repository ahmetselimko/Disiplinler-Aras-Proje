# Proje Durum Raporu — Bilgisayar Mühendisliği Kısmı

*Ters Sarkaç Disiplinlerarası Projesi · Ekim–Kasım 2026 (son güncelleme 18.11.2026)*

Bu rapor bilgisayar ekibinin şimdiye kadar yaptıklarını, mekatronik ekibinin üzerindeki işleri, çözümün güçlü ve zayıf yönlerini ve sırada ne olduğunu özetler.

---

## 1. Kısa özet

* Bilgisayar tarafı **çalışır durumda**: Matlab'in yerini tutan bir Python kütüphanesi, canlı izleme ve kontrol arayüzü, donanım olmadan deneme yapmayı sağlayan bir sanal ESP32, mekatronik ekibe başlangıç olarak bir ESP32 firmware'i ve iki ekip arasındaki haberleşme sözleşmesi hazır.
* Sanal cihazla uçtan uca deneme başarılı: kalibrasyon (homing) → swing-up → LQR ile denge. Veri 200 Hz'de kayıpsız geliyor, grafik ~60 FPS çiziliyor.
* **305 otomatik test** geçiyor; firmware her push'ta GitHub Actions'ta iki kart (ESP32-S3, klasik ESP32) × iki core sürümüyle derleniyor. Kod art arda bağımsız incelemelerden geçti (§4: ilk tur 21, doğrulama turu 6 bulgu; §4.2: 22–25.11.2026 tarihli turlar, 25.11'deki tam proje denetimi dahil); bulunanların hepsi düzeltildi. 18.11.2026'da belgeler kodla ayrıca karşılaştırıldı (§4.1).
* Fiziksel parametreler **mekatronik ara raporunun (23.11.2026) tasarım değerleri** (#50 Aşama 2): +20 g uç kütleli sarkaç (m = 0.105 kg, l = 0.0992 m), araba M = 1.28 kg, JGB37-520 19:1 (Tablo 16: α = 2.26 N/V, b = 48.41 N·s/m), 3 N sürtünme, enkoder 44 CPR (836 sayım/tur, 0.072 mm). Uç kütlesiz ve +50 g seçenekleri `params_ucsuz.json` / `params_50g.json`. Ölçüm değil (`placeholder: true`). Bu değerlerle K = [−9.83, −46.64, +101.47, +14.36]; aşağıdan kalkış ~4.3 s, max \|x\| ≈ 0.14 m (swing-up hedef enerji oranı 1.05). Ölçüm gelince yalnızca `config/params.json` değişecek.
* Mekatronik ekibinin Matlab/Simulink dosyaları `matlab/` klasöründe; model Python kütüphanesiyle birebir aynı (testle doğrulanıyor).
* **θ işareti (#50 Aşama 1, 25.11.2026):** bütün projede mekatronik raporundaki (8.1.3) ve Matlab'daki kural geçerli: θ = 0 dik, **θ > 0 sarkaç sola yatık**, x ve F sağa pozitif. Fiziksel davranış değişmedi (eski kodla alınmış referans senaryolarla doğrulandı: `tests/test_theta_isaret.py`); K'nın θ, θ̇ kazançlarının işareti döndü, ör. K = [−9.52, −27.29, +61.49, +9.74]. Eski kurala göre üretilmiş `gains.h` kullanılmamalı; firmware `ref-1.2` boot/ping'de `"theta":"sol"` bildirir, bildirmeyen (eski) cihaza arayüz K göndermez. Eski CSV kayıtları (başlıkta `theta_pozitif` yok) okunurken otomatik çevrilir; başlıksız CSV eski kuralda sayılır.

**En önemli tasarım kararı:** Sarkacı dengede tutan döngü (saniyede 500 kez) **ESP32 üzerinde** çalışır, bilgisayarda değil. USB gecikmesi ve Windows'un düzensiz zamanlaması, döngü PC'de olsaydı hangi dille yazılırsa yazılsın sarkacı düşürürdü. Bilgisayar zaman açısından kritik olmayan işleri yapar: K'yı hesaplamak ve göndermek, izlemek, kaydetmek, simülasyon yapmak.

---

## 2. Bilgisayar tarafında neler var

### 2.1 `pendulum_lab` kütüphanesi (Matlab karşılığı)
| Modül | Ne yapar | Matlab karşılığı |
|---|---|---|
| `model/params.py` | Fiziksel parametreler (JSON'dan okunur). Çubuk + küre/küp uç kütleden m, l, I hesabı. Rotor ataletinin eşdeğer kütlesi | — |
| `model/dynamics.py` | Lineer olmayan araba-sarkaç denklemleri (Lagrange). DC motor, ideal kuvvet ve step motor (ivme) girişleri; Coulomb sürtünmesi; arabaya veya sarkaca dış itme | Simulink modeli |
| `model/linearize.py` | Dik konumda (θ = 0) analitik A, B ve sayısal jacobian | `jacobian` |
| `model/motor.py` | DC motor transfer fonksiyonu, en yüksek araba hızı/kuvveti, step motor ivme sınırı | `tf` |
| `control/lqr.py` | Sürekli ve ayrık LQR (Riccati çözümü), ZOH ayrıklaştırma | `lqr`, `dlqr`, `c2d` |
| `control/swingup.py` | Enerji bazlı swing-up (Åström–Furuta) ve araba merkezleme | — |
| `control/supervisor.py` | Mod makinesi: SWING_UP ↔ LQR geçişi (15°/30° histerezis; açısal hız, araba konumu ve hızı şartları), ray sınırı güvenliği | — |
| `control/friction.py` | Sürtünme ölçümü (araba iki yönde sürülür, Coulomb sürtünmesi gerilim olarak bulunur) | — |
| `control/pid.py` | Anti-windup'lı PID (karşılaştırma için) | `pid` |
| `analysis/linear.py` | Durum uzayı nesnesi, kutuplar, kontrol edilebilirlik, transfer fonksiyonu, basamak yanıtı, kök eğrisi | `ss`, `eig`, `ctrb`, `step`, `rlocus` |
| `analysis/summary.py`, `analysis/sensitivity.py` | Deneme özeti (dengeye geçiş, oturma süresi, doyum); parametre hassasiyet taraması | — |
| `sim/simulator.py` | Lineer olmayan simülasyon: ESP32 gibi 500 Hz ayrık kontrol, AS5600 12-bit ve 44 CPR × 19 enkoder kuantalaması, Kalman, gürültü, gecikme, dürtme, ray uçları | `ode45` |
| `sim/matlab_compat.py` | `cart_pendulum_application.m` ile aynı deneme; θ işareti artık aynı olduğundan eski dönüşüm fonksiyonları birim | — |
| `filters.py` | Türevden hız kestirimi (25 Hz), Butterworth, dik bölgede kararlı hal Kalman filtresi | `butter`, `kalman` |
| `comm/` | JSON protokolü, seri port, ayrı thread'de okuyucu, sanal ESP32 | `serialport` |
| `recorder.py`, `export.py` | CSV kayıt, kaydı okuma ve aynı başlangıç koşullu simülasyonla karşılaştırma; ESP32 için `gains.h` üretimi (K, geçiş şartları, swing-up, Kalman matrisleri) | — |

### 2.2 Arayüz (`python main.py`)
* **Canlı İzleme:** θ, x, u, enerji ve mod kayan grafikleri, araba-sarkaç animasyonu, mod göstergesi, sonuç özeti, KALİBRE ET / BAŞLAT / DURDUR / ACİL STOP (**Esc**), hedef konum, swing-up aç/kapa, sürtünme ölçümü, CSV kayıt; simülasyonda kuvvet/süre/yer (sarkaç veya araba) seçilebilen dürtme, duraklat ve sıfırla; durum çubuğunda veri hızı, kayıp satır ve FPS.
* **Model ve LQR Tasarımı:** parametre (g dahil) ve motor değerleri, Q/R ağırlıkları → K. Açık ve kapalı çevrim kutupları, kutup haritası, en yüksek araba hızı uyarısı, swing-up ayarları (hedef enerji oranı) ve LQR'a geçiş şartları, "küre mi küp mü" geometri hesaplayıcı, `gains.h` dışa aktarma.
* **Analiz ve Simülasyon:** aşağıdan swing-up dahil lineer olmayan simülasyon (başlangıç hızları, Kalman aç/kapa) ve animasyonlu tekrar oynatma, model uyumsuzluğu ve dürtme denemeleri, Matlab uyumlu deneme, basamak yanıtı, kök eğrisi.
* **Kayıt Oynatma:** CSV kaydını aynı başlangıç koşullu simülasyonla üst üste çizer, RMSE verir; gerçek denemelerde modeli doğrulamak için.
* Bağlanınca cihazın kontrol periyodunu (Ts) sorar. Swing-up açık/kapalı anahtarını her durumda gönderir; K ve diğer ayarları yalnız Ts eşleşirse, parametreler yer tutucu değilse ve giriş türü voltage ise otomatik yükler (yer tutucuyla gerçek kartta onayla gönderilir). Cihaz resetlenirse ayarları yeniden yükler.

### 2.3 Sanal ESP32
Donanım olmadan, gerçek kartla **aynı protokol ve aynı komut kurallarıyla** çalışır: homing, limit switch'ler, swing-up, LQR, sensör kusurları. Gerçek karta geçişte arayüzde yalnızca port seçimi değişir.

### 2.4 Referans ESP32 firmware'i (`firmware/esp32_pendulum/`)
Mekatronik ekibe başlangıç noktası. İki FreeRTOS görevi var: 500 Hz kontrol çekirdek 1'de, haberleşme çekirdek 0'da çalışır. Python'daki kontrol mantığının birebir karşılığıdır. Güvenlik davranışları: yanlış yöne homing, süre aşımı, sensör kopması, kalibrasyon hatası ve baskın acil stop. **Hatasız derleniyor (ESP32 core 3.3.12 ve 2.0.17), ama kartta henüz denenmedi.**

### 2.5 Belgeler (`docs/`)
| Belge | Okuyucu | İçerik |
|---|---|---|
| `protokol.md` | **İki ekip** | Haberleşme sözleşmesi: mesajlar, komutlar, mod makinesi, işaret kontrolü |
| `parametre_tablosu.md` | **Mekatronik ekibi** | Doldurulacak ölçü tablosu ve nasıl ölçüleceği |
| `mimari.md` | Bilgisayar ekibi | Neden döngü ESP32'de, thread yapısı, katmanlar |
| `matlab_karsiliklari.md` | Herkes | Matlab fonksiyonu → kütüphanedeki karşılığı; mekatronik Matlab modeliyle karşılaştırma |
| `kullanim_kilavuzu.md` | Herkes | Ekran görüntülü arayüz anlatımı |
| `katki_3_6_13.md`, `katki_10.md`, `katki_14_15.md` | Bilgisayar ekibi | Enerji grafiği, deneme özeti, hassasiyet analizi; Matlab karşılaştırması; Kalman ve kayıt oynatma ayrıntıları |
| `swingup_karar_calismasi.md` (+ `.json`) | İki ekip | #50 Aşama 3 karar çalışması: uç kütlesi × geçiş kuralı × E_ratio × ray × telafi (betikle üretilir) |
| `hassasiyet_raporu.md` | Herkes | Parametre hassasiyet taramasının sonuçları (25.11.2026'da Aşama 2'nin `params.json`'ı, Kalman ve 44 CPR enkoderle yeniden üretildi) |
| `../matlab/README.md` | İki ekip | Matlab dosyaları, ortak θ işareti (sola +), eski K'lara dikkat |
| `mekatronik_on_degerlendirme.md` | İki ekip | Mekatronik tasarım ölçüleriyle ön değerlendirme: swing-up için uç kütlesi ve hedef enerji, iki raporun farkları |
| `media/rapor/README.md` | Herkes | Rapor şekilleri (aşağıdan kalkış, dürtme, kutup haritası, model uyumsuzluğu) ve sayıları; `python tools/rapor_sekilleri.py` ile yeniden üretilir |

---

## 3. Mekatronik ekibinde neler var / neler bekleniyor

Toplantı notları ve tahtadaki plana göre:

| Alan | Durum | Bilgisayar tarafına etkisi |
|---|---|---|
| **Mekanik tasarım:** tabla mekanizması, kayış-kasnak, sigma profil çerçeve, rulman ve köşebentler | Inventor modeli hazırlanıyor | Bittiğinde m, M, l ve atalet `parametre_tablosu.md`'ye girilecek. Inventor "Physical" sekmesi I değerini doğrudan verir |
| **Motor seçimi** | **Seçildi: JGB37-520 12 V 530 rpm** (mekatronik ara raporu, "final actuator"). BOM v3'teki diğer aday DFRobot FIT0186 (12 V, 251 rpm, 43.8:1, 16 CPR, durma 7 A / 18 kg·cm). Önceki "Pololu 25D 251 RPM" bilgisi aslında FIT0186'yı kastediyordu (o da 37 mm gövdeli, Pololu değil). Kasnak **HTD3M 20 diş** (r ≈ 9.55 mm, mekatronik doğruladı) | **BOM simülasyonu (22.11.2026, `python tools/motor_karsilastirma.py --sadece BOM`, yer tutucu sarkaç/araba, iki ray × iki J_rotor × sürtünmeli/sürtünmesiz = 8 deneme):** JGB37-520 + 20 diş (v_max ≈ 0.56 m/s) durma torku 3 kg·cm varsayımıyla **8/8**, 6 kg·cm varsayımıyla **4/8** (başarısızlar raydan taşma: swing-up kazançları bu motora göre ayarlanmamış); 40 dişle iki varsayımda da 8/8. FIT0186 + 20 diş (v_max ≈ 0.26 m/s) **0/8**, 40 dişle 3/8 (sürtünmeli denemelerin hepsi başarısız). Sonuç: **JGB37-520 + 20 diş uygulanabilir görünüyor, FIT0186 + 20 diş değil.** JGB37-520'nin tork/akım/enkoder değerleri BOM'da yok, VARSAYIM; datasheet gelince araçtaki `BOM_ADAYLAR` güncellenip yeniden çalıştırılmalı. Sarkaç uzunluğu ve kütleleri değişirse sonuç değişir. *Önceki karşılaştırmalar (37D / 25D / Pololu 35 / NEMA17) aynı araçta duruyor; ayrıntı #7* |
| **Sarkaç kütlesi: küp mü küre mi** | Karar verilmedi | Geometri hesaplayıcı ikisini karşılaştırır. Uzun çubukta fark küçüktür; belirleyici olan ağırlık merkezi uzaklığı l'dir. Montaj kolaylığına göre seçilebilir |
| **Sistem modelleme ve lineerleştirme** (θ = 0'da, parametrik) | Matlab dosyaları geldi (`matlab/`) | Kütüphanede hazır ve testli (analitik + sayısal jacobian). Mekatronik ekibin Matlab modeliyle doğrusal ve doğrusal olmayan denklemler birebir aynı (`tests/test_matlab_compat.py`). İşaret kuralı birleştirildi (#1 → #50 Aşama 1): iki tarafta da θ sola pozitif, K dönüşümsüz aynı |
| **Kontrol:** LQR kararı, enerji bazlı swing-up araştırması | LQR seçildi, swing-up araştırılacak | İkisi de uygulandı ve simülasyonda çalışıyor: mekatronik tasarım değerleriyle (+20 g) aşağıdan ~4.3 s'de kalkış, max \|x\| ≈ 0.14 m (hedef enerji oranı 1.05; uç kütlesiz sarkaç 1.05'te kalkmıyor); üç uç kütlesi seçeneğinin hepsinde karma bir model hatasında (M +%20, m −%15, l +%10, b ×2) da yakalama (test). m ve l birlikte saptığında çalışma aralığı sınırlı (rapor şekilleri §4). Kazançlar araştırmaya başlangıç noktası olarak kullanılabilir |
| **Sensör / mikrodenetleyici / sürücü** | **BOM v3:** ESP32-S3-DevKitC-1 **N16R8**, AS5600, BTS7960B; ayrıca **MPU6050** listede | **Kart klasik ESP32'den S3'e değişti:** eski pinlerin çoğu (22, 25, 26, 27, 32, 33) S3 N16R8'de yok ya da flash/PSRAM'e bağlı. Pinler `firmware/esp32_pendulum/board_pins.h`'e taşındı; S3 ve klasik ESP32 için ayrı setler, derleme hedefinden otomatik seçilir, CI ikisini de derler. **MPU6050'nin rolü teyit bekliyor:** firmware açıyı AS5600'den okuyor; MPU6050 sarkaç açısı için kullanılacaksa (ivmeölçer dönen sarkaçta bozulur, jiroskop kayar) açı okuma ve Kalman kısmı yeniden ele alınmalı. Aynı I2C hattına bağlanabilir (0x68 ≠ 0x36) |
| **Uç sensörü** | **BOM v3:** 2 × DC163-3 micro switch ve 2 × TCST2103 yarık optik sensör listede; notta "ya biri ya diğeri". Hangisi kullanılacak, teyit bekliyor | Firmware ikisini de değişiklik olmadan destekliyor (`LIMIT_SWITCHES 1`, `LIM_PRESSED = HIGH`): NC switch'te ve pull-up'lı TCST2103'te tetiklenme ve kopuk kablo HIGH okunur. TCST2103 bağlantısı (LED direnci, harici pull-up) firmware README'de |
| **Donanımsal acil stop** | **Yok** (16.11.2026: "limitler aşılınca otomatik durdurulacak") | Durdurma yazılıma (ESP32) bağlı; motor beslemesi elle kolayca kesilebilmeli |
| **Homing'i kim başlatacak** | Kapandı | Kart açılınca bekler; kalibrasyon yalnızca arayüzdeki KALİBRE ET ile, sol switch'i bularak (`protokol.md` §6) |

---

### Swing-up karar çalışması (#50 Aşama 3, 25.11.2026)
 `python tools/swingup_karar.py` → [`swingup_karar_calismasi.md`](swingup_karar_calismasi.md). Uç kütlesi (yok / +20 g / +50 g) × geçiş kuralı (bizim 15°/30° ya da mekatroniğin 10°/25° + θ̇, x, ẋ şartları) × E_ratio (1.00–1.10) × ray (±0.40 / 0.60 / 0.80 m) × sürtünme telafisi; her durum 3 gürültü tohumu ve m, l ±%5 ile (900 koşu, tasarım değerleri). Tablolardan çıkanlar:
* "Kalktı" = sonda dengede **ve** LQR yakaladıktan sonra hiç bırakmadı (`rapor_sekilleri.classify`); LQR'ın bırakıp swing-up'ın yeniden kaldırdığı koşular kalkmış sayılmadı (26.11 incelemesi).
* Her rayda ve telafi açık/kapalı sağlam yalnız **+50 g, bizim kural, E_ratio 1.02 ya da 1.05** (araba en fazla 0.26 / 0.19 m).
* Yalnız sürtünme telafisi açıkken her rayda sağlam: uç kütlesiz – bizim – 1.10; **+20 g – bizim – 1.05**; +50 g – bizim – 1.01; +50 g – mekatronik – 1.02. +20 g telafi kapalıyken hiçbir durumda sağlam değil: **ilk denemeden önce sürtünme ölçülüp telafi açılmalı.**
* **Mekatroniğin geçiş kuralı** yalnız +50 g'de sağlam (E_ratio 1.02 telafi açık, 1.05 telafi kapalı); uç kütlesiz ve +20 g'de hiçbir durumda sağlam değil.
* **Ray uzunluğu** 60 grubun hiçbirinde sonucu değiştirmedi; kalkan koşularda araba en fazla 0.34 m gitti: mevcut swing-up kazançlarıyla ±0.40 m yetiyor.
* Varsayılanlar (eşikler #11, `rail_half`) değiştirilmedi; karar iki ekibin.

## 4. Bağımsız kod incelemesi

Kod, yazandan bağımsız bir yapay zekâ ajanına inceletildi. Ajan fizik denklemlerini kendisi türetip karşılaştırdı, testleri çalıştırdı ve şüphelendiği hataları küçük betiklerle doğruladı.

**Doğru bulunanlar:** hareket denklemleri, doğrusallaştırma, motor sabitleri, swing-up enerji kuralının işareti, LQR/DLQR/ayrıklaştırma formülleri (python-control ile aynı sonuç), açı sarma, halka tampon, satır birleştirme ve kayıp sayımı.

**Bulunan ve düzeltilenler (21 bulgu):**
| Önem | Konu | Düzeltme |
|---|---|---|
| Kritik | Firmware homing sırasında sağ switch'e bakmıyordu, süre sınırı yoktu | Yanlış taraf switch'i → hata, 20 s süre sınırı, en fazla yol kontrolü; kablo kopmasına karşı NC switch önerisi |
| Önemli | AS5600 okunamazsa fark edilmiyordu | 3 hatalı okumada FAULT, 2 ms I2C zaman aşımı, mıknatıs kontrolü |
| Önemli | Açılış kalibrasyonu 4095/0 sınırında 180° hatalı olabiliyordu | Dairesel ortalama, hareketsizlik şartı, `calib` komutu |
| Önemli | Acil stop aynı anda gelen bir `start`'la ezilebiliyordu | Acil stop her zaman en son ve baskın işleniyor; hatadan doğrudan başlatılamıyor |
| Önemli | "Swing-up açık" ayarı ve hedef konum cihaza gitmiyordu | Bağlanınca ve cihaz resetlenince tüm ayarlar yükleniyor |
| Önemli | Seri port her okumada yeniden yapılandırılıyordu (Windows) | Zaman aşımı yalnızca açılışta ayarlanıyor |
| Önemli | Kontrol periyodu (Ts) arayüz ve firmware arasında tutarsız olabiliyordu | Yalnızca tam milisaniye; cihaz Ts'yi bildiriyor, uyuşmazsa K gönderilmiyor; derleme anı denetimleri |
| Küçük (14) | Parametre aralıkları üç yerde farklıydı; bilinmeyen mod CSV'ye "FAULT" yazılıyordu; kayıt hatası okuyucuyu durduruyordu; iki kayıt aynı dosyaya yazabiliyordu; simülasyonda araba rayı aşabiliyordu; rotor ataleti modelde yoktu; bazı testler zayıftı … | Hepsi düzeltildi. Parametre aralıklarının ve komut listesinin Python–firmware eşliğini artık bir test denetliyor |

**Doğrulama turu:** Aynı ajan düzeltmeleri yeniden inceledi. 18 bulgu tam, 3 bulgu kısmen düzelmişti; ayrıca 6 yeni sorun buldu. Bunların hepsi giderildi:
* **Besleme gerilimi:** firmware'de 12 V sabitti; artık arayüzdeki değerle `gains.h`'den geliyor.
* **Ayar gönderimi:** swing-up açık/kapalı ayarı başka bir alanın reddiyle kaybolmasın diye ayrı mesajla gönderiliyor.
* **Hedef konum:** x_ref ray boyuna göre sınırlanıyor.
* **Açı sıfırlama:** arayüze "Açı Sıfırla (calib)" butonu eklendi.
* **Homing kurtarması:** sağ switch üzerinde duran araba homing'e başlayınca artık hemen hata vermiyor.
* **Simülasyon:** IDLE/FAULT'ta araba, firmware'deki gibi serbest kayıyor.
* **Bozuk parametre dosyası:** arayüz çökmüyor, varsayılanlarla açılıp uyarı veriyor.
* **Esc:** dosya diyaloglarında da acil stop çalışıyor.

### 4.1 Belge–kod karşılaştırması (18.11.2026)
Son PR'lar (#35, #36, #37) yapay zekâ araçlarıyla hazırlandığı için, eski belgelere göre yazılmış kod olabileceği düşünülerek belgelerdeki her somut iddia ayrı bir ajanla koda karşı denetlendi. Bulunan kod tutarsızlıkları düzeltildi:
* **Enkoder çözünürlüğü:** simülasyon, sanal ESP32 ve kayıt oynatma arabanın konumunu eski 37D enkoderiyle ölçüyordu; firmware ve Kalman 48 CPR kullanıyordu. Artık hepsi firmware'deki formülü kullanıyor.
* **Dürtme:** Canlı sekmedeki dürtme "Matlab'daki Apply push gibi" diye anlatılıyordu ama sarkaca uygulanıyordu; Matlab arabaya uygular. Artık yer seçilebiliyor ve itme hareket denklemlerine tam olarak giriyor.
* **Matlab uyumlu deneme:** Matlab'ın parametreleri, Q, R'si ve ±10 N kuvvet sınırı yoktu; eklendi.
* **Motor varsayılanları:** `params.py` ile `params.json` farklı yer tutucular kullanıyordu; eşitlendi. `gains.h` güncel dışa aktarımla yeniden üretildi.

Küçük, bilinen fark: sanal ESP32'de homing yol sınırı hatası (`homing_travel`) yok (firmware'de var). Sürtünme ölçümünde uç switch'i ve yazılımsal ray sınırı ikisinde de denetlenir; `calib` olayı 22.11.2026'da eklendi. `hassasiyet_raporu.md` 25.11.2026'da güncel kasnakla (r = 9.55 mm) yeniden üretildi; sınıflar değişti: KRİTİK olanlar `l`, `motor.Kt`, `motor.Ke`, `motor.gear`, `motor.r_pulley`, `motor.T_f`; `m` ve `I` artık TOLERANSLI. #50 Aşama 2'de mekatronik tasarım değerleriyle yeniden üretildi: KRİTİK `m` ve `I`; ORTA `motor.Ke`, `motor.gear`, `motor.r_pulley`; diğerleri taranan koşullarda TOLERANSLI (hafif sarkaç: m ve I'nın ölçümü öncelikli).

### 4.2 Uçtan uca test ve bağımsız inceleme (22–25.11.2026)
Arayüz sanal ESP32 ile baştan sona otomatik olarak sürüldü (bağlan → homing → swing-up → denge → dürt → duraklat → durdur → acil stop → sıfırla → sürtünme ölçümü → tüm sekmeler → kayıt oynatma) ve kod üç ayrı ajana (model/kontrol, haberleşme/firmware, arayüz) inceletildi. Model, LQR, Kalman, swing-up, mod geçişleri, Matlab işaret dönüşümü ve protokol uyumu doğru bulundu. Düzeltilenler:

| Önem | Konu | Düzeltme |
|---|---|---|
| Önemli | PC bağlantısı koparsa (USB çekildi, arayüz çöktü) motor durmuyordu | Bağlantı bekçisi: arayüz 0.25 s'de bir `hb` gönderir; cihaz hareketli modda 1 s satır almazsa FAULT `link` (protokol.md §4) |
| Önemli | Kart resetlenince ROM'un 115200 baud mesajı boot satırına yapışıyor, PC reseti görmüyor, ayarlar yüklenmiyordu | Firmware boot'tan önce `\n` basar; okuyucu satırı ilk `{`'den keser |
| Önemli | 1 s içinde yeniden bağlanma ya da reset olursa ayarlar gönderilmiyordu | Yeni bağlantıda sayaç sıfırlanır; boot her zaman ayarları yükler |
| Önemli | Kasnak eski GT2 40 diş (12.7 mm) değerindeydi | `params.json` HTD3M 20 diş (9.55 mm), `gains.h` yeniden üretildi. Enkoder 48 CPR motor kesinleşene kadar yer tutucu |
| Küçük | Tasarım hatası uyarısına rağmen eski K gönderiliyor / dışa aktarılıyordu | Hata varsa gönderim, dışa aktarma ve kaydetme durur |
| Küçük | Bir ayar reddedilince geri kalanı gönderilmiyordu; u_max ve x_ref arayüzde protokolden geniş | Her ayar ayrı gönderilir, gönderilemeyenler uyarıyla listelenir; u_max ≤ 48, \|x_ref\| ≤ 0.3 |
| Küçük | "Dondur" mod göstergesini de donduruyordu (FAULT görünmüyordu) | Yalnız grafikler donar |
| Küçük | Analiz simülasyonu x_ref'i yok sayıyordu | Cihaza gönderilen x_ref kullanılır |
| Küçük | Kayıt oynatma dürtme/duraklatmayı uyarısız yok sayıyordu; çubuğa tıklayınca geri atlıyordu | Uyarı eklendi; saat tıklamada yeniden kurulur |
| Küçük | Firmware: açılışta sahte (θ = 0) ilk telemetri; komut kontrolleri bayat örneğe bakıyordu; switch'te parazit filtresi yoktu; yarım kalan homing eski sıfırı geçerli bırakıyordu | Hepsi düzeltildi (sanal ESP32 de aynı) |
| Küçük | Sanal ESP32 `calib` olayını göndermiyordu; swingup'sız `gains.h` derlenmiyordu; Ts > 20 ms kabul ediliyordu | Düzeltildi |

**İkinci tur (dış inceleme, 22.11.2026):**
| Önem | Konu | Düzeltme |
|---|---|---|
| Önemli | `hb` arayüzün ana thread'inden gidiyordu: dengedeyken Analiz'de simülasyon (15 s'lik ~0.9 s) arayüzü dondurunca FAULT `link`, sarkaç düşüyordu (yeniden üretildi) | `hb` okuyucu thread'inden gönderilir; 60 s'lik analiz sırasında da denge sürüyor |
| Önemli | ACK'ler yalnız komut adıyla eşleşiyordu: bozuk satıra gelen `"?"` yanıtından sonra her onay bir önceki komuta bağlanıyordu (u_max=5 onaylandı, arayüz 12 V sandı; yeniden üretildi) | Yanıt bekleyen en eski komutla eşleşir, `"?"` onun yerini tutar; kayıp yanıt kalıcı kaymaya yol açmaz. Kayıtta komut, yanıtından önce yazılır |
| Küçük | Canlı sekmedeki "K'yı Cihaza Gönder" K'yı yeniden hesaplamadan gönderiyordu | Tasarım sekmesindeki gibi önce hesaplar |
| Ek önlem | Homing'de ters bağlantıda 12 V çarpma iddiası simülasyonda doğrulanmadı (araba ≤ 0.07 m/s sürünüp `homing_wrong_side` veriyor) | Yine de `homing_runaway` koruması: \|ẋ\| > 0.3 m/s ya da 0.3 s doyum → FAULT |
| Belge | Firmware README hâlâ "Pololu 25D" diyordu; enkoder 5 V uyarısı yoktu | Düzeltildi |
| Önemli | Dosya yazılamazsa (izin yok, kayıt klasörü) PyQt uygulamayı kapatıyordu; kapanınca bağlantı bekçisi motoru keser | Kayıt, gains.h ve parametre kaydında hata yakalanır; genel hata penceresi (`install_excepthook`, aynı anda tek pencere; her hata log alanına ve `kayitlar/hatalar.log` dosyasına da yazılır) uygulamayı açık tutar |
| Belge | Arayüz donarsa Esc/DURDUR işlemez; bozuk `hb` satırı bir komutun onayını tüketebilir | Kullanım kılavuzuna "USB'yi çekin" notu, protokole bilinen sınırlama notu |

**Üçüncü tur (bağımsız inceleme, 23.11.2026):**
| Önem | Konu | Düzeltme |
|---|---|---|
| Önemli | Sanal ESP32, sürtünme telafisi gönderildikten sonra `u_max` değişikliğini kontrolcüye uygulamıyordu (simülasyon firmware'den farklı sınırlıyordu) | `u_max` fizik ve kontrolcü modeline birlikte uygulanır |
| Önemli | Giriş türü force/acceleration iken K, yalnız DC motorla çalışan karta uyarısız gönderiliyordu (araba raydan kaçıyordu) | Yalnız `voltage` K'sı gönderilir, diğerinde açıklamalı ret |
| Küçük | Tasarım kutuları dosya değerlerini yuvarlıyordu (r_pulley 0.009549 → 0.00955, c −%17) | Kullanıcı değiştirmediyse dosyadaki tam değer korunur |
| Küçük | Bozuk alanlı bir komut (ör. `"F": null`) sanal ESP32'yi tamamen durduruyordu | `"?"` ile reddedilir, cihaz çalışmaya devam eder |
| Küçük | "Swing-up açık" deneme sırasında kapatılınca pompalama sürüyordu (belgeye aykırı) | Firmware ve sanal cihazda hemen WAIT'e geçer (açılınca SWING_UP) |
| Küçük | Reddedilen gönderimden sonra durum çubuğu cihazda olmayan K'yı gösteriyordu | Cihazın onayladığından farklıysa "(cihaza gönderilmedi)" yazar |
| Küçük | Animasyonlardaki hedef çizgisi gönderilmemiş değeri ya da hep 0'ı gösteriyordu | Cihaza giden / simülasyonun kullandığı x_ref çizilir |
| Küçük | Sürtünme sonucunu kabul etmek Tasarım'daki kaydedilmemiş düzenlemeleri uyguluyordu | Yalnız T_f güncellenir |
| Belge | `mimari.md`, `proje_durumu.md`, `parametre_tablosu.md` çelişkileri; README'de eski derleme/motor satırları | Düzeltildi |

**Dördüncü tur (düzeltmelerin bağımsız yeniden incelemesi, 23.11.2026):** üçüncü turun düzeltmeleri doğrulandı (swing-up anahtarı geçişleri, histerezis, analiz ve oynatma bozulmadı). Yeni bulgular:
| Önem | Konu | Düzeltme |
|---|---|---|
| Önemli | Yer tutucu parametrelerle gerçek kartta K gitmediği için swing-up kapalı bilgisi de gitmiyordu; kart resetlenince firmware varsayılanı (açık) geçerli oluyor, arayüz kapalı gösteriyordu (ilk donanım denemesi tam bu durum) | Swing-up anahtarı K engellerinden bağımsız, her bağlantıda ve her boot'ta gönderilir |
| Küçük | Kayıt oynatma özeti FAULT ile biten denemeyi BALANCE gösteriyordu | Denemenin bittiği mod (FAULT/IDLE) özette |
| Küçük | Tam değer koruması tam ortadaki değerlerde (0.0625) Qt ile Python'un farklı yuvarlaması yüzünden şaşıyordu | Kutunun kendi gösterdiği değerle karşılaştırılır |
| Küçük | Sürtünme sonucu kaydedilmemiş motor düzenlemelerini geri alıyordu | Yalnız durumda değişen alan güncellenir |
| Küçük | Sanal ESP32'nin BAŞLAT kontrolü ölçülen değil gerçek konumu kullanıyordu (ters enkoder test modunda yanlış ret) | Firmware gibi ölçülen konum/hız |
| Belge | Protokoldeki mod şemasında WAIT ve swing-up anahtarı geçişleri yoktu; FAULT koşulları "her mod" diye yazıyordu | Şema ve koşullar düzeltildi |

**Beşinci tur (23.11.2026):** dördüncü turun düzeltmeleri doğrulandı; yeni bulgular:
| Önem | Konu | Düzeltme |
|---|---|---|
| Önemli | Tam değer koruması aralık dışı dosya değerini kullanıyordu: kutu 48 gösterirken u_max 60 ile hesap/gönderim (dördüncü turun düzeltmesinin yan etkisi) | Tam değer yalnız kutunun aralığındaysa kullanılır |
| Küçük | K Hesapla'dan sonra yapılan kaydedilmemiş motor düzenlemesi sürtünme sonucuyla geri alınıyordu | Son uygulanan motor değerlerine göre yalnız değişen alan yazılır |
| Küçük | Canlı animasyondaki hedef çizgisi, karta gitmemiş x_ref'i gösterebiliyordu | Cihazın onayladığı x_ref çizilir |
| Küçük | Kayıt oynatma, gerçek kartın K'yı onaylamadığı denemelerde (yer tutucu) uyarı vermiyordu | Uyarı eklendi (gains.h'deki K çalışmış olabilir) |
| Belge | Yer tutucu uyarıları swing-up anahtarının yine gittiğini söylemiyordu; `mimari.md` ve sanal ESP32'deki bir açıklama eksik/yanlıştı | Düzeltildi |

**Altıncı tur (23.11.2026):** beşinci turun düzeltmeleri çalıştırılarak doğrulandı; **yeni kod hatası bulunmadı**. Belge: `kullanim_kilavuzu.md` ve bu dosyadaki "K ve tüm ayarlar otomatik yüklenir" cümlesi yer tutucu/giriş türü koşullarını içermiyordu, inceleme sayısı yanlıştı — düzeltildi. Ayrıca analizde hedef çizgisi yalnız başarılı simülasyondan sonra güncellenir.

**Yedinci tur (son kontrol, 23.11.2026):** son iki commit ve belgeler kodla karşılaştırıldı; **yeni kod hatası bulunmadı**. `mimari.md`'ye eksik "giriş türü voltage" koşulu ve inceleme sayısı düzeltildi; Tasarım sekmesi açılıştaki hesap başarısız olsa da sürtünme sonucunu (T_f) doğru eşitler.

**Tam proje denetimi (25.11.2026, PR #43–#45 sonrası main):** firmware ↔ sanal ESP32 ↔ supervisor ↔ protokol satır satır karşılaştırıldı, bütün araçlar yeniden çalıştırıldı; motoru yanlış anda çalıştıran bir hata bulunmadı. Düzeltilenler:
| Önem | Konu | Düzeltme |
|---|---|---|
| Küçük (güvenlik teşhisi) | Sol uç sensörü sürekli "basılı" okunursa (NC/TCST2103'te kopuk kablo, NO/NC karışıklığı) homing yanlış sıfırla "tamam" diyordu (BAŞLAT yine FAULT veriyordu) | Sola dayandıktan 5 cm sonra hâlâ basılıysa FAULT `homing_switch` (firmware + sanal ESP32) |
| Küçük | Zaten çalışan karta bağlanınca açılıştaki `calib` olayı kaçıyor, açı kalibrasyonunun sonucu görülemiyordu | `ping` yanıtında `angle_ok`; başarısızsa arayüz uyarır |
| Küçük | Bazı ray uzunluklarında (ör. 0.302 m) arayüzün sınıra kırptığı x_ref, float32 yuvarlaması yüzünden kartta reddediliyordu | Firmware ve sanal ESP32'de 1e-4 m tolerans |
| Küçük | "Ölçülmüş gerçek değerler" kutusu kaldırıldığında hesap başarısız olursa durum "ölçülmüş" kalabiliyordu (K onaysız gidebilirdi) | Güvenli yön (yer tutucu) hemen uygulanır; işaretleme başarısızsa kutu geri alınır |
| Küçük | `demo_kayit.py` bu ortamda `demo.mp4` yazıyordu, belgeler `demo.webm`'e bağlı | Önce WebM; olmazsa uyarı |
| Belge | `homing_travel` için "kopuk kablo" önerisi önerilen NC bağlantısında yanlıştı; işaret kontrolü 5. adımı ham gerilim komutu olmadığı için uygulanamıyordu; "%20 model hatasına dayanıklılık" rapor şekilleriyle çelişiyordu; test sayısı; `katki_3_6_13.md`'de kaldırılmış `E_target` | Düzeltildi |
| Belge | Hassasiyet raporu eski kasnakla (12.7 mm) üretilmişti; güncel parametrelerle sınıflar değişiyor | Yeniden üretildi |

**#50 Aşama 1 (θ sola pozitif), birinci tur (bağımsız inceleme, 25.11.2026):** denetçi hareket denklemlerini yeni kuralla kendisi türetti; model, doğrusallaştırma, swing-up, firmware formülleri ve `gains.h` doğru bulundu. Referans JSON'u eski kodla yeniden üretip birebir aynı çıktığını, 12 rastgele senaryoda (3 giriş türü, iki dürtme yeri) ideal sensörle farkın ≤ 2e-12 olduğunu, 5 işaret mutasyonunun testlerce yakalandığını ve gerçek bir eski CSV'nin doğru çevrildiğini doğruladı. Kritik/önemli bulgu yok. Düzeltilenler:
| Önem | Konu | Düzeltme |
|---|---|---|
| Küçük (önleyici) | Firmware sürüm adı değişmemişti; PC eski kurallı (θ sağa +) firmware'i ayırt edemez, ona K gönderirse θ geri beslemesi ters çalışırdı | Firmware `ref-1.2`, sanal ESP32 `mock-1.2`; boot ve ping'de `"theta":"sol"`. Bildirmeyen ya da farklı bildiren cihaza arayüz K ve ayarları göndermez (swing-up anahtarı yine gider), uyarır |
| Küçük | Eski CSV dönüşümü θ taşıyan komut alanlarını (eski biçim kick `dth`, reset `th0_deg`) çevirmiyordu; dönüşüm arayüzde görünmüyordu | İkisi de çevrilir; Kayıt Oynatma özetinde "eski θ kuralıyla alınmış kayıt, çevrildi" notu; başlıksız CSV'nin eski kural sayıldığı belgelendi |
| Belge | Test sayısı; `mekatronik_on_degerlendirme.md`'de yeniden üretilen şekillerle uyuşmayan max \|x\| (0.142 → 0.143 m); `katki_10.md`'deki motorlu model K'ları eski 12.7 mm kasnakla hesaplanmıştı; referans senaryo dosyasında T_f açıklaması | Düzeltildi; K'lar güncel `params.json` ile yeniden hesaplandı |

**#50 Aşama 1, ikinci tur (25.11.2026):** birinci turun düzeltmeleri doğrulandı (tampon boyutları, protokol ↔ kod, arayüzün bütün K gönderim yolları). Sanal ESP32'de calib → homing → swing-up → denge → dürtmeler → x_ref → reset senaryosu eski ve yeni kodla ayna başlangıçla koşuldu: 7856 telemetri satırında x, ẋ, u ve mod aynı, θ, θ̇ ters. Bulgular:
| Önem | Konu | Düzeltme |
|---|---|---|
| Önemli | #50 öncesi dışa aktarılmış bir `gains.h` yeni firmware'le hatasız derleniyordu; onda `LQR_K`'nın θ, θ̇ kazançları ve Kalman matrisleri ters işaretli kalır. Kalman matrisleri yalnız `gains.h`'den geldiği için PC düzeltemez ve kart `"theta":"sol"` bildirdiğinden uyarı çıkmazdı (simülasyonda: eski Kalman matrisleriyle sarkaç düşüyor, araba ray sonuna gidiyor) | `gains.h`'ye `#define THETA_POSITIVE_LEFT 1`; firmware bu makro yoksa `#error` ile derlenmez. `gains.h` yeniden üretildi, test denetliyor |
| Küçük | θ engeli cihaz bilgisi gelene kadar kapalıydı: ping yanıtı gelmeden ya da `fw` alanı olmayan bir boot/ping'den sonra K gidiyordu (protokol.md ile çelişki) | K yalnız cihaz `"theta":"sol"` bildirdikten sonra gider; bilgi henüz yoksa ayrı uyarı. `{"Ts": …}` ile çağıran eski testler cihaz bilgisini tam verecek şekilde güncellendi |
| Küçük | Eski firmware'e bağlıyken alınan kayıt `theta_pozitif: sol` diye işaretleniyordu (oynatma ayna görüntüsü olurdu); uyarı grafiklerin ters olduğunu söylemiyordu | Kayıt başlığına `theta_pozitif: sag` yazılır (okurken çevrilir); uyarı metni genişletildi |
| Belge | Kılavuzda otomatik yükleme koşulu ve sorun giderme satırları, firmware README'de `ref-1.2` / `theta` alanı / eski `gains.h` notu yoktu | Eklendi |

**#50 Aşama 1, üçüncü tur (25.11.2026):** ikinci turun düzeltmeleri doğrulandı; sanal ESP32 ile pencere baştan sona sürüldü (bağlan → K bilgi gelmeden reddediliyor → boot/ping → K otomatik gidiyor ve onaylanıyor; reset ve yeniden bağlanma; `theta` göndermeyen cihazda K gitmiyor). Kart için tehlikeli bulgu yok. Düzeltilenler:
| Önem | Konu | Düzeltme |
|---|---|---|
| Küçük | Eski firmware'e bağlıyken alınan kayıtta (`theta_pozitif: sag`) okuma, PC'nin yeni kuralla yazdığı K'ları da çeviriyordu; karşılaştırma kararsız K ile koşuyordu | PC alanları her zaman `theta_pozitif: sol`; cihazın kuralı ayrı alanda (`cihaz_theta_pozitif`), `sag` ise yalnız θ, θ̇ sütunları çevrilir. Test eklendi |
| Küçük | Kayıt oynatmadaki ayar anlık görüntüleri `theta_pozitif`'i etkin meta'ya geri yazıyordu (bugün zararsız) | θ alanları birleştirmede atlanır |
| Küçük | Ping yanıtı kaybolursa cihaz bilgisi hiç gelmiyor, K hiç gitmiyordu (güvenli ama kullanılamaz) | Bilgi yokken gönderim denemesi ping'i yeniden ister, mesaj "olmazsa yeniden bağlanın" der |
| Küçük | Cihaz bilgisi gelmeden başlatılan kayıt yanlış θ kuralıyla işaretlenebilirdi | Bilgi gelmeden kayıt başlatılmaz (ping yeniden istenir) |
| Belge | Firmware'deki "eski gains.h" yedek yorumları ve `katki_14_15.md` artık yanıltıcıydı (#50 öncesi `gains.h` `#error` verir); recorder açıklamaları yeni durumu anlatmıyordu | Güncellendi |

**#50 Aşama 1, dördüncü tur (25.11.2026):** üçüncü turun düzeltmeleri gerçek arayüzle sanal ESP32 üzerinden doğrulandı (kayıt bilgi beklemesi, ping yeniden isteme, eski firmware benzetiminde kayıt ve oynatma, `demo_kayit.py`). Kritik/önemli bulgu yok. Tek bulgu (Küçük, tutarlılık): tanınmayan bir `cihaz_theta_pozitif` değeri (yalnız elle düzenlenmiş dosyada olur) sessizce "sol" sayılıyordu; artık `theta_pozitif` gibi `ValueError` verir, test eklendi.

**#50 Aşama 1, beşinci tur (son kontrol, 25.11.2026):** bütün diff taze gözle incelendi; denklemler yeniden türetildi, eski ve yeni kod ayna başlangıçla karşılaştırıldı (ideal sensörle fark ≤ 1e-13), `gains.h` yeniden üretilip birebir aynı bulundu. **Yeni kod hatası bulunmadı.** Belge: `protokol.md`'deki `set_gain` örneği eski kuraldaki K'yı gösteriyordu (kopyalanıp gönderilse sarkaç düşerdi; main'de de böyleydi); `mimari.md`'de otomatik yüklemenin θ koşulu eksikti; Analiz sekmesinde bir ipucu eski "iki işaret" ifadesini taşıyordu. Düzeltildi.

**#50 Aşama 2 (mekatronik parametreleri), birinci tur (25.11.2026):** denetçi değerleri Tablo 15–16 ve rapordaki toplamlarla (M_tot = 2.73 kg, ω_n) karşılaştırdı; firmware ↔ `filters.py` ↔ `gains.h` ↔ `params.json` zinciri, `gains.h`, rapor şekilleri ve hassasiyet raporu yeniden üretilip birebir aynı bulundu. Kt = Ke seçiminin issue'daki R = 2.18 önerisiyle aynı α ve b'yi verdiği (K farkı ≤ 1.4e-4) ve 3 N'luk ölçülmemiş telafinin gerçek sürtünme 0–4.5 N iken de kalkışı bozmadığı doğrulandı. Kritik/önemli bulgu yok. Düzeltilenler:
| Önem | Konu | Düzeltme |
|---|---|---|
| Küçük | `E_ratio`'suz eski kayıtlar oynatmada yeni varsayılanla (1.05) koşuyordu | Başlıkta `E_ratio` yoksa 1.0 (eski varsayılan); test |
| Küçük | `params.py` varsayılanları artık `params.json`'dan farklı (10:1, 12.7 mm); `params.json` okunamazsa arayüz bunlarla açılıyor ve dışa aktarılan `gains.h` 44 CPR'lı firmware'de x'i 2.5 kat yanlış ölçeklerdi | Dosya okunamaz/yoksa K gönderilmez, `gains.h` dışa aktarılmaz; bir parametre dosyası yüklenince kalkar. Docstring düzeltildi |
| Küçük | Hassasiyet aracı yakalama/bırakma açılarını ortak sabitten almıyordu | `CATCH_DEG` / `RELEASE_DEG` |
| Belge | Rapor şekillerinde elle yazılı "`l` KRİTİK" cümlesi yeni hassasiyet sonucuyla çelişiyordu; `mekatronik_on_degerlendirme.md` §3–4'te eski sayılar ve kaynak; §6 ve motor yorumlarında "T_f = 0"; firmware README'de telafinin nasıl gittiği | Cümle kaldırıldı, şekiller yeniden üretildi; belgeler güncellendi; ilk açılış kontrol listesine "Sürtünme Ölç" adımı |

**#50 Aşama 2, ikinci tur (26.11.2026):** birinci turun düzeltmeleri doğrulandı; değerler PDF'le yeniden karşılaştırıldı, `gains.h` birebir. **Yeni kod hatası yok.** Kalan küçük kenarlar düzeltildi: parametre dosyası okunamadığında "Kaydet" koruma bayrağını kaldırıyordu (genel varsayılanlar `params.json`'a yazılıp K gidebilirdi) → artık yalnız Yükle kaldırır; firmware README'de eski CPR hatasının yönü (x ~1.74 kat büyük okunurdu).

**#50 Aşama 3 (swing-up karar çalışması), birinci tur (26.11.2026):** denetçi kuralların Supervisor'a (Kalman bölgesinin release açısı dahil), ray yarı boyunun hem fiziğe hem kontrolcüye uygulandığını, telafi kapalının doğru modellendiğini ve 900 koşunun tam çarpım olduğunu doğruladı; JSON'dan rapor birebir yeniden üretildi, 4 koşu tek tek tekrarlandı. **Yeni kod hatası yok.** Düzeltilenler (Küçük/Belge): "bizim" kural elle yazılıydı → ortak sabitlerden; "dike geldi" sebebi artık hangi şartın tutmadığını yazıyor (hepsi \|θ̇\|); "kalkan koşularda en büyük max \|x\|" yalnız kalkan koşulardan; nominal koşunun da gürültülü olduğu (ön değerlendirmeyle birebir karşılaştırılamaz) notu; proje_durumu özetindeki iki cümle tablolara göre düzeltildi, alt başlık ve belge tablosu satırı; süre tek değer (~6 dk). Tarama yeniden koşuldu; sağlamlık sonuçları değişmedi.

**#50 Aşama 1–5 ortak inceleme (26.11.2026, ahmetselimko):** Aşama 1'in θ işaret testi kuantalı sensörde 1 ulp'lik farkın kaydırdığı bir adım yüzünden Windows'ta düşüyordu (eski kod aynı makinede referansla 6e-9 m içinde) → adım kaydıktan sonrası gevşek tolerans. Aşama 2: protokoldeki `set_gain` örneği ve `katki_10.md` K tablosu eski parametrelerleydi. **Aşama 3: karar aracı LQR'ın bırakıp swing-up'ın yeniden kaldırdığı koşuları "kalktı" sayıyordu → `classify` ile yeniden üretildi; "+50 g ve +20 g'de 1.10 sağlam" gibi sonuçlar düştü, özet yukarıda güncellendi.**

Sanal cihazda gözlenen, hata olmayan bir etki (Aşama 2 öncesi parametrelerle): motor sürtünmesi (T_f) 0 olduğu için homing bittiğinde sallanan sarkaç arabayı serbestçe kaydırıyor ve sürtünme ölçümü "sarkaç hareketsiz olmalı" diye reddediliyordu. Aşama 2'nin `params.json`'ında T_f 3 N olduğundan artık kabul ediliyor.

---

## 5. Artılar

* **Doğru mimari:** zaman kritik döngü mikrodenetleyicide; PC tarafının hızı dengeyi etkilemez.
* **Donanımsız geliştirme:** sanal ESP32 sayesinde arayüz, protokol ve kontrol mantığı kart gelmeden uçtan uca denendi.
* **Tek kaynaktan tutarlılık:** aynı kontrol mantığı Python'da, sanal cihazda ve firmware'de var. Parametre aralıkları ve komut listesi testle eşleştiriliyor.
* **Gerçekçi simülasyon:** 12-bit açı ve 44 CPR × 19 enkoder kuantalaması, türevle ve Kalman ile hız kestirimi, gecikme, gürültü ve model uyumsuzluğu dahil. Swing-up kazançları ±0.4 m'lik kısa ray için taranarak seçildi.
* **Doğrulanmış matematik:** LQR sonuçları python-control ile aynı; analitik model sayısal jacobian ile aynı; bağımsız incelemede türetmeyle de karşılaştırıldı.
* **Güvenlik öncelikli:** homing yapılmadan başlatılamaz; ray sınırı, limit switch, sensör hatası ve acil stop'ta motor kapanır. Esc tuşu her pencerede acil stop'tur.
* **Kolay güncelleme:** gerçek ölçüler gelince yalnızca `config/params.json` değişir; K, simülasyon ve `gains.h` arayüzden tek tuşla yenilenir.
* **Belgelenmiş sözleşme:** iki ekip neyi nasıl göndereceğini `protokol.md`'den okur.

## 6. Eksiler, sınırlamalar ve riskler

* **Firmware gerçek kartta hiç denenmedi.** Derlendiği doğrulandı; donanıma özgü sorunlar (pin seçimi, sürücü davranışı, enkoder gürültüsü) olabilir.
* **Parametreler tasarım değeri, ölçüm değil.** Swing-up kazançları ve LQR ağırlıkları ölçülmüş değerlerle yeniden ayarlanmalı. Uç kütlesi kararı ve raydaki gerçek yol bekleniyor.
* **Motor değerleri rapordan türetildi:** JGB37-520'nin R, Kt, Ke'si Tablo 16'daki α ve b'den, enkoder (44 CPR) 836 sayım/tur'dan; datasheet ya da ölçümle doğrulanmalı. FIT0186 + 20 diş simülasyonda sarkacı kaldıramıyor (§3).
* **12-bit açı sensörünün türevi gürültülü:** dengede motor geriliminde ±1–1.5 V titreşim görülüyor. Filtre daha da kısılırsa yakalama başarısız oluyor. **Çözüldü (#14):** dik bölgede Kalman filtresi kullanılıyor; simülasyonda σ(u) 0.63 V → 0.21 V. Firmware'de `USE_KALMAN 0` ile kapatılabilir (bkz. `docs/katki_14_15.md`).
* **Step motor desteği yarım:** modelde ve simülasyonda var, firmware'de yok.
* **Model sadeleştirmeleri:** kayış esnekliği ve motor endüktansı ihmal ediliyor; motorun kuru (Coulomb) sürtünmesi `T_f` (rapordaki 3 N), rotor ataleti (yansıyan 1.346 kg) ve viskoz sürtünmesi `B_m` (0) simülasyonda var; tasarım değerleri, ölçülmedi (Coulomb sürtünmesi LQR tasarımında ihmal edilir). Gerçek sistemde kazançlar biraz farklı çıkabilir.
* **Donanımsal acil stop yok:** limit switch'ler ve yazılımsal ray sınırı motoru ESP32 üzerinden durdurur. Firmware takılırsa veya sürücü arızalanırsa (motoru tam güçte bırakırsa) arabayı hiçbir şey durdurmaz. Motor beslemesine elle kesilebilen bir anahtar/fiş önerilir.
* Arayüz Windows'ta denendi; Linux/macOS'ta denenmedi (PyQt5 çapraz platform olduğu için sorun beklenmiyor).

## 7. Yapılması gerekenler

### Mekatronik ekibi
1. Motor seçimini kesinleştirmek (JGB37-520 mi FIT0186 mı) ve datasheet'ini vermek (durma torku/akımı, boşta akım, redüktör oranı, enkoder CPR). Ray boyunu bildirmek. MPU6050'nin ne için kullanılacağını ve uç sensörü olarak switch mi optik sensör mü kullanılacağını belirtmek.
2. Mekanik tasarım bitince `docs/parametre_tablosu.md`'yi doldurmak (m, M, l, I, sürtünmeler, motor sabitleri, rotor ataleti).
3. Firmware'i karta (ESP32-S3-DevKitC-1, "COM" USB portu) yükleyip `board_pins.h`'deki S3 pinlerine göre kablolamak; ilk açılış kontrol listesini uygulamak (`firmware/esp32_pendulum/README.md`): işaret kontrolü → homing → yalnız LQR → swing-up.
4. Uç sensörleri (NC switch ya da TCST2103) ve motor beslemesini elle kesmenin kolay bir yolu (acil stop butonu olmayacak).

### Şimdi başlanabilecek işler (mekatronik verisi beklemeyen)
Aşağıdaki işler gerçek ölçüleri ya da mekanizmayı beklemeden, yer tutucu değerlerle hemen yapılabilir. Projeye yeni katılanlar için iyi bir başlangıçtır. İlk ikisi önceliklidir.

| # | İş | Nasıl yapılır | Faydası |
|---|---|---|---|
| 1 | ~~**Motor karşılaştırması**~~ ✔ *yapıldı* | `python tools/motor_karsilastirma.py` tüm adaylarla θ₀ = 180° swing-up'ı dener; sonuç ve tablo issue #7'de | Gerçek ölçüler gelince aynı betik yeniden çalıştırılır |
| 2 | ~~**Firmware'i derlemek**~~ ✔ *yapıldı* | Core 3.3.12 ve 2.0.17 ile hatasız derleniyor; sürümler `firmware/esp32_pendulum/README.md`'de | Derleme her push'ta GitHub Actions'ta tekrarlanıyor |
| 3 | ~~**Hassasiyet analizi**~~ ✔ *yapıldı (#13)* | `python tools/hassasiyet_analizi.py`; sonuç `docs/hassasiyet_raporu.md` (25.11.2026, Aşama 2 parametreleri, Kalman + 44 CPR). Gerçek parametreler gelince yeniden çalıştırılmalı | Mekatronik ekibe "şunları özellikle dikkatli ölçün" listesi |
| 4 | ~~**Kalman filtresi**~~ ✔ *yapıldı (#14)* | `pendulum_lab/filters.py::KalmanFilter` ile hız kestirimi simülasyonda denenir, motor gerilimi titreşimi ölçülür, sonra firmware'e taşınır | Dengedeki ±1–1.5 V titreşimi azaltır. Gerçek değerler gelince yalnızca sayılar değişir |
| 5 | ~~**Kayıt oynatma sekmesi**~~ ✔ *yapıldı (#15)* | Arayüze CSV kaydını açıp aynı başlangıç koşullu simülasyonla üst üste çizen bir sekme eklenir; sanal cihazdan alınan kayıtla denenir | Gerçek denemeler başlayınca modelin doğrulanmasını kolaylaştırır |
| 6 | ~~**Matlab karşılaştırması**~~ ✔ *yapıldı (#37)* | Matlab dosyaları `matlab/`'da; doğrusal ve doğrusal olmayan model testle karşılaştırılıyor; Analiz sekmesinde Matlab uyumlu deneme; sonuçlar `katki_10.md`'de | İki ekip aynı modeli kullandığından emin olur |
| 7 | **Masada sensör testi** *(elde ESP32 + AS5600 varsa)* | Mekanizma olmadan ESP32'ye AS5600 bağlanıp açı okuma ve arayüzle haberleşme gerçek kartla denenir | Firmware'in sensör ve haberleşme kısmı donanım gelmeden doğrulanır |

### Bilgisayar ekibi (veri geldikten sonra)
1. Gerçek parametreler gelince: K'yı yeniden hesaplamak, aşağıdan swing-up simülasyonunu tekrarlamak, gerekirse swing-up kazançlarını ayarlamak, `gains.h` üretmek.
2. İlk donanım denemesinde mekatronik ekiple birlikte olmak; CSV kayıtlarından simülasyonla gerçek davranışı karşılaştırıp modeli düzeltmek (sürtünme vb.).
3. Kalman filtresinin (#14) gerçek kartta titreşimi azalttığını doğrulamak; model tutmazsa `USE_KALMAN 0` ile kapatıp parametreleri düzeltmek.
4. Step motor seçilirse firmware'in motor sürme kısmını yazmak.
5. İsteğe bağlı: Wi-Fi/WebSocket ile kablosuz telemetri (ESP32'de mümkün), otomatik parametre tanılama (kayıttan m, b, c tahmini).
