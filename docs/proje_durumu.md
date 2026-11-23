# Proje Durum Raporu — Bilgisayar Mühendisliği Kısmı

*Ters Sarkaç Disiplinlerarası Projesi · Ekim–Kasım 2026 (son güncelleme 18.11.2026)*

Bu rapor bilgisayar ekibinin şimdiye kadar yaptıklarını, mekatronik ekibinin üzerindeki işleri, çözümün güçlü ve zayıf yönlerini ve sırada ne olduğunu özetler.

---

## 1. Kısa özet

* Bilgisayar tarafı **çalışır durumda**: Matlab'in yerini tutan bir Python kütüphanesi, canlı izleme ve kontrol arayüzü, donanım olmadan deneme yapmayı sağlayan bir sanal ESP32, mekatronik ekibe başlangıç olarak bir ESP32 firmware'i ve iki ekip arasındaki haberleşme sözleşmesi hazır.
* Sanal cihazla uçtan uca deneme başarılı: kalibrasyon (homing) → swing-up → LQR ile denge. Veri 200 Hz'de kayıpsız geliyor, grafik ~60 FPS çiziliyor.
* **264 otomatik test** geçiyor; firmware her push'ta GitHub Actions'ta iki kart (ESP32-S3, klasik ESP32) × iki core sürümüyle derleniyor. Kod art arda bağımsız incelemelerden geçti (§4: ilk tur 21, doğrulama turu 6 bulgu; §4.2: 22–23.11.2026 tarihli turlar); bulunanların hepsi düzeltildi, son iki turda yeni kod hatası çıkmadı. 18.11.2026'da belgeler kodla ayrıca karşılaştırıldı (§4.1).
* Fiziksel parametreler henüz **yer tutucu**. Donanım BOM v3'e (13.11.2026) göre güncellendi: kart ESP32-S3-DevKitC-1 N16R8, kasnak HTD3M 20 diş; motor JGB37-520 (mekatronik ara raporunda son seçim); datasheet'i gelmediği için motor değerleri hâlâ eski 37D yer tutucusu. Gerçek değerler gelince yalnızca bir dosya (`config/params.json`) değişecek.
* Mekatronik ekibinin Matlab/Simulink dosyaları `matlab/` klasöründe; model Python kütüphanesiyle θ işareti dışında birebir aynı (testle doğrulanıyor).

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
| `sim/simulator.py` | Lineer olmayan simülasyon: ESP32 gibi 500 Hz ayrık kontrol, AS5600 12-bit ve 48 CPR enkoder kuantalaması, Kalman, gürültü, gecikme, dürtme, ray uçları | `ode45` |
| `sim/matlab_compat.py` | Matlab'ın θ işaretine dönüşüm (K ve durum), `cart_pendulum_application.m` ile aynı deneme | — |
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
| `hassasiyet_raporu.md` | Herkes | Parametre hassasiyet taramasının sonuçları (22.11.2026'da Kalman ve 48 CPR enkoderle yeniden üretildi) |
| `../matlab/README.md` | İki ekip | Matlab dosyaları, θ işareti farkı, K dönüşümü |
| `media/rapor/README.md` | Herkes | Rapor şekilleri (aşağıdan kalkış, dürtme, kutup haritası, model uyumsuzluğu) ve sayıları; `python tools/rapor_sekilleri.py` ile yeniden üretilir |

---

## 3. Mekatronik ekibinde neler var / neler bekleniyor

Toplantı notları ve tahtadaki plana göre:

| Alan | Durum | Bilgisayar tarafına etkisi |
|---|---|---|
| **Mekanik tasarım:** tabla mekanizması, kayış-kasnak, sigma profil çerçeve, rulman ve köşebentler | Inventor modeli hazırlanıyor | Bittiğinde m, M, l ve atalet `parametre_tablosu.md`'ye girilecek. Inventor "Physical" sekmesi I değerini doğrudan verir |
| **Motor seçimi** | **Seçildi: JGB37-520 12 V 530 rpm** (mekatronik ara raporu, "final actuator"). BOM v3'teki diğer aday DFRobot FIT0186 (12 V, 251 rpm, 43.8:1, 16 CPR, durma 7 A / 18 kg·cm). Önceki "Pololu 25D 251 RPM" bilgisi aslında FIT0186'yı kastediyordu (o da 37 mm gövdeli, Pololu değil). Kasnak **HTD3M 20 diş** (r ≈ 9.55 mm, mekatronik doğruladı) | **BOM simülasyonu (22.11.2026, `python tools/motor_karsilastirma.py --sadece BOM`, yer tutucu sarkaç/araba, iki ray × iki J_rotor × sürtünmeli/sürtünmesiz = 8 deneme):** JGB37-520 + 20 diş (v_max ≈ 0.56 m/s) durma torku 3 kg·cm varsayımıyla **8/8**, 6 kg·cm varsayımıyla **4/8** (başarısızlar raydan taşma: swing-up kazançları bu motora göre ayarlanmamış); 40 dişle iki varsayımda da 8/8. FIT0186 + 20 diş (v_max ≈ 0.26 m/s) **0/8**, 40 dişle 3/8 (sürtünmeli denemelerin hepsi başarısız). Sonuç: **JGB37-520 + 20 diş uygulanabilir görünüyor, FIT0186 + 20 diş değil.** JGB37-520'nin tork/akım/enkoder değerleri BOM'da yok, VARSAYIM; datasheet gelince araçtaki `BOM_ADAYLAR` güncellenip yeniden çalıştırılmalı. Sarkaç uzunluğu ve kütleleri değişirse sonuç değişir. *Önceki karşılaştırmalar (37D / 25D / Pololu 35 / NEMA17) aynı araçta duruyor; ayrıntı #7* |
| **Sarkaç kütlesi: küp mü küre mi** | Karar verilmedi | Geometri hesaplayıcı ikisini karşılaştırır. Uzun çubukta fark küçüktür; belirleyici olan ağırlık merkezi uzaklığı l'dir. Montaj kolaylığına göre seçilebilir |
| **Sistem modelleme ve lineerleştirme** (θ = 0'da, parametrik) | Matlab dosyaları geldi (`matlab/`) | Kütüphanede hazır ve testli (analitik + sayısal jacobian). Mekatronik ekibin Matlab modeliyle doğrusal ve doğrusal olmayan denklemler θ işareti dışında birebir aynı (`tests/test_matlab_compat.py`). İşaret kuralı henüz birleştirilmedi (#1): Matlab'da θ sola, projede sağa pozitif; K'yı dönüştürmek için `gain_from_matlab` |
| **Kontrol:** LQR kararı, enerji bazlı swing-up araştırması | LQR seçildi, swing-up araştırılacak | İkisi de uygulandı ve simülasyonda çalışıyor: aşağıdan ~4 s'de kalkış, ±0.2 m ray kullanımı, %20 model hatasında da yakalama. Kazançlar araştırmaya başlangıç noktası olarak kullanılabilir |
| **Sensör / mikrodenetleyici / sürücü** | **BOM v3:** ESP32-S3-DevKitC-1 **N16R8**, AS5600, BTS7960B; ayrıca **MPU6050** listede | **Kart klasik ESP32'den S3'e değişti:** eski pinlerin çoğu (22, 25, 26, 27, 32, 33) S3 N16R8'de yok ya da flash/PSRAM'e bağlı. Pinler `firmware/esp32_pendulum/board_pins.h`'e taşındı; S3 ve klasik ESP32 için ayrı setler, derleme hedefinden otomatik seçilir, CI ikisini de derler. **MPU6050'nin rolü teyit bekliyor:** firmware açıyı AS5600'den okuyor; MPU6050 sarkaç açısı için kullanılacaksa (ivmeölçer dönen sarkaçta bozulur, jiroskop kayar) açı okuma ve Kalman kısmı yeniden ele alınmalı. Aynı I2C hattına bağlanabilir (0x68 ≠ 0x36) |
| **Uç sensörü** | **BOM v3:** 2 × DC163-3 micro switch ve 2 × TCST2103 yarık optik sensör listede; notta "ya biri ya diğeri". Hangisi kullanılacak, teyit bekliyor | Firmware ikisini de değişiklik olmadan destekliyor (`LIMIT_SWITCHES 1`, `LIM_PRESSED = HIGH`): NC switch'te ve pull-up'lı TCST2103'te tetiklenme ve kopuk kablo HIGH okunur. TCST2103 bağlantısı (LED direnci, harici pull-up) firmware README'de |
| **Donanımsal acil stop** | **Yok** (16.11.2026: "limitler aşılınca otomatik durdurulacak") | Durdurma yazılıma (ESP32) bağlı; motor beslemesi elle kolayca kesilebilmeli |
| **Homing'i kim başlatacak** | Kapandı | Kart açılınca bekler; kalibrasyon yalnızca arayüzdeki KALİBRE ET ile, sol switch'i bularak (`protokol.md` §6) |

---

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

Küçük, bilinen fark: sanal ESP32'de homing yol sınırı hatası (`homing_travel`) yok (firmware'de var). Sürtünme ölçümünde uç switch'i ve yazılımsal ray sınırı ikisinde de denetlenir; `calib` olayı 22.11.2026'da eklendi. `hassasiyet_raporu.md` 22.11.2026'da Kalman ve 48 CPR enkoderle yeniden üretildi: 168 deneyden başarılı olanlar (120) ve parametre sınıfları değişmedi; raporun ayarlar bölümündeki elle yazılmış eski enkoder çözünürlüğü de düzeltildi.

### 4.2 Uçtan uca test ve bağımsız inceleme (22–23.11.2026)
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

Sanal cihazda gözlenen, hata olmayan bir etki: motor sürtünmesi (T_f) 0 olduğu için homing bittiğinde sallanan sarkaç arabayı serbestçe kaydırıyor ve sürtünme ölçümü "sarkaç hareketsiz olmalı" diye reddediliyor. Gerçek redüktörlü motorda bu kayma olmaz.

---

## 5. Artılar

* **Doğru mimari:** zaman kritik döngü mikrodenetleyicide; PC tarafının hızı dengeyi etkilemez.
* **Donanımsız geliştirme:** sanal ESP32 sayesinde arayüz, protokol ve kontrol mantığı kart gelmeden uçtan uca denendi.
* **Tek kaynaktan tutarlılık:** aynı kontrol mantığı Python'da, sanal cihazda ve firmware'de var. Parametre aralıkları ve komut listesi testle eşleştiriliyor.
* **Gerçekçi simülasyon:** 12-bit açı ve 48 CPR enkoder kuantalaması, türevle ve Kalman ile hız kestirimi, gecikme, gürültü ve model uyumsuzluğu dahil. Swing-up kazançları ±0.4 m'lik kısa ray için taranarak seçildi.
* **Doğrulanmış matematik:** LQR sonuçları python-control ile aynı; analitik model sayısal jacobian ile aynı; bağımsız incelemede türetmeyle de karşılaştırıldı.
* **Güvenlik öncelikli:** homing yapılmadan başlatılamaz; ray sınırı, limit switch, sensör hatası ve acil stop'ta motor kapanır. Esc tuşu her pencerede acil stop'tur.
* **Kolay güncelleme:** gerçek ölçüler gelince yalnızca `config/params.json` değişir; K, simülasyon ve `gains.h` arayüzden tek tuşla yenilenir.
* **Belgelenmiş sözleşme:** iki ekip neyi nasıl göndereceğini `protokol.md`'den okur.

## 6. Eksiler, sınırlamalar ve riskler

* **Firmware gerçek kartta hiç denenmedi.** Derlendiği doğrulandı; donanıma özgü sorunlar (pin seçimi, sürücü davranışı, enkoder gürültüsü) olabilir.
* **Parametreler yer tutucu.** Swing-up kazançları ve LQR ağırlıkları gerçek değerlerle yeniden ayarlanmalı.
* **Motor değerleri bekleniyor:** motor JGB37-520 seçildi ama tork, akım, redüktör ve enkoder değerleri (datasheet) bilinmiyor; simülasyon varsayımla çalışıyor. Firmware'deki `ENC_CPR_MOTOR` (48) ve modeldeki 37D yer tutucusu motor kesinleşince güncellenmeli. FIT0186 + 20 diş simülasyonda sarkacı kaldıramıyor (§3).
* **12-bit açı sensörünün türevi gürültülü:** dengede motor geriliminde ±1–1.5 V titreşim görülüyor. Filtre daha da kısılırsa yakalama başarısız oluyor. **Çözüldü (#14):** dik bölgede Kalman filtresi kullanılıyor; simülasyonda σ(u) 0.63 V → 0.21 V. Firmware'de `USE_KALMAN 0` ile kapatılabilir (bkz. `docs/katki_14_15.md`).
* **Step motor desteği yarım:** modelde ve simülasyonda var, firmware'de yok.
* **Model sadeleştirmeleri:** kayış esnekliği ve motor endüktansı ihmal ediliyor; motorun kuru (Coulomb) sürtünmesi `T_f`, viskoz sürtünmesi `B_m` ve rotor ataleti simülasyonda var ama değer girilene kadar 0 (Coulomb sürtünmesi LQR tasarımında ihmal edilir). Gerçek sistemde kazançlar biraz farklı çıkabilir.
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
| 3 | ~~**Hassasiyet analizi**~~ ✔ *yapıldı (#13)* | `python tools/hassasiyet_analizi.py`; sonuç `docs/hassasiyet_raporu.md` (22.11.2026, Kalman + 48 CPR). Gerçek parametreler gelince yeniden çalıştırılmalı | Mekatronik ekibe "şunları özellikle dikkatli ölçün" listesi |
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
