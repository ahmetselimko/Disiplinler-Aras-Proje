# Proje Durum Raporu — Bilgisayar Mühendisliği Kısmı

*Ters Sarkaç Disiplinlerarası Projesi · Ekim 2026*

Bu rapor bilgisayar ekibinin şimdiye kadar yaptıklarını, mekatronik ekibinin üzerindeki işleri, çözümün güçlü ve zayıf yönlerini ve sırada ne olduğunu özetler.

---

## 1. Kısa özet

* Bilgisayar tarafı **çalışır durumda**: Matlab'in yerini tutan bir Python kütüphanesi, canlı izleme ve kontrol arayüzü, donanım olmadan deneme yapmayı sağlayan bir sanal ESP32, mekatronik ekibe başlangıç olarak bir ESP32 firmware'i ve iki ekip arasındaki haberleşme sözleşmesi hazır.
* Sanal cihazla uçtan uca deneme başarılı: kalibrasyon (homing) → swing-up → LQR ile denge. Veri 200 Hz'de kayıpsız geliyor, grafik ~60 FPS çiziliyor.
* **75 otomatik test** geçiyor. Kod iki tur bağımsız incelemeden geçti: ilk turda bulunan 21 bulgu ve doğrulama turunda çıkan 6 yeni sorun düzeltildi (bkz. §4).
* Fiziksel parametreler henüz **yer tutucu**. Gerçek değerler gelince yalnızca bir dosya (`config/params.json`) değişecek.

**En önemli tasarım kararı:** Sarkacı dengede tutan döngü (saniyede 500 kez) **ESP32 üzerinde** çalışır, bilgisayarda değil. USB gecikmesi ve Windows'un düzensiz zamanlaması, döngü PC'de olsaydı hangi dille yazılırsa yazılsın sarkacı düşürürdü. Bilgisayar zaman açısından kritik olmayan işleri yapar: K'yı hesaplamak ve göndermek, izlemek, kaydetmek, simülasyon yapmak.

---

## 2. Bilgisayar tarafında neler var

### 2.1 `pendulum_lab` kütüphanesi (Matlab karşılığı)
| Modül | Ne yapar | Matlab karşılığı |
|---|---|---|
| `model/params.py` | Fiziksel parametreler (JSON'dan okunur). Çubuk + küre/küp uç kütleden m, l, I hesabı. Rotor ataletinin eşdeğer kütlesi | — |
| `model/dynamics.py` | Lineer olmayan araba-sarkaç denklemleri (Lagrange). DC motor, ideal kuvvet ve step motor (ivme) girişleri | Simulink modeli |
| `model/linearize.py` | Dik konumda (θ = 0) analitik A, B ve sayısal jacobian | `jacobian` |
| `model/motor.py` | DC motor transfer fonksiyonu, en yüksek araba hızı/kuvveti, step motor ivme sınırı | `tf` |
| `control/lqr.py` | Sürekli ve ayrık LQR (Riccati çözümü), ZOH ayrıklaştırma | `lqr`, `dlqr`, `c2d` |
| `control/swingup.py` | Enerji bazlı swing-up (Åström–Furuta) ve araba merkezleme | — |
| `control/supervisor.py` | Mod makinesi: SWING_UP ↔ LQR geçişi (15°/30° histerezis), ray sınırı güvenliği | — |
| `control/pid.py` | Anti-windup'lı PID (karşılaştırma için) | `pid` |
| `analysis/linear.py` | Durum uzayı nesnesi, kutuplar, kontrol edilebilirlik, transfer fonksiyonu, basamak yanıtı, kök eğrisi | `ss`, `eig`, `ctrb`, `step`, `rlocus` |
| `sim/simulator.py` | Lineer olmayan simülasyon: ESP32 gibi 500 Hz ayrık kontrol, AS5600 12-bit kuantalama, gürültü, gecikme, dürtme, ray uçları | `ode45` |
| `filters.py` | Türevden hız kestirimi (25 Hz), Butterworth, Kalman | `butter`, `kalman` |
| `comm/` | JSON protokolü, seri port, ayrı thread'de okuyucu, sanal ESP32 | `serialport` |
| `recorder.py`, `export.py` | CSV kayıt; ESP32 için `gains.h` üretimi | — |

### 2.2 Arayüz (`python main.py`)
* **Canlı İzleme:** θ, x, u kayan grafikleri, araba-sarkaç animasyonu, mod göstergesi, KALİBRE ET / BAŞLAT / DURDUR / ACİL STOP (**Esc**), hedef konum, swing-up aç/kapa, CSV kayıt, durum çubuğunda veri hızı, kayıp satır ve FPS.
* **Model ve LQR Tasarımı:** parametre ve motor değerleri, Q/R ağırlıkları → K. Açık ve kapalı çevrim kutupları, kutup haritası, en yüksek araba hızı uyarısı, "küre mi küp mü" geometri hesaplayıcı, `gains.h` dışa aktarma.
* **Analiz ve Simülasyon:** aşağıdan swing-up dahil lineer olmayan simülasyon ve animasyonlu tekrar oynatma, model uyumsuzluğu ve dürtme denemeleri, basamak yanıtı, kök eğrisi.
* Bağlanınca cihazın kontrol periyodunu (Ts) sorar. Tasarımdakiyle eşleşirse K ve tüm ayarları otomatik yükler, eşleşmezse K göndermeyi reddeder. Cihaz resetlenirse ayarları yeniden yükler.

### 2.3 Sanal ESP32
Donanım olmadan, gerçek kartla **aynı protokol ve aynı komut kurallarıyla** çalışır: homing, limit switch'ler, swing-up, LQR, sensör kusurları. Gerçek karta geçişte arayüzde yalnızca port seçimi değişir.

### 2.4 Referans ESP32 firmware'i (`firmware/esp32_pendulum/`)
Mekatronik ekibe başlangıç noktası. İki FreeRTOS görevi var: 500 Hz kontrol çekirdek 1'de, haberleşme çekirdek 0'da çalışır. Python'daki kontrol mantığının birebir karşılığıdır. Güvenlik davranışları: yanlış yöne homing, süre aşımı, sensör kopması, kalibrasyon hatası ve baskın acil stop. **Henüz derlenmedi ve kartta denenmedi.**

### 2.5 Belgeler (`docs/`)
| Belge | Okuyucu | İçerik |
|---|---|---|
| `protokol.md` | **İki ekip** | Haberleşme sözleşmesi: mesajlar, komutlar, mod makinesi, işaret kontrolü |
| `parametre_tablosu.md` | **Mekatronik ekibi** | Doldurulacak ölçü tablosu ve nasıl ölçüleceği |
| `mimari.md` | Bilgisayar ekibi | Neden döngü ESP32'de, thread yapısı, katmanlar |
| `matlab_karsiliklari.md` | Herkes | Matlab fonksiyonu → kütüphanedeki karşılığı |

---

## 3. Mekatronik ekibinde neler var / neler bekleniyor

Toplantı notları ve tahtadaki plana göre:

| Alan | Durum | Bilgisayar tarafına etkisi |
|---|---|---|
| **Mekanik tasarım:** tabla mekanizması, kayış-kasnak, sigma profil çerçeve, rulman ve köşebentler | Inventor modeli hazırlanıyor | Bittiğinde m, M, l ve atalet `parametre_tablosu.md`'ye girilecek. Inventor "Physical" sekmesi I değerini doğrudan verir |
| **Motor seçimi:** Pololu 35 / Pololu 37D / NEMA17 | Sistem modelleme ekibiyle birlikte karar verilecek | Tasarım sekmesi motorla ulaşılabilecek en yüksek araba hızını gösteriyor. **37D 30:1 + 20 dişli kasnak ~0.2 m/s ile swing-up için yetersiz**; 10:1 + 40 diş (~1.4 m/s) yer tutucu olarak kullanıldı. NEMA17 seçilirse giriş türü "acceleration" olur ve firmware'in motor kısmı yeniden yazılmalıdır |
| **Sarkaç kütlesi: küp mü küre mi** | Karar verilmedi | Geometri hesaplayıcı ikisini karşılaştırır. Uzun çubukta fark küçüktür; belirleyici olan ağırlık merkezi uzaklığı l'dir. Montaj kolaylığına göre seçilebilir |
| **Sistem modelleme ve lineerleştirme** (θ = 0'da, parametrik) | Ekip toplantıda yoktu | Kütüphanede hazır ve testli (analitik + sayısal jacobian). Mekatronik ekibin Matlab sonuçlarıyla karşılaştırılması faydalı olur |
| **Kontrol:** LQR kararı, enerji bazlı swing-up araştırması | LQR seçildi, swing-up araştırılacak | İkisi de uygulandı ve simülasyonda çalışıyor: aşağıdan ~4 s'de kalkış, ±0.2 m ray kullanımı, %20 model hatasında da yakalama. Kazançlar araştırmaya başlangıç noktası olarak kullanılabilir |
| **Sensör / mikrodenetleyici** | ESP32 ve manyetik enkoder seçildi; kesin sensör bu hafta | Her şey AS5600 (12-bit, I2C) varsayımıyla yazıldı. Başka bir sensör seçilirse firmware'deki okuma fonksiyonu ve simülatördeki çözünürlük değişir |
| **Homing'i kim başlatacak** | Belgede açık soru olarak duruyordu | Öneri: kart açılınca bekler, kalibrasyon yalnızca arayüzdeki KALİBRE ET ile başlar (`protokol.md` §6). **Onay bekliyor** |

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

---

## 5. Artılar

* **Doğru mimari:** zaman kritik döngü mikrodenetleyicide; PC tarafının hızı dengeyi etkilemez.
* **Donanımsız geliştirme:** sanal ESP32 sayesinde arayüz, protokol ve kontrol mantığı kart gelmeden uçtan uca denendi.
* **Tek kaynaktan tutarlılık:** aynı kontrol mantığı Python'da, sanal cihazda ve firmware'de var. Parametre aralıkları ve komut listesi testle eşleştiriliyor.
* **Gerçekçi simülasyon:** 12-bit sensör kuantalaması, türevle hız kestirimi, gecikme, gürültü ve model uyumsuzluğu dahil. Swing-up kazançları ±0.4 m'lik kısa ray için taranarak seçildi.
* **Doğrulanmış matematik:** LQR sonuçları python-control ile aynı; analitik model sayısal jacobian ile aynı; bağımsız incelemede türetmeyle de karşılaştırıldı.
* **Güvenlik öncelikli:** homing yapılmadan başlatılamaz; ray sınırı, limit switch, sensör hatası ve acil stop'ta motor kapanır. Esc tuşu her pencerede acil stop'tur.
* **Kolay güncelleme:** gerçek ölçüler gelince yalnızca `config/params.json` değişir; K, simülasyon ve `gains.h` arayüzden tek tuşla yenilenir.
* **Belgelenmiş sözleşme:** iki ekip neyi nasıl göndereceğini `protokol.md`'den okur.

## 6. Eksiler, sınırlamalar ve riskler

* **Firmware gerçek kartta hiç derlenmedi veya denenmedi.** Derleme hataları ve donanıma özgü sorunlar (pin seçimi, sürücü davranışı, enkoder gürültüsü) olabilir.
* **Parametreler yer tutucu.** Swing-up kazançları ve LQR ağırlıkları gerçek değerlerle yeniden ayarlanmalı.
* **Motor kararı kritik:** düşük devirli bir redüktörle swing-up fiziksel olarak mümkün olmayabilir.
* **12-bit açı sensörünün türevi gürültülü:** dengede motor geriliminde ±1–1.5 V titreşim görülüyor. Filtre daha da kısılırsa yakalama başarısız oluyor. Gerekirse Kalman filtresi (kütüphanede hazır) firmware'e taşınabilir.
* **Step motor desteği yarım:** modelde ve simülasyonda var, firmware'de yok.
* **Model sadeleştirmeleri:** kuru (Coulomb) sürtünme, kayış esnekliği, motor endüktansı ve rotor ataleti (değer girilene kadar) ihmal ediliyor. Gerçek sistemde kazançlar biraz farklı çıkabilir.
* **Donanımsal acil stop yazılımla sağlanamaz:** motor beslemesini fiziksel olarak kesen bir buton şart.
* Arayüz Windows'ta denendi; Linux/macOS'ta denenmedi (PyQt5 çapraz platform olduğu için sorun beklenmiyor).

## 7. Yapılması gerekenler

### Mekatronik ekibi
1. Motor + redüktör + kasnak kararı. Tasarım sekmesindeki "en yüksek araba hızı" ≥ ~1 m/s olmalı.
2. Mekanik tasarım bitince `docs/parametre_tablosu.md`'yi doldurmak (m, M, l, I, sürtünmeler, motor sabitleri, rotor ataleti).
3. Homing önerisini (`protokol.md` §6) onaylamak veya değiştirmek.
4. Firmware'i derleyip pinleri donanıma uyarlamak; ilk açılış kontrol listesini uygulamak (`firmware/esp32_pendulum/README.md`): işaret kontrolü → homing → yalnız LQR → swing-up.
5. NC limit switch'ler ve motor beslemesini kesen donanımsal acil stop butonu.

### Bilgisayar ekibi
1. Gerçek parametreler gelince: K'yı yeniden hesaplamak, aşağıdan swing-up simülasyonunu tekrarlamak, gerekirse swing-up kazançlarını ayarlamak, `gains.h` üretmek.
2. İlk donanım denemesinde mekatronik ekiple birlikte olmak; CSV kayıtlarından simülasyonla gerçek davranışı karşılaştırıp modeli düzeltmek (sürtünme vb.).
3. Titreşim sorun olursa firmware'e Kalman filtresi eklemek.
4. Step motor seçilirse firmware'in motor sürme kısmını yazmak.
5. İsteğe bağlı: Wi-Fi/WebSocket ile kablosuz telemetri (ESP32'de mümkün), otomatik parametre tanılama (kayıttan m, b, c tahmini).
