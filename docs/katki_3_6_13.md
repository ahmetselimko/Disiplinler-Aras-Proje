# Enerji grafikleri, deneme özeti ve hassasiyet analizi

Bu değişiklikler #3, #6 ve #13 görevlerini kapsar. Kontrol algoritması, firmware
ve seri protokolün komut/telemetri biçimi değiştirilmez.

## #3 — Enerji ve mod grafikleri

Canlı İzleme ve Analiz sekmelerinde sarkaç enerjisi çizilir:

`E = 0.5 * (I + m*l²) * dth² + m*g*l*(cos(th) - 1)`

Bu, araba dahil tüm sistemin enerjisi değildir. Dik ve durağan konumda 0 J,
aşağıda durağan konumda −2mgl J olur. Canlı grafik PC model parametrelerini,
uyumsuzluk simülasyonu gerçek tesis parametrelerini kullanır. Gerçek fiziksel
parametreler ölçülene kadar canlı enerji, modelden hesaplanan bir tahmindir.

Mod grafikleri IDLE, HOMING, WAIT, SWING_UP, BALANCE ve FAULT'u ayrı değerlerle,
basamak biçiminde gösterir. Enerji grafiğindeki kesikli çizgi 0 J hedefidir.

## #6 — Deneme özeti ve kayıt

- İlk dengeleme: sarılmış açı |θ| ≤ 2° bandında kesintisiz en az 1 s gözlenirse,
  o bandın başlangıcı gösterilir. Bu değer, sonraki zamanlarda düşülmediği anlamına gelmez.
- LQR ilk devreye girişi ayrıca gösterilir; dengeleme zamanı ile karıştırılmaz.
- Maksimum araba hareketi `max(abs(x))` olup rayın merkezine göredir; toplam kat edilen mesafe değildir.
- Doyum süresi: sol örneğin girdisi bir sonraki örneğe kadar sabit sayılır.
  `abs(u) >= u_max - min(0.001, 0.001*u_max)` koşulunun süresi toplanır.
  Yüzde, veri bulunan ve duraklatılmamış süreye bölünür.
- Deneme ilk WAIT/SWING_UP/BALANCE telemetrisinde başlar. WAIT↔BALANCE geçişleri
  aynı denemedir. IDLE/FAULT/HOMING'e geçişte son örnek dahil edilerek bitirilir.
- Özet, okuyucu thread'inde her telemetriyi işler; grafik dondurma ve sekme değişimi
  toplamayı durdurmaz. Örnek geçmişi saklanmaz: bellek ve örnek başına iş sabittir.
- Eksik sıra numarası, 1.5 s'den büyük boşluk ve onaylanmış simülasyon duraklatması
  denge sürekliliğini keser; bu aralıklarda doyum tahmini yapılmaz. Hariç süre gösterilir.
- Bağlantı kesilince son özet korunur. Yeni bağlantı/deneme ayrı hesaplanır.

CSV'nin `#` ile başlayan başlığında K, Q/R, tasarım Ts, cihazın bildirdiği Ts,
fiziksel parametreler ve motor modeli, swing-up ayarları, geçiş açıları, x_ref,
birimler ve özet tanımı saklanır. İç içe sözlükler ve diziler JSON biçimindedir.
Başlık PC tasarımının kayıt başındaki görüntüsüdür; cihazın bunu uyguladığına
dair tek başına kanıt sayılmaz. Kayıt sırasında gönderilen komutlar ve cihazın
ACK/olayları `# event:` JSON satırlarında tutulur; ret yanıtları da korunur.
Başlıkta cihazın ACK ile kabul ettiği bilinen ayarlar ayrıca saklanır. Kayıt
sırasında PC tasarımı yeniden hesaplanırsa yeni Q/R, Ts ve parametreler de bir
`pc_design_snapshot` olayı olarak korunur.
Firmware'in PC'den değiştirilemeyen a_kick/E_target gibi varsayılanları PC
tasarım değerlerinden farklı olabilir; fiziksel parametreler firmware'e otomatik yüklenmez.

Örnek veri okuma:

```python
import pandas as pd
data = pd.read_csv("kayitlar/oturum_....csv", comment="#")
```

## #13 — Parametre hassasiyeti

```powershell
python tools/hassasiyet_analizi.py
# Yerel sanal ortam kullanılıyorsa:
.\.venv\Scripts\python.exe tools/hassasiyet_analizi.py
```

Komut `docs/hassasiyet_raporu.md` ve `docs/hassasiyet_raporu.json` dosyalarını
yeniden üretir. Farklı veri/süre/çıktı için `--params`, `--duration`, `--dt`,
`--output` seçenekleri vardır. Nominal K bir kez tasarlanır. Her seferinde yalnız
gerçek sistemin bir parametresi değiştirilir; kontrolcünün modeli ve K sabit kalır.

14 parametre: m, M, l, I, b, c; motor R, Kt, Ke, gear, r_pulley, J_rotor, T_f, B_m.
Pozitif nominal değerler −%30, −%20, −%10, +%10, +%20, +%30 değiştirilir.
Her geçerli sapmada iki bağımsız deney yapılır: 180°'den swing-up ve 5°'den denge.
Nominal model de aynı iki deneyden geçirilir. Nominal deney başarısızsa parametreler
başarısızlığın nedeni olarak sıralanmaz.

Başarı: son en az 1 s boyunca BALANCE ve |θ| ≤ 2°; deney boyunca FAULT ve ray
teması yok. Yalnız son örneğin dik olması yeterli değildir. Rayın %75'inden fazlası
kullanılan başarılı deneyler SINIRDA gösterilir. ±%10/20'de başarısızlık KRİTİK,
yalnız %30'da başarısızlık veya sınıra yakınlık ORTA; diğer geçerli sonuçlar yalnız
taranan koşullar için TOLERANSLI olarak sınıflanır.

Sıfır nominal I/J_rotor/T_f/B_m değerlerini yüzdeyle çarpmak deney üretmez.
Bu nedenle yalnız pozitif yönde, raporda açıkça belirtilen mutlak referansın
%10/20/30'u eklenir; negatif değerler uygulanamaz olarak işaretlenir. Bu referanslar
ölçüm değildir ve nominal kontrol modelini değiştirmez. Gerçek değerler ölçülünce
normal ±% taraması yapılmalıdır. Sıfır b/c için de raporda belirtilen keşif ölçekleri vardır.

Rapor; her deneyin gerçek parametre değerini, ilk dengeleme zamanını, son kesintisiz
denge süresini, max |x| ve doyum süresini içerir. Sayısal ıraksama açık HATA olarak
raporlanır; programlama hataları başarısız fizik deneyi gibi gizlenmez.

Sonuçlar tek başlangıç ailesi, sıfır gürültülü kuantalanmış sensör ve sonlu deney
süresi içindir. Fiziksel parametre uyumsuzluğu ölçülür; sensör ölçek hatası,
birleşik sapmalar ve gecikme bu taramanın kapsamı dışındadır. Gerçek donanımda
başarı veya daha geniş tolerans garantisi verilmez.

## Doğrulama

```powershell
python -m pytest -q
```

Regresyon testleri; Analiz'de gerçek simülasyon çalıştırmayı ve enerji değerlerini,
çizim yapılmadan kısa tepe/doyum yakalamayı, WAIT/FAULT/boot/bağlantı kesme,
duraklatma ve eksik paket davranışını, CSV ayar/ACK kaydını, geç yakalamayı,
denge sonrası düşmeyi ve sonuçlardan türetilen raporu kapsar.
