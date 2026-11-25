# Mekatronik Tasarım Ölçüleriyle Ön Değerlendirme

Bu belge, mekatronik ekibinin ara raporundaki (ENG401 Midterm Progress Report, **23.11.2026 sürümü**, 52 sayfa)
**tasarım** değerleriyle yazılımın nasıl davrandığını gösterir. Amaç, gerçek ölçüler gelmeden önce riskleri erkenden
görmek ve iki ekibin netleştirmesi gereken noktaları listelemektir.

> ⚠ **Bu bir ön değerlendirmedir.** Kullanılan değerler CAD tasarımından ve mekatroniğin hesaplarından gelir, ölçüm
> değildir. JGB37-520'nin datasheet'i henüz yok. Motor, raporun Tablo 16'daki gerilim–kuvvet kazancı ve zıt EMK
> sönümüyle modellendi; R, Kt ve Ke bu iki değerden Kt = Ke varsayımıyla türetildi, ölçülmedi. Projenin asıl parametre dosyası `config/params.json` **değiştirilmedi**;
> bu çalışma ayrı bir dosyayla yapıldı: `config/params_mekatronik_tasarim.json`.

Yeniden üretmek için:

```bash
python tools/mekatronik_on_degerlendirme.py          # uç kütlesi × hedef enerji tabloları (~1.5 dk)
python tools/rapor_sekilleri.py --params config/params_mekatronik_tasarim.json \
       --out docs/media/rapor_mekatronik --e-ratio 1.05   # şekiller
```

> **#50 Aşama 2 (25.11.2026):** bu belgedeki tasarım değerleri artık projenin asıl parametre dosyasında:
> `config/params.json` (+20 g), `config/params_ucsuz.json`, `config/params_50g.json`; enkoder 44 CPR (836 sayım /
> çıkış turu), swing-up hedef enerji oranı varsayılanı 1.05. Aşağıdaki "`params.json` (yer tutucu)" sütunu Aşama 2
> öncesi dosyayı gösterir. §2 tabloları 44 CPR enkoderle yeniden üretildi (`python tools/mekatronik_on_degerlendirme.py`).

## 1. Ölçüler: yer tutucu ve mekatronik tasarımı

| Büyüklük | `params.json` (Aşama 2 öncesi yer tutucu) | Mekatronik tasarımı | Kaynak |
|---|---|---|---|
| Sarkaç kütlesi m | 0.20 kg | 0.085 / **0.105** / 0.135 kg (uç kütlesiz / **+20 g** / +50 g) | Bölüm 13.2, Tablo 15 |
| Pivot–ağırlık merkezi l | **0.30 m** | 0.064 / **0.099** / 0.133 m | Tablo 15 |
| Pivota göre atalet | noktasal kütle (I = 0; m·l² = 0.018 kg·m²) | 0.000986 / **0.002236** / 0.004111 kg·m² (modelde ağırlık merkezine göre: I = I_pivot − m·l²) | Tablo 15 |
| Araba kütlesi M | 0.5 kg | 1.10 kg (sarkaç dahil) − m + 0.26 kg kayış + 0.025 kg kasnak → **1.28 kg** | Bölüm 12.1, 14.1 |
| Redüktör | 10 (yer tutucu) | **19:1** | Bölüm 1.2, 13.3 |
| Enkoder | 48 CPR motor mili (yer tutucu) | 836 sayım / çıkış mili turu (≈ 44 CPR × 19) | Bölüm 13.3 |
| Motor gerilim → kuvvet | 37D yer tutucusu | **α = 2.26 N/V**, zıt EMK sönümü **48.41 N·s/m** (yüksüz ≈ 0.56 m/s, 12 V'ta durma kuvveti ≈ 27 N) | Tablo 16 |
| Rotorun arabaya yansıyan kütlesi | 0 | **1.346 kg** | Bölüm 14.1 |
| Sürtünme | 0 (modelde) | **3.0 N** (rulman + kayış) | Bölüm 14.1 |
| Ray | ±0.4 m | ±0.40 m (geçici); mil 2.5 m, çerçeve 2.18 m | Bölüm 8.4, 12.1 |

Kalın değerler `params_mekatronik_tasarim.json`'a girilenlerdir (+20 g seçeneği). Motor sabitleri, Tablo 16'daki iki
değeri ideal bir DC motorun SI birimlerindeki Kt = Ke eşitliğiyle verecek şekilde seçildi: R = b/α² ≈ 9.48 Ω,
Kt = Ke = α·R·r/N ≈ 0.0108. Bunlar ayrı ayrı ölçülmüş değerler değildir; simülasyonda yalnız α ve b birleşik değerleri
kullanılır. Datasheet gelince güncellenmeli.

**En önemli fark sarkaç boyudur:** gerçek sarkacın ağırlık merkezi yer tutucudakinin üçte biri kadar. Sarkaç daha
kısa olduğu için daha hızlı devriliyor (açık çevrim kararsız kutup 5.77 → 6.67 1/s).

## 2. Swing-up: uç kütlesi × hedef enerji oranı

Kontrolcü arayüzün varsayılanları (Q = diag(10, 1, 100, 1), R = 0.1, Ts = 2 ms, yakalama 15°, bırakma 30°).
Sarkaç aşağıda başlıyor, 30 s. Enkoder 44 CPR × 19 (Aşama 2). "Dike en fazla" = sarkacın dik konuma en çok yaklaştığı açı.

**Hedef enerji oranının tanımı:** bizim `E_ratio`, hedef enerjiyi aşağı-durağan konumdan ölçüp dik konum enerjisine
(2mgl) oranlar. Mekatroniğin E_ref = 1.02·mgl'si (enerji dikte +mgl, aşağıda −mgl) bizim tanımımızda **1.01**'dir.

**Sürtünme telafisi AÇIK** (kontrolcü 3 N'u biliyor; firmware'de sürtünme ölçümünün sonucu `fc_volt` gönderilmiş gibi):

| Hedef enerji oranı | uç kütlesiz | +20 g | +50 g |
|---|---|---|---|
| 1.00 (#50 öncesi varsayılan) | ✘ dike en fazla 32° | ✘ dike en fazla 22° | ✘ 3.8 s'de yakalandı ama tutulamadı |
| 1.01 (= mekatroniğin E_ref 1.02·mgl'si) | ✘ dike en fazla 29° | ✘ dike en fazla 18° | ✔ 3.8 s (max \|x\| 0.15 m) |
| 1.02 | ✘ dike en fazla 27° | ✔ 4.4 s (max \|x\| 0.23 m) | ✔ 3.7 s (max \|x\| 0.15 m) |
| **1.05 (varsayılan, #50 Aşama 2)** | ✘ dike en fazla 16° | ✔ 4.3 s (max \|x\| 0.14 m) | ✔ 3.7 s (max \|x\| 0.15 m) |
| 1.10 | ✔ 5.6 s (max \|x\| 0.27 m) | ✔ 4.3 s (max \|x\| 0.21 m) | ✔ ilk yakalama 3.7 s; düştü, yeniden yakalandı (max \|x\| 0.29 m) |

**Sürtünme telafisi KAPALI** (`fc_volt` = 0; kontrolcü T_f'yi bilmiyor; fizikte 3 N sürtünme var):

| Hedef enerji oranı | uç kütlesiz | +20 g | +50 g |
|---|---|---|---|
| 1.00 (#50 öncesi varsayılan) | ✘ dike en fazla 180° | ✘ dike en fazla 27° | ✘ dike en fazla 21° |
| 1.01 (= mekatroniğin E_ref 1.02·mgl'si) | ✘ dike en fazla 180° | ✘ dike en fazla 25° | ✘ dike en fazla 17° |
| 1.02 | ✘ dike en fazla 180° | ✘ dike en fazla 22° | ✔ 5.4 s (max \|x\| 0.14 m) |
| **1.05 (varsayılan, #50 Aşama 2)** | ✘ dike en fazla 90° | ✔ 7.2 s (max \|x\| 0.14 m) | ✔ 5.4 s (max \|x\| 0.14 m) |
| 1.10 | ✘ dike en fazla 89° | ✔ ilk yakalama 6.4 s; düştü, yeniden yakalandı (max \|x\| 0.35 m) | ✔ ilk yakalama 5.3 s; düştü, yeniden yakalandı (max \|x\| 0.36 m) |

**Sonuçlar:**
1. **#50 öncesi varsayılanlarla (oran 1.00, telafi kapalı) swing-up hiçbir seçenekte tamamlanmıyor.** Uç kütlesiz sarkaç
   hiç kalkmıyor (telafisiz 3 N sürtünme ilk dürtmeyi yutuyor); +20 g ve +50 g'de dike 21–27°'ye kadar gelip 15°'lik
   yakalama bölgesine giremiyor. Bu yüzden Aşama 2'de varsayılan oran 1.05 yapıldı.
2. **Uç kütlesi takılmalı.** Uç kütlesiz sarkaç yalnız telafi açıkken ve oran 1.10'da kalkıyor; telafi kapalıyken hiç
   kalkmıyor. Mekatronik raporu da uç kütlesini "titreşimi azaltmak" için öneriyor (Bölüm 13.2); bu sonuç ayrıca
   swing-up için de gerekli olduğunu gösteriyor.
3. **Hedef enerji oranı 1.05 civarına çekilmeli** (Aşama 2'de yapıldı). 1.05 ve 1.10, +20 g ve +50 g'de telafi açık
   da kapalı da kalkıyor (1.10'da bazı hücrelerde sarkaç bir kez düşüp yeniden yakalanıyor ve araba 0.35 m'ye kadar
   gidiyor); 1.05 daha az ray kullanıyor. Mekatroniğin değeri (bizde 1.01) yalnız +50 g ve telafi açıkken yetiyor.
   Ayrıntılı sağlamlık taraması: `docs/swingup_karar_calismasi.md` (Aşama 3).
4. **Sürtünme telafisi önemli.** 3 N sürtünme, enerji pompalamanın küçük itmelerini yutuyor. İlk donanım denemesinden
   önce sürtünme ölçümü (arayüzdeki "Sürtünme ölç") yapılıp telafi açılmalı. (Aşama 2'den beri `params.json`'da T_f tasarım
   değerinde, yani arayüz bağlanınca fc_volt ≈ 1.33 V gönderir; sürtünme ölçülünce güncellenmeli.)
5. **Sınırdaki durumlar hassas.** Örneğin +50 g, oran 1.00, telafi açık: sarkaç yakalanıyor ama tutulamıyor; motor
   sabitlerinin son basamaktaki yuvarlaması bile bu hücreyi "dengede kaldı" ile "tutulamadı" arasında değiştirdi.
   Karar verirken tek hücreye değil, satırın ve sütunun genel eğilimine bakılmalı.

## 3. Önerilen ayarla sonuçlar (+20 g, oran 1.05, telafi açık)

Ayrıntılı şekiller ve tablolar: [`media/rapor_mekatronik/README.md`](media/rapor_mekatronik/README.md) (Aşama 2'den
beri `media/rapor/README.md` de aynı değerlerle üretiliyor). Sol sütun Aşama 2 öncesi yer tutucu parametrelerin
sonuçlarıdır; kaynağı git geçmişindeki `docs/media/rapor/README.md` (Aşama 1 sonu):

| | Aşama 2 öncesi yer tutucu (oran 1.00) | Mekatronik tasarımı, +20 g (oran 1.05) |
|---|---|---|
| LQR K (θ sola +, #50) | [−9.52, −27.29, 61.49, 9.74] | [−9.83, −46.64, 101.46, 14.36] |
| Aşağıdan kalkış: LQR'a geçiş | 4.34 s | 4.32 s |
| Max \|x\| / max \|u\| | 0.157 m / 6.7 V | 0.142 m / **12.0 V** (gerilim sınırında) |
| LQR'ın kurtardığı en büyük itme (50 ms, sarkaca) | ≈ 3.3 N | **≈ 0.8 N** |
| Model uyumsuzluğunda dengede bittiği aralık (m ve l birlikte) | 0 … +30% (+20% hariç) | **−20% … +20%** (−20% ve −10%'da bir kez düştü, yeniden kalktı) |

* **İtmeye dayanıklılık dörtte birine düşüyor.** Kısa ve hafif sarkaç çok daha küçük bir itmeyle devriliyor; 0.8 N'dan
  büyük itmelerde LQR sarkacı tutamıyor.
* **Motor gerilim sınırında çalışıyor.** Swing-up sırasında 12 V'a dayanıyor: motorun gerçek kuvveti (≈ 27 N durma)
  yer tutucudakinden çok düşük ve rotorun 1.35 kg'lık yansıyan kütlesi arabayı ağırlaştırıyor. Datasheet gelince
  yeniden bakılmalı.
* **Model uyumsuzluğu:** −%20 … +%20 aralığında deneme dengede bitiyor; −%20 ve −%10'da LQR sarkacı bir kez bırakıyor
  ve swing-up yeniden kaldırıyor (araba 0.29–0.33 m'ye kadar gidiyor), +%20'de araba ray ucuna yaklaşıyor (0.35 m).
  −%25 ve altında sarkaç yakalama bölgesine hiç yaklaşamıyor;
  +%25'te LQR yakalıyor ama tutamıyor, +%30'da yakalama bölgesinin hemen dışında kalıyor. **m ve l'nin ölçülerek doğru
  girilmesi şart.**
* **Dengede sürtünmeden doğan konum kayması:** LQR Coulomb sürtünmesini telafi etmez (firmware'de de telafi yalnız
  swing-up'ta). 3 N sürtünme, konum hatası küçükken LQR'ın komutunu yutar ve araba yaklaşık F_c / (α·|K_x|) ≈ 0.14 m'lik
  bir ölü bantta durabilir: dengede araba ≈ 0.14 m'ye kadar kayabiliyor (dürtme tablosunda 0.2 N'lik itmede bile
  0.141 m); sürtünmesiz dengede arabanın durduğu yer birkaç mm. ±0.4 m rayın üçte biri; gerçek sürtünme ölçülünce dengede de telafi ya da
  integral etki değerlendirilmeli. (Tablolardaki max \|x\| bütün koşuyu kapsar; swing-up evresi de ≈ 0.14 m'ye çıkar.)

## 4. İki ekibin raporları arasındaki farklar

| Konu | Bilgisayar ekibi (yazılım ve rapor) | Mekatronik raporu | İlgili issue |
|---|---|---|---|
| θ işareti | ~~Sağa (+x) yatınca pozitif~~ → **sola yatınca pozitif** (#50 Aşama 1) | Sola yatınca pozitif (Bölüm 8.1.3); "enkoder, AS5600 ve motor kablolaması bu işarete uymalı" | #1 kapandı (#50) |
| LQR'a geçiş / bırakma | 15° / 30° | 10° / 25°; ayrıca \|θ̇\| ≤ 1.5 rad/s, \|x\| ≤ 0.25 m, \|ẋ\| ≤ 0.70 m/s (Bölüm 8.4) | #11 |
| Swing-up hedef enerjisi | **1.05** (#50 Aşama 2; arayüzden ayarlanabilir) | E_ref = 1.02·mgl (Bölüm 8.2.4) = bizim tanımımızda 1.01 | #11 |
| Kontrol döngüsü | 500 Hz | Bölüm 8'de 2 ms (500 Hz), Bölüm 1.6'da 1 kHz | #2 |
| Telemetri | 200 Hz | 500–1000 Hz (Bölüm 1.6) | #2 |
| USB bağlantısı | Kartın "COM" (UART) portu | Kart seçiminin ana gerekçesi Native USB (Bölüm 1.6) | #2 |
| Enkoder okuma | Kesme (interrupt) | Donanım sayacı PCNT (Bölüm 1.6) | — |
| θ̇ tahmini | Türev filtresi + dik bölgede Kalman; MPU6050 kullanılmıyor | AS5600 türevi + MPU6050 jiroskobu birleştirmesi (Bölüm 8.1.4) | #50 |
| Kontrol girişi | Motor gerilimi (modelde motor var) | Kuvvet; donanımda motor modeliyle gerilime çevrilecek (Bölüm 8.1.5) | — |
| Motor verisi | Rapordaki değerler işlendi (#50 Aşama 2): 19:1, 44 CPR (836 sayım/tur), Tablo 16'daki α ve b | 19:1, 836 CPR, 2.2 / 3.1 kg·cm, α = 2.26 N/V (satın alınca doğrulanacak) | #50 |

Mekatronik raporunun kendi içindeki tutarsızlıklar (onlara iletildi): araba kütlesi Bölüm 1.2'de 500–600 g,
Bölüm 12–15'te 1.10 kg; rotorun yansıyan kütlesi Bölüm 8.1.5'te ≈ 25 kg (eski 43.8:1 motor), Bölüm 14.1'de
1.346 kg; Bölüm 4 ve Tablo 1 hâlâ FIT0186'ya göre; MATLAB parametreleri (Tablo 12) eski geçici değerler.

## 5. Mekatronik ekibinden beklenenler
1. **Uç kütlesi:** takılacak mı, hangisi (+20 g / +50 g)? Bu belgeye göre takılması gerekiyor.
2. **JGB37-520 datasheet'i:** R, Kt, Ke ve enkoder; ya da motor gelince ölçüm.
3. **Geçiş eşikleri** için ortak karar (#11). (θ işareti kararlaştırıldı ve uygulandı: sola pozitif, #50.)
4. **Kontrol döngüsü ve USB portu:** 500 Hz mi 1 kHz mi; UART mı Native USB mi (#2).
5. **Raydaki kullanılabilir yol** (şasi 2.18 m; şu an ±0.40 m varsayılıyor).
6. Montaj bitince **ölçülmüş** m, l, M ve sürtünme. Bunlar gelince `config/params.json` güncellenir,
   `tools/rapor_sekilleri.py` ve hassasiyet analizi yeniden çalıştırılır.
