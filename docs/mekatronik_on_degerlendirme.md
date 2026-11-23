# Mekatronik Tasarım Ölçüleriyle Ön Değerlendirme

Bu belge, mekatronik ekibinin ara raporundaki (ENG401 Midterm Progress Report, **23.11.2026 sürümü**, 52 sayfa)
**tasarım** değerleriyle yazılımın nasıl davrandığını gösterir. Amaç, gerçek ölçüler gelmeden önce riskleri erkenden
görmek ve iki ekibin netleştirmesi gereken noktaları listelemektir.

> ⚠ **Bu bir ön değerlendirmedir.** Kullanılan değerler CAD tasarımından ve mekatroniğin hesaplarından gelir, ölçüm
> değildir. JGB37-520'nin datasheet'i henüz yok. Motor, raporun Tablo 16'daki gerilim–kuvvet kazancı ve zıt EMK
> sönümüyle modellendi; direnç (R) yer tutucu. Projenin asıl parametre dosyası `config/params.json` **değiştirilmedi**;
> bu çalışma ayrı bir dosyayla yapıldı: `config/params_mekatronik_tasarim.json`.

Yeniden üretmek için:

```bash
python tools/mekatronik_on_degerlendirme.py          # uç kütlesi × hedef enerji tabloları (~1.5 dk)
python tools/rapor_sekilleri.py --params config/params_mekatronik_tasarim.json \
       --out docs/media/rapor_mekatronik --e-ratio 1.05   # şekiller
```

## 1. Ölçüler: yer tutucu ve mekatronik tasarımı

| Büyüklük | `params.json` (yer tutucu) | Mekatronik tasarımı | Kaynak |
|---|---|---|---|
| Sarkaç kütlesi m | 0.20 kg | 0.085 / **0.105** / 0.135 kg (uç kütlesiz / **+20 g** / +50 g) | Bölüm 13.2, Tablo 15 |
| Pivot–ağırlık merkezi l | **0.30 m** | 0.064 / **0.099** / 0.133 m | Tablo 15 |
| Pivota göre atalet | (çubuk modeli) | 0.000986 / **0.002236** / 0.004111 kg·m² (modelde ağırlık merkezine göre: I = I_pivot − m·l²) | Tablo 15 |
| Araba kütlesi M | 0.5 kg | 1.10 kg (sarkaç dahil) − m + 0.26 kg kayış + 0.025 kg kasnak → **1.28 kg** | Bölüm 12.1, 14.1 |
| Redüktör | 10 (yer tutucu) | **19:1** | Bölüm 1.2, 13.3 |
| Enkoder | 48 CPR motor mili (yer tutucu) | 836 sayım / çıkış mili turu (≈ 44 CPR × 19) | Bölüm 13.3 |
| Motor gerilim → kuvvet | 37D yer tutucusu | **α = 2.26 N/V**, zıt EMK sönümü **48.41 N·s/m** (yüksüz ≈ 0.56 m/s, 12 V'ta durma kuvveti ≈ 27 N) | Tablo 16 |
| Rotorun arabaya yansıyan kütlesi | 0 | **1.346 kg** | Bölüm 14.1 |
| Sürtünme | 0 (modelde) | **3.0 N** (rulman + kayış) | Bölüm 14.1 |
| Ray | ±0.4 m | ±0.40 m (geçici); mil 2.5 m, çerçeve 2.18 m | Bölüm 8.4, 12.1 |

Kalın değerler `params_mekatronik_tasarim.json`'a girilenlerdir (+20 g seçeneği). Motor sabitleri Kt ve Ke, R = 2.18 Ω
yer tutucusuyla Tablo 16'daki iki değeri verecek şekilde seçildi: Kt = α·R·r/N, Ke = b·R·r²/(Kt·N²). Simülasyonda
yalnız bu iki birleşik değer önemli olduğu için R'nin yer tutucu olması sonucu değiştirmez.

**En önemli fark sarkaç boyudur:** gerçek sarkacın ağırlık merkezi yer tutucudakinin üçte biri kadar. Sarkaç daha
kısa olduğu için daha hızlı devriliyor (açık çevrim kararsız kutup 5.77 → 6.67 1/s).

## 2. Swing-up: uç kütlesi × hedef enerji oranı

Kontrolcü arayüzün varsayılanları (Q = diag(10, 1, 100, 1), R = 0.1, Ts = 2 ms, yakalama 15°, bırakma 30°).
Sarkaç aşağıda başlıyor, 30 s. "Dike en fazla" = sarkacın dik konuma en çok yaklaştığı açı.

**Hedef enerji oranının tanımı:** bizim `E_ratio`, hedef enerjiyi aşağı-durağan konumdan ölçüp dik konum enerjisine
(2mgl) oranlar. Mekatroniğin E_ref = 1.02·mgl'si (enerji dikte +mgl, aşağıda −mgl) bizim tanımımızda **1.01**'dir.

**Sürtünme telafisi AÇIK** (kontrolcü 3 N'u biliyor; firmware'de sürtünme ölçümünün sonucu `fc_volt` gönderilmiş gibi):

| Hedef enerji oranı | uç kütlesiz | +20 g | +50 g |
|---|---|---|---|
| **1.00 (şu anki varsayılan)** | ✘ dike en fazla 32° | ✘ dike en fazla 22° | ✔ 3.8 s (max \|x\| 0.29 m) |
| 1.01 (= mekatroniğin E_ref 1.02·mgl'si) | ✘ dike en fazla 30° | ✘ dike en fazla 18° | ✔ 3.8 s (max \|x\| 0.15 m) |
| 1.02 | ✘ dike en fazla 27° | ✔ 4.4 s (max \|x\| 0.31 m) | ✔ 3.7 s (max \|x\| 0.15 m) |
| **1.05** | ✘ dike en fazla 16° | ✔ 4.3 s (max \|x\| 0.14 m) | ✔ 3.7 s (max \|x\| 0.15 m) |
| 1.10 | ✔ 5.5 s (max \|x\| 0.14 m) | ✔ 4.3 s (max \|x\| 0.20 m) | ✔ 3.7 s (max \|x\| 0.30 m) |

**Sürtünme telafisi KAPALI** (`fc_volt` = 0, şu anki varsayılan; fizikte 3 N sürtünme var):

| Hedef enerji oranı | uç kütlesiz | +20 g | +50 g |
|---|---|---|---|
| **1.00 (şu anki varsayılan)** | ✘ dike en fazla 90° | ✘ dike en fazla 28° | ✘ dike en fazla 21° |
| 1.01 (= mekatroniğin E_ref 1.02·mgl'si) | ✘ dike en fazla 90° | ✘ dike en fazla 25° | ✘ dike en fazla 17° |
| 1.02 | ✘ dike en fazla 90° | ✘ dike en fazla 22° | ✔ 5.4 s (max \|x\| 0.14 m) |
| **1.05** | ✘ dike en fazla 90° | ✔ 7.2 s (max \|x\| 0.14 m) | ✔ 5.4 s (max \|x\| 0.14 m) |
| 1.10 | ✘ dike en fazla 89° | ✘ ray ucuna çarptı | ✔ 5.3 s (max \|x\| 0.36 m) |

**Sonuçlar:**
1. **Mevcut varsayılanlarla (oran 1.00, telafi kapalı) swing-up hiçbir seçenekte tamamlanmıyor.** Sarkaç dik konuma
   yaklaşıyor ama 15°'lik yakalama bölgesine giremiyor.
2. **Uç kütlesi takılmalı.** Uç kütlesiz sarkaç yalnız telafi açıkken ve oran 1.10'da kalkıyor; telafi kapalıyken hiç
   kalkmıyor. Mekatronik raporu da uç kütlesini "titreşimi azaltmak" için öneriyor (Bölüm 13.2); bu sonuç ayrıca
   swing-up için de gerekli olduğunu gösteriyor.
3. **Hedef enerji oranı 1.05 civarına çekilmeli.** 1.05, +20 g ve +50 g'de telafi açık da kapalı da kalkan tek
   değer. Mekatroniğin değeri (bizde 1.01) yalnız +50 g ve telafi açıkken yetiyor.
4. **Sürtünme telafisi önemli.** 3 N sürtünme, enerji pompalamanın küçük itmelerini yutuyor. İlk donanım denemesinden
   önce sürtünme ölçümü (arayüzdeki "Sürtünme ölç") yapılıp telafi açılmalı.

## 3. Önerilen ayarla sonuçlar (+20 g, oran 1.05, telafi açık)

Ayrıntılı şekiller ve tablolar: [`media/rapor_mekatronik/README.md`](media/rapor_mekatronik/README.md). Yer tutucu
parametrelerle karşılaştırma ([`media/rapor/README.md`](media/rapor/README.md)):

| | Yer tutucu (oran 1.00) | Mekatronik tasarımı, +20 g (oran 1.05) |
|---|---|---|
| LQR K | [−9.52, −27.29, −61.49, −9.74] | [−9.83, −46.64, −101.46, −14.36] |
| Aşağıdan kalkış: LQR'a geçiş | 4.34 s | 4.32 s |
| Max \|x\| / max \|u\| | 0.157 m / 6.7 V | 0.139 m / **12.0 V** (gerilim sınırında) |
| LQR'ın kurtardığı en büyük itme (50 ms, sarkaca) | ≈ 3.3 N | **≈ 0.8 N** |
| Model uyumsuzluğunda swing-up'ın tamamlandığı aralık (m ve l birlikte) | 0 … +30% (+20% hariç) | **−20% … +20%** |

* **İtmeye dayanıklılık dörtte birine düşüyor.** Kısa ve hafif sarkaç çok daha küçük bir itmeyle devriliyor; 0.8 N'dan
  büyük itmelerde LQR sarkacı tutamıyor.
* **Motor gerilim sınırında çalışıyor.** Swing-up sırasında 12 V'a dayanıyor: motorun gerçek kuvveti (≈ 27 N durma)
  yer tutucudakinden çok düşük ve rotorun 1.35 kg'lık yansıyan kütlesi arabayı ağırlaştırıyor. Datasheet gelince
  yeniden bakılmalı.
* **Model uyumsuzluğu:** −%20 … +%20 aralığında swing-up tamamlanıyor (−%10 ve +%20'de araba ray ucuna yaklaşıyor).
  −%25 ve altında sarkaç yakalama bölgesine hiç yaklaşamıyor; +%25'te LQR yakalıyor ama tutamıyor, +%30'da yakalama
  bölgesinin hemen dışında kalıyor. **m ve l'nin ölçülerek doğru girilmesi şart.**

## 4. İki ekibin raporları arasındaki farklar

| Konu | Bilgisayar ekibi (yazılım ve rapor) | Mekatronik raporu | İlgili issue |
|---|---|---|---|
| θ işareti | Sağa (+x) yatınca pozitif | Sola yatınca pozitif (Bölüm 8.1.3); "enkoder, AS5600 ve motor kablolaması bu işarete uymalı" | #1 (karar: sola pozitif, #50) |
| LQR'a geçiş / bırakma | 15° / 30° | 10° / 25°; ayrıca \|θ̇\| ≤ 1.5 rad/s, \|x\| ≤ 0.25 m, \|ẋ\| ≤ 0.70 m/s (Bölüm 8.4) | #11 |
| Swing-up hedef enerjisi | 1.00 (arayüzden ayarlanabilir) | E_ref = 1.02·mgl (Bölüm 8.2.4) = bizim tanımımızda 1.01 | #11 |
| Kontrol döngüsü | 500 Hz | Bölüm 8'de 2 ms (500 Hz), Bölüm 1.6'da 1 kHz | #2 |
| Telemetri | 200 Hz | 500–1000 Hz (Bölüm 1.6) | #2 |
| USB bağlantısı | Kartın "COM" (UART) portu | Kart seçiminin ana gerekçesi Native USB (Bölüm 1.6) | #2 |
| Enkoder okuma | Kesme (interrupt) | Donanım sayacı PCNT (Bölüm 1.6) | — |
| θ̇ tahmini | Türev filtresi + dik bölgede Kalman; MPU6050 kullanılmıyor | AS5600 türevi + MPU6050 jiroskobu birleştirmesi (Bölüm 8.1.4) | #50 |
| Kontrol girişi | Motor gerilimi (modelde motor var) | Kuvvet; donanımda motor modeliyle gerilime çevrilecek (Bölüm 8.1.5) | — |
| Motor verisi | "Datasheet yok, 48 CPR yer tutucu" | 19:1, 836 CPR, 2.2 / 3.1 kg·cm, α = 2.26 N/V (satın alınca doğrulanacak) | #50 |

Mekatronik raporunun kendi içindeki tutarsızlıklar (onlara iletildi): araba kütlesi Bölüm 1.2'de 500–600 g,
Bölüm 12–15'te 1.10 kg; rotorun yansıyan kütlesi Bölüm 8.1.5'te ≈ 25 kg (eski 43.8:1 motor), Bölüm 14.1'de
1.346 kg; Bölüm 4 ve Tablo 1 hâlâ FIT0186'ya göre; MATLAB parametreleri (Tablo 12) eski geçici değerler.

## 5. Mekatronik ekibinden beklenenler
1. **Uç kütlesi:** takılacak mı, hangisi (+20 g / +50 g)? Bu belgeye göre takılması gerekiyor.
2. **JGB37-520 datasheet'i:** R, Kt, Ke ve enkoder; ya da motor gelince ölçüm.
3. **θ işareti ve geçiş eşikleri** için ortak karar (#1, #11).
4. **Kontrol döngüsü ve USB portu:** 500 Hz mi 1 kHz mi; UART mı Native USB mi (#2).
5. **Raydaki kullanılabilir yol** (şasi 2.18 m; şu an ±0.40 m varsayılıyor).
6. Montaj bitince **ölçülmüş** m, l, M ve sürtünme. Bunlar gelince `config/params.json` güncellenir,
   `tools/rapor_sekilleri.py` ve hassasiyet analizi yeniden çalıştırılır.
