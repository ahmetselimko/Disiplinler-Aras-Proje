# Mekatronik Tasarım Ölçüleriyle Ön Değerlendirme

Bu belge, mekatronik ekibinin ara raporundaki (ENG401 Midterm Progress Report, 22.11.2026) **tasarım** değerleriyle
yazılımın nasıl davrandığını gösterir. Amaç, gerçek ölçüler gelmeden önce riskleri erkenden görmek ve iki ekibin
netleştirmesi gereken noktaları listelemektir.

> ⚠ **Bu bir ön değerlendirmedir.** Kullanılan değerler CAD tasarımından gelir, ölçüm değildir. JGB37-520 motorunun
> elektriksel değerleri (R, Kt, Ke) hâlâ yer tutucudur (37D). Projenin asıl parametre dosyası `config/params.json`
> **değiştirilmedi**; bu çalışma ayrı bir dosyayla yapıldı: `config/params_mekatronik_tasarim.json`.

Yeniden üretmek için:

```bash
python tools/mekatronik_on_degerlendirme.py          # uç kütlesi × hedef enerji tablosu
python tools/rapor_sekilleri.py --params config/params_mekatronik_tasarim.json \
       --out docs/media/rapor_mekatronik --e-ratio 1.05   # şekiller
```

## 1. Ölçüler: yer tutucu ve mekatronik tasarımı

| Büyüklük | `params.json` (yer tutucu) | Mekatronik tasarımı | Kaynak |
|---|---|---|---|
| Sarkaç kütlesi m | 0.20 kg | 0.085 / **0.105** / 0.135 kg (uç kütlesiz / **+20 g** / +50 g) | Bölüm 13.2, Tablo 15 |
| Pivot–ağırlık merkezi l | **0.30 m** | 0.064 / **0.099** / 0.133 m | Tablo 15 |
| Pivota göre atalet | (çubuk modeli) | 0.000986 / **0.002236** / 0.004111 kg·m² | Tablo 15 |
| Araba kütlesi M | 0.5 kg | 1.10 kg (sarkaç dahil) + 0.26 kg kayış + 0.025 kg kasnak → **1.28 kg** | Bölüm 12.1, 14.1 |
| Redüktör | 10 (yer tutucu) | **19:1** | Bölüm 1.2, 13.3 |
| Enkoder | 48 CPR motor mili (yer tutucu) | 836 sayım / çıkış mili turu (≈ 44 CPR × 19) | Bölüm 13.3 |
| Motor torku | — | 2.2 kg·cm nominal, 3.1 kg·cm durma | Bölüm 13.3 |
| Sürtünme | ≈ 1.9 N (tahmin) | ≈ 3.0 N (rulman + kayış) | Bölüm 14.1 |
| Ray | ±0.4 m | ±0.40 m (geçici); mil 2.5 m, çerçeve 2.18 m | Bölüm 8.4, 12.1 |

Kalın değerler `params_mekatronik_tasarim.json`'a girilenlerdir (+20 g seçeneği). Sürtünme bu çalışmada modele
eklenmedi; gerçek düzenekte sürtünme kompanzasyonu açık olacağı varsayıldı.

**En önemli fark sarkaç boyudur:** gerçek sarkacın ağırlık merkezi yer tutucudakinin üçte biri kadar. Sarkaç daha
kısa olduğu için daha hızlı devriliyor (açık çevrim kararsız kutup 5.77 → 6.65 1/s).

## 2. Swing-up: uç kütlesi × hedef enerji oranı

Kontrolcü arayüzün varsayılanları (Q = diag(10, 1, 100, 1), R = 0.1, Ts = 2 ms, yakalama 15°, bırakma 30°).
Sarkaç aşağıda başlıyor, 30 s. "Dike en fazla" = sarkacın dik konuma en çok yaklaştığı açı.

| Hedef enerji oranı | uç kütlesiz | +20 g | +50 g |
|---|---|---|---|
| **1.00 (şu anki varsayılan)** | ✘ dike en fazla 52° | ✘ dike en fazla 24° | ✘ dike en fazla 18° |
| 1.02 (mekatroniğin Matlab'da kullandığı) | ✘ dike en fazla 46° | ✘ dike en fazla 16° | ✔ 5.1 s (max \|x\| 0.13 m) |
| **1.05** | ✘ dike en fazla 34° | ✔ 5.5 s (max \|x\| 0.12 m) | ✔ 5.0 s (max \|x\| 0.13 m) |
| 1.10 | ✔ 24.4 s (max \|x\| 0.11 m) | ✔ 5.4 s (max \|x\| 0.12 m) | ✔ 5.0 s (max \|x\| 0.23 m) |

**Sonuçlar:**
1. **Mevcut varsayılanlarla (oran 1.00) swing-up hiçbir seçenekte tamamlanmıyor.** Sarkaç dik konuma yaklaşıyor ama
   15°'lik yakalama bölgesine giremiyor.
2. **Uç kütlesi takılmalı.** Uç kütlesiz sarkaç ancak oran 1.10'da ve 24 s'de dengeye geliyor; +20 g ve +50 g
   seçenekleri 1.05'te yaklaşık 5 s'de dengeye geliyor. Mekatronik raporu da uç kütlesini "titreşimi azaltmak" için
   öneriyor (Bölüm 13.2); bu sonuç ayrıca swing-up için de gerekli olduğunu gösteriyor.
3. **Hedef enerji oranı 1.05 civarına çekilmeli.** Mekatroniğin Matlab'da kullandığı 1.02 yalnız +50 g ile yetiyor.

## 3. Önerilen ayarla sonuçlar (+20 g, oran 1.05)

Ayrıntılı şekiller ve tablolar: [`media/rapor_mekatronik/README.md`](media/rapor_mekatronik/README.md). Yer tutucu
parametrelerle karşılaştırma ([`media/rapor/README.md`](media/rapor/README.md)):

| | Yer tutucu (oran 1.00) | Mekatronik tasarımı, +20 g (oran 1.05) |
|---|---|---|
| LQR K | [−9.52, −27.29, −61.49, −9.74] | [−9.59, −46.64, −80.89, −11.31] |
| Aşağıdan kalkış: LQR'a geçiş | 4.34 s | 5.45 s |
| Max \|x\| / max \|u\| | 0.157 m / 6.7 V | 0.122 m / 9.9 V |
| LQR'ın kurtardığı en büyük itme (50 ms, sarkaca) | ≈ 3.3 N | **≈ 1.1 N** |
| Model uyumsuzluğunda swing-up'ın tamamlandığı aralık (m ve l birlikte) | 0 … +30% (+20% hariç) | **−30% … +10%** |

* **İtmeye dayanıklılık üçte birine düşüyor.** Kısa ve hafif sarkaç daha küçük bir itmeyle devriliyor. 1.1 N'dan
  büyük itmelerde sarkaç düşüyor ve swing-up onu yeniden kaldırıyor (bkz. Şekil 2).
* **Hedef enerji oranı, dayanıklılık aralığını kaydırıyor.** Oran 1.00'de swing-up yalnız gerçek sarkaç modelden
  *ağırsa* tamamlanıyordu; 1.05'te yalnız modelden *hafif veya en fazla %10 ağırsa* tamamlanıyor. Yani tek bir
  oran her durumu karşılamıyor: **m ve l'nin ölçülerek doğru girilmesi şart**, oran ondan sonra ayarlanmalı.
* Max |u| 9.9 V; 12 V sınırına yaklaşıyor. Motorun gerçek değerleri gelince yeniden bakılmalı.

## 4. İki ekibin raporları arasındaki farklar

| Konu | Bilgisayar ekibi (yazılım ve rapor) | Mekatronik raporu | İlgili issue |
|---|---|---|---|
| θ işareti | Sağa (+x) yatınca pozitif | Sola yatınca pozitif (Bölüm 8.1.3); "enkoder, AS5600 ve motor kablolaması bu işarete uymalı" | #1 |
| LQR'a geçiş / bırakma | 15° / 30° | 10° / 25°; ayrıca \|θ̇\| ≤ 1.5 rad/s, \|x\| ≤ 0.25 m, \|ẋ\| ≤ 0.70 m/s (Bölüm 8.4) | #11 |
| Swing-up hedef enerjisi | 1.00 (arayüzden ayarlanabilir) | 1.02 (Bölüm 8.2.4) | #11 |
| Kontrol döngüsü | 500 Hz | Bölüm 8'de 2 ms (500 Hz), Bölüm 1.6'da 1 kHz | #2 |
| Telemetri | 200 Hz | 500–1000 Hz (Bölüm 1.6) | #2 |
| USB bağlantısı | Kartın "COM" (UART) portu | Kart seçiminin ana gerekçesi Native USB (Bölüm 1.6) | #2 |
| Enkoder okuma | Kesme (interrupt) | Donanım sayacı PCNT (Bölüm 1.6) | — |
| θ̇ tahmini | Türev filtresi + dik bölgede Kalman; MPU6050 kullanılmıyor | AS5600 türevi + MPU6050 jiroskobu birleştirmesi (Bölüm 8.1.4) | — |
| Kontrol girişi | Motor gerilimi (modelde motor var) | Kuvvet; donanımda motor modeliyle gerilime çevrilecek (Bölüm 8.1.5) | — |
| Motor verisi | "Datasheet yok, 48 CPR yer tutucu" | 19:1, 836 CPR, 2.2 / 3.1 kg·cm (satın alınca doğrulanacak) | — |

## 5. Mekatronik ekibinden beklenenler
1. **Uç kütlesi:** takılacak mı, hangisi (+20 g / +50 g)? Bu belgeye göre takılması gerekiyor.
2. **JGB37-520 datasheet'i:** R, Kt, Ke ve enkoder; ya da motor gelince ölçüm.
3. **θ işareti ve geçiş eşikleri** için ortak karar (#1, #11).
4. **Kontrol döngüsü ve USB portu:** 500 Hz mi 1 kHz mi; UART mı Native USB mi (#2).
5. Montaj bitince **ölçülmüş** m, l, M ve sürtünme. Bunlar gelince `config/params.json` güncellenir,
   `tools/rapor_sekilleri.py` ve hassasiyet analizi yeniden çalıştırılır.
