# Swing-up karar çalışması (#50 Aşama 3)

`tools/swingup_karar.py` ile 26.11.2026 tarihinde üretildi (900 koşu, 356 s). **Elle düzenlemeyin;** parametreler değişince betiği yeniden çalıştırın. Ham sonuçlar: [`swingup_karar_calismasi.json`](swingup_karar_calismasi.json).

> ⚠ **Tasarım değerleriyle (ölçüm değil) üretildi: Aşama 2 değerleri** (mekatronik ara raporu 23.11.2026; `config/params_ucsuz.json`, `config/params.json` (+20 g), `config/params_50g.json`). Ölçüm gelince sayılar değişir.

## Kurulum

- Simülasyon firmware gibi: Ts = 2 ms ayrık kontrol, 12-bit açı ve 44 CPR × 19 enkoder kuantalaması, dik bölgede Kalman, θ₀ = 180°, T = 30 s. Ölçüme σ = 0.5 sayım beyaz gürültü eklenir (tohum farkı anlamlı olsun).
- K her profil ve ray için yeniden hesaplanır: Q = diag(10.0, 1.0, 100.0, 1.0), R = 0.1, Ts = 2 ms. Swing-up kazançları arayüz varsayılanları; değişen yalnız E_ratio.
- Geçiş kuralları: **bizim**: bizim (15°/30°, 5 rad/s, 0.40 m, 2 m/s); **mekatronik**: mekatronik (10°/25°, 1.5 rad/s, 0.25 m, 0.70 m/s).
- E_ratio bizim tanımımız (aşağıdan ölçülen hedef enerji / 2mgl). **1.01 = mekatroniğin E_ref = 1.02·mgl'si.**
- Sürtünme telafisi **açık**: kontrolcü 3 N'u biliyor (sürtünme ölçümü yapılıp `fc_volt` gönderilmiş gibi); **kapalı**: `fc_volt` = 0, fizikte yine 3 N.
- Hücre: nominal koşu (tohum 0) için LQR'a geçiş zamanı / max |x|; parantezde 5 koşunun (3 tohum, m ve l −%5, +%5) kaçında son mod BALANCE. Nominal koşuya da gürültü eklendiği için sayılar gürültüsüz `mekatronik_on_degerlendirme.md` tablolarıyla birebir karşılaştırılamaz. ✅ beşinde de kalktı; ⚠️ nominalde kalktı ama en az birinde kalkmadı; ❌ nominalde kalkmadı (sebep).

## uç kütlesiz

**Sürtünme telafisi açık**

| E_ratio | Geçiş kuralı | ray ±0.40 m | ray ±0.60 m | ray ±0.80 m |
|---|---|---|---|---|
| 1.00 | bizim | ❌ kalkmadı (dike en fazla 32°) (0/5) | ❌ kalkmadı (dike en fazla 32°) (0/5) | ❌ kalkmadı (dike en fazla 32°) (0/5) |
| 1.00 | mekatronik | ❌ kalkmadı (dike en fazla 32°) (0/5) | ❌ kalkmadı (dike en fazla 32°) (0/5) | ❌ kalkmadı (dike en fazla 32°) (0/5) |
| 1.01 (= mekatronik 1.02·mgl) | bizim | ❌ kalkmadı (dike en fazla 29°) (0/5) | ❌ kalkmadı (dike en fazla 29°) (0/5) | ❌ kalkmadı (dike en fazla 29°) (0/5) |
| 1.01 (= mekatronik 1.02·mgl) | mekatronik | ❌ kalkmadı (dike en fazla 29°) (0/5) | ❌ kalkmadı (dike en fazla 29°) (0/5) | ❌ kalkmadı (dike en fazla 29°) (0/5) |
| 1.02 | bizim | ❌ kalkmadı (dike en fazla 27°) (0/5) | ❌ kalkmadı (dike en fazla 27°) (0/5) | ❌ kalkmadı (dike en fazla 27°) (0/5) |
| 1.02 | mekatronik | ❌ kalkmadı (dike en fazla 27°) (0/5) | ❌ kalkmadı (dike en fazla 27°) (0/5) | ❌ kalkmadı (dike en fazla 27°) (0/5) |
| 1.05 | bizim | ❌ kalkmadı (dike en fazla 16°) (1/5) | ❌ kalkmadı (dike en fazla 16°) (1/5) | ❌ kalkmadı (dike en fazla 16°) (1/5) |
| 1.05 | mekatronik | ❌ kalkmadı (dike en fazla 17°) (1/5) | ❌ kalkmadı (dike en fazla 17°) (1/5) | ❌ kalkmadı (dike en fazla 17°) (1/5) |
| 1.10 | bizim | ✅ 5.5 s / 0.14 m (5/5) | ✅ 5.5 s / 0.14 m (5/5) | ✅ 5.5 s / 0.14 m (5/5) |
| 1.10 | mekatronik | ⚠️ 5.6 s / 0.15 m (2/5) | ⚠️ 5.6 s / 0.15 m (2/5) | ⚠️ 5.6 s / 0.15 m (2/5) |

**Sürtünme telafisi kapalı**

| E_ratio | Geçiş kuralı | ray ±0.40 m | ray ±0.60 m | ray ±0.80 m |
|---|---|---|---|---|
| 1.00 | bizim | ❌ kalkmadı (dike en fazla 90°) (0/5) | ❌ kalkmadı (dike en fazla 90°) (0/5) | ❌ kalkmadı (dike en fazla 90°) (0/5) |
| 1.00 | mekatronik | ❌ kalkmadı (dike en fazla 90°) (0/5) | ❌ kalkmadı (dike en fazla 90°) (0/5) | ❌ kalkmadı (dike en fazla 90°) (0/5) |
| 1.01 (= mekatronik 1.02·mgl) | bizim | ❌ kalkmadı (dike en fazla 90°) (0/5) | ❌ kalkmadı (dike en fazla 90°) (0/5) | ❌ kalkmadı (dike en fazla 90°) (0/5) |
| 1.01 (= mekatronik 1.02·mgl) | mekatronik | ❌ kalkmadı (dike en fazla 90°) (0/5) | ❌ kalkmadı (dike en fazla 90°) (0/5) | ❌ kalkmadı (dike en fazla 90°) (0/5) |
| 1.02 | bizim | ❌ kalkmadı (dike en fazla 90°) (0/5) | ❌ kalkmadı (dike en fazla 90°) (0/5) | ❌ kalkmadı (dike en fazla 90°) (0/5) |
| 1.02 | mekatronik | ❌ kalkmadı (dike en fazla 90°) (0/5) | ❌ kalkmadı (dike en fazla 90°) (0/5) | ❌ kalkmadı (dike en fazla 90°) (0/5) |
| 1.05 | bizim | ❌ kalkmadı (dike en fazla 90°) (0/5) | ❌ kalkmadı (dike en fazla 90°) (0/5) | ❌ kalkmadı (dike en fazla 90°) (0/5) |
| 1.05 | mekatronik | ❌ kalkmadı (dike en fazla 90°) (0/5) | ❌ kalkmadı (dike en fazla 90°) (0/5) | ❌ kalkmadı (dike en fazla 90°) (0/5) |
| 1.10 | bizim | ❌ kalkmadı (dike en fazla 89°) (0/5) | ❌ kalkmadı (dike en fazla 89°) (0/5) | ❌ kalkmadı (dike en fazla 89°) (0/5) |
| 1.10 | mekatronik | ❌ kalkmadı (dike en fazla 89°) (0/5) | ❌ kalkmadı (dike en fazla 89°) (0/5) | ❌ kalkmadı (dike en fazla 89°) (0/5) |

## +20 g

**Sürtünme telafisi açık**

| E_ratio | Geçiş kuralı | ray ±0.40 m | ray ±0.60 m | ray ±0.80 m |
|---|---|---|---|---|
| 1.00 | bizim | ❌ kalkmadı (dike en fazla 22°) (1/5) | ❌ kalkmadı (dike en fazla 22°) (1/5) | ❌ kalkmadı (dike en fazla 22°) (1/5) |
| 1.00 | mekatronik | ❌ kalkmadı (dike en fazla 22°) (0/5) | ❌ kalkmadı (dike en fazla 22°) (0/5) | ❌ kalkmadı (dike en fazla 22°) (0/5) |
| 1.01 (= mekatronik 1.02·mgl) | bizim | ❌ kalkmadı (dike en fazla 18°) (1/5) | ❌ kalkmadı (dike en fazla 18°) (1/5) | ❌ kalkmadı (dike en fazla 18°) (1/5) |
| 1.01 (= mekatronik 1.02·mgl) | mekatronik | ❌ kalkmadı (dike en fazla 19°) (1/5) | ❌ kalkmadı (dike en fazla 19°) (1/5) | ❌ kalkmadı (dike en fazla 19°) (1/5) |
| 1.02 | bizim | ⚠️ 4.4 s / 0.32 m (3/5) | ⚠️ 4.4 s / 0.32 m (3/5) | ⚠️ 4.4 s / 0.32 m (4/5) |
| 1.02 | mekatronik | ❌ kalkmadı (dike en fazla 15°) (1/5) | ❌ kalkmadı (dike en fazla 15°) (1/5) | ❌ kalkmadı (dike en fazla 15°) (1/5) |
| 1.05 | bizim | ✅ 4.3 s / 0.14 m (5/5) | ✅ 4.3 s / 0.14 m (5/5) | ✅ 4.3 s / 0.14 m (5/5) |
| 1.05 | mekatronik | ❌ dike geldi (0°), yakalama şartı sağlanmadı: |θ̇| (1/5) | ❌ dike geldi (0°), yakalama şartı sağlanmadı: |θ̇| (1/5) | ❌ dike geldi (0°), yakalama şartı sağlanmadı: |θ̇| (1/5) |
| 1.10 | bizim | ✅ 4.3 s / 0.34 m (5/5) | ✅ 4.3 s / 0.34 m (5/5) | ✅ 4.3 s / 0.34 m (5/5) |
| 1.10 | mekatronik | ❌ dike geldi (0°), yakalama şartı sağlanmadı: |θ̇| (0/5) | ❌ dike geldi (0°), yakalama şartı sağlanmadı: |θ̇| (0/5) | ❌ dike geldi (0°), yakalama şartı sağlanmadı: |θ̇| (0/5) |

**Sürtünme telafisi kapalı**

| E_ratio | Geçiş kuralı | ray ±0.40 m | ray ±0.60 m | ray ±0.80 m |
|---|---|---|---|---|
| 1.00 | bizim | ❌ kalkmadı (dike en fazla 27°) (0/5) | ❌ kalkmadı (dike en fazla 27°) (0/5) | ❌ kalkmadı (dike en fazla 27°) (0/5) |
| 1.00 | mekatronik | ❌ kalkmadı (dike en fazla 27°) (0/5) | ❌ kalkmadı (dike en fazla 27°) (0/5) | ❌ kalkmadı (dike en fazla 27°) (0/5) |
| 1.01 (= mekatronik 1.02·mgl) | bizim | ❌ kalkmadı (dike en fazla 25°) (0/5) | ❌ kalkmadı (dike en fazla 25°) (0/5) | ❌ kalkmadı (dike en fazla 25°) (0/5) |
| 1.01 (= mekatronik 1.02·mgl) | mekatronik | ❌ kalkmadı (dike en fazla 25°) (0/5) | ❌ kalkmadı (dike en fazla 25°) (0/5) | ❌ kalkmadı (dike en fazla 25°) (0/5) |
| 1.02 | bizim | ❌ kalkmadı (dike en fazla 22°) (0/5) | ❌ kalkmadı (dike en fazla 22°) (0/5) | ❌ kalkmadı (dike en fazla 22°) (0/5) |
| 1.02 | mekatronik | ❌ kalkmadı (dike en fazla 22°) (0/5) | ❌ kalkmadı (dike en fazla 22°) (0/5) | ❌ kalkmadı (dike en fazla 22°) (0/5) |
| 1.05 | bizim | ✅ 7.2 s / 0.14 m (5/5) | ✅ 7.2 s / 0.14 m (5/5) | ✅ 7.2 s / 0.14 m (5/5) |
| 1.05 | mekatronik | ⚠️ 8.3 s / 0.17 m (4/5) | ⚠️ 8.3 s / 0.17 m (4/5) | ⚠️ 8.3 s / 0.17 m (4/5) |
| 1.10 | bizim | ✅ 7.2 s / 0.14 m (5/5) | ✅ 7.2 s / 0.14 m (5/5) | ✅ 7.2 s / 0.14 m (5/5) |
| 1.10 | mekatronik | ❌ dike geldi (0°), yakalama şartı sağlanmadı: |θ̇| (0/5) | ❌ dike geldi (0°), yakalama şartı sağlanmadı: |θ̇| (0/5) | ❌ dike geldi (0°), yakalama şartı sağlanmadı: |θ̇| (0/5) |

## +50 g

**Sürtünme telafisi açık**

| E_ratio | Geçiş kuralı | ray ±0.40 m | ray ±0.60 m | ray ±0.80 m |
|---|---|---|---|---|
| 1.00 | bizim | ⚠️ 3.8 s / 0.22 m (3/5) | ⚠️ 3.8 s / 0.22 m (3/5) | ⚠️ 3.8 s / 0.22 m (3/5) |
| 1.00 | mekatronik | ❌ kalkmadı (dike en fazla 14°) (1/5) | ❌ kalkmadı (dike en fazla 14°) (1/5) | ❌ kalkmadı (dike en fazla 14°) (1/5) |
| 1.01 (= mekatronik 1.02·mgl) | bizim | ✅ 3.8 s / 0.15 m (5/5) | ✅ 3.8 s / 0.15 m (5/5) | ✅ 3.8 s / 0.15 m (5/5) |
| 1.01 (= mekatronik 1.02·mgl) | mekatronik | ⚠️ 3.8 s / 0.15 m (4/5) | ⚠️ 3.8 s / 0.15 m (4/5) | ⚠️ 3.8 s / 0.15 m (4/5) |
| 1.02 | bizim | ✅ 3.7 s / 0.14 m (5/5) | ✅ 3.7 s / 0.14 m (5/5) | ✅ 3.7 s / 0.14 m (5/5) |
| 1.02 | mekatronik | ✅ 3.8 s / 0.14 m (5/5) | ✅ 3.8 s / 0.14 m (5/5) | ✅ 3.8 s / 0.14 m (5/5) |
| 1.05 | bizim | ✅ 3.7 s / 0.15 m (5/5) | ✅ 3.7 s / 0.15 m (5/5) | ✅ 3.7 s / 0.15 m (5/5) |
| 1.05 | mekatronik | ❌ dike geldi (0°), yakalama şartı sağlanmadı: |θ̇| (0/5) | ❌ dike geldi (0°), yakalama şartı sağlanmadı: |θ̇| (0/5) | ❌ dike geldi (0°), yakalama şartı sağlanmadı: |θ̇| (0/5) |
| 1.10 | bizim | ✅ 3.7 s / 0.31 m (5/5) | ✅ 3.7 s / 0.31 m (5/5) | ✅ 3.7 s / 0.31 m (5/5) |
| 1.10 | mekatronik | ❌ dike geldi (0°), yakalama şartı sağlanmadı: |θ̇| (0/5) | ❌ dike geldi (0°), yakalama şartı sağlanmadı: |θ̇| (0/5) | ❌ dike geldi (0°), yakalama şartı sağlanmadı: |θ̇| (0/5) |

**Sürtünme telafisi kapalı**

| E_ratio | Geçiş kuralı | ray ±0.40 m | ray ±0.60 m | ray ±0.80 m |
|---|---|---|---|---|
| 1.00 | bizim | ❌ kalkmadı (dike en fazla 21°) (0/5) | ❌ kalkmadı (dike en fazla 21°) (0/5) | ❌ kalkmadı (dike en fazla 21°) (0/5) |
| 1.00 | mekatronik | ❌ kalkmadı (dike en fazla 21°) (0/5) | ❌ kalkmadı (dike en fazla 21°) (0/5) | ❌ kalkmadı (dike en fazla 21°) (0/5) |
| 1.01 (= mekatronik 1.02·mgl) | bizim | ❌ kalkmadı (dike en fazla 17°) (0/5) | ❌ kalkmadı (dike en fazla 17°) (0/5) | ❌ kalkmadı (dike en fazla 17°) (0/5) |
| 1.01 (= mekatronik 1.02·mgl) | mekatronik | ❌ kalkmadı (dike en fazla 17°) (0/5) | ❌ kalkmadı (dike en fazla 17°) (0/5) | ❌ kalkmadı (dike en fazla 17°) (0/5) |
| 1.02 | bizim | ✅ 5.4 s / 0.15 m (5/5) | ✅ 5.4 s / 0.15 m (5/5) | ✅ 5.4 s / 0.15 m (5/5) |
| 1.02 | mekatronik | ❌ kalkmadı (dike en fazla 12°) (0/5) | ❌ kalkmadı (dike en fazla 12°) (0/5) | ❌ kalkmadı (dike en fazla 12°) (0/5) |
| 1.05 | bizim | ✅ 5.4 s / 0.14 m (5/5) | ✅ 5.4 s / 0.14 m (5/5) | ✅ 5.4 s / 0.14 m (5/5) |
| 1.05 | mekatronik | ✅ 6.5 s / 0.14 m (5/5) | ✅ 6.5 s / 0.14 m (5/5) | ✅ 6.5 s / 0.14 m (5/5) |
| 1.10 | bizim | ⚠️ 5.3 s / 0.37 m (4/5) | ⚠️ 5.3 s / 0.37 m (4/5) | ✅ 5.3 s / 0.37 m (5/5) |
| 1.10 | mekatronik | ❌ dike geldi (0°), yakalama şartı sağlanmadı: |θ̇| (0/5) | ❌ dike geldi (0°), yakalama şartı sağlanmadı: |θ̇| (0/5) | ❌ dike geldi (0°), yakalama şartı sağlanmadı: |θ̇| (0/5) |

## Sonuç

Aşağıdaki maddeleri betik tablolardaki sonuçlardan üretir.

**Sağlam kombinasyonlar** (3 tohumun hepsinde ve m, l ±%5'te kalkan; ✅):
- uç kütlesiz: 3/60 durum sağlam; sürtünme telafisi kapalıyken 0/30.
- +20 g: 12/60 durum sağlam; sürtünme telafisi kapalıyken 6/30.
- +50 g: 25/60 durum sağlam; sürtünme telafisi kapalıyken 10/30.

**Ray uzunluğu:** 60 (uç kütlesi, kural, E_ratio, telafi) grubundan 1'inde sonuç (sağlamlık ya da nominal kalkış) ray boyuyla değişti. Kalkan koşularda en büyük max |x| 0.76 m.

**Kural ve hedef enerji oranına göre gereken en kısa ray** (sağlam olduğu en kısa `rail_half`; telafi açık / kapalı):

| Uç kütlesi | Geçiş kuralı | E_ratio 1.00 | E_ratio 1.01 | E_ratio 1.02 | E_ratio 1.05 | E_ratio 1.10 |
|---|---|---|---|---|---|---|
| uç kütlesiz | bizim | — / — | — / — | — / — | — / — | 0.40 / — |
| uç kütlesiz | mekatronik | — / — | — / — | — / — | — / — | — / — |
| +20 g | bizim | — / — | — / — | — / — | 0.40 / 0.40 | 0.40 / 0.40 |
| +20 g | mekatronik | — / — | — / — | — / — | — / — | — / — |
| +50 g | bizim | — / — | 0.40 / — | 0.40 / 0.40 | 0.40 / 0.40 | 0.40 / 0.80 |
| +50 g | mekatronik | — / — | — / — | 0.40 / — | — / 0.40 | — / — |

**Her ray uzunluğunda ve telafi açık/kapalı sağlam olan (uç kütlesi, kural, E_ratio):**
- +20 g, bizim, E_ratio 1.05 (5 koşunun en büyük max |x|'i 0.30 m)
- +20 g, bizim, E_ratio 1.10 (5 koşunun en büyük max |x|'i 0.36 m)
- +50 g, bizim, E_ratio 1.02 (5 koşunun en büyük max |x|'i 0.26 m)
- +50 g, bizim, E_ratio 1.05 (5 koşunun en büyük max |x|'i 0.19 m)

**Yalnız telafi açıkken (ilk denemeden önce sürtünme ölçülüp gönderilirse) her rayda sağlam olan:**
- uç kütlesiz, bizim, E_ratio 1.10
- +50 g, bizim, E_ratio 1.01
- +50 g, bizim, E_ratio 1.10
- +50 g, mekatronik, E_ratio 1.02

Bu çalışma yalnız karar verisidir: `params.json`, firmware, `supervisor.py` ve swing-up varsayılanları değiştirilmedi (#11 ve iki ekibin ortak kararı).
