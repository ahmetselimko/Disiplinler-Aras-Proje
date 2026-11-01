# Parametre Hassasiyet Analizi Raporu

Issue #13 — Tek seferde bir fiziksel parametre değiştirilir; nominal model ve K sabit kalır.
Gerçek donanım doğrulaması değildir. Parametreler yer tutucu olabilir; sonuçlar yalnız bu modele,
başlangıç durumlarına, sensöre ve deney süresine aittir. Birleşik hatalar ayrıca test edilmelidir.

## Tekrarlanabilir deney ayarları

- Her deney: 12 s; Ts = 0.002 s; sensör: AS5600 12-bit, gürültü yok.
- Başarı: son en az 1 s boyunca BALANCE ve |θ| ≤ 2°; FAULT ve ray teması yok.
- Ayrı başlangıçlar: aşağıdan 180° swing-up; dik çevresinde 5° denge.
- Doyum: sol örnek sabit kabul edilerek |u| ≥ u_max − 0.001 integralidir.
- Nominal kontroller:
  - swing: BAŞARILI; son denge süresi 7.856 s
  - balance: BAŞARILI; son denge süresi 11.928 s

Tüm fiziksel parametreler, K, Q/R ve kontrol ayarları:

```json
{
  "nominal": {
    "M": 0.5,
    "m": 0.2,
    "l": 0.3,
    "I": 0.0,
    "b": 0.1,
    "c": 0.0005,
    "g": 9.81,
    "rail_half": 0.4,
    "u_max": 12.0,
    "input_mode": "voltage",
    "motor": {
      "R": 2.2,
      "Kt": 0.0083,
      "Ke": 0.0109,
      "gear": 10.0,
      "r_pulley": 0.0127,
      "V_max": 12.0,
      "J_rotor": 0.0,
      "T_f": 0.0,
      "B_m": 0.0
    },
    "note": "YER TUTUCU DEĞERLER — mekatronik ekibinden gerçek değerler gelince güncellenecek"
  },
  "settings": {
    "sim_time": 12.0,
    "dt": 0.002,
    "threshold_deg": 2.0,
    "hold_s": 1.0,
    "Q": [
      10.0,
      1.0,
      100.0,
      1.0
    ],
    "R": 0.1,
    "K_source": "nominal Q/R ile tasarlandı",
    "K": [
      -9.568725160198044,
      -22.128073536795796,
      -56.44184372806983,
      -8.714120750703069
    ],
    "variations": [
      -0.3,
      -0.2,
      -0.1,
      0.1,
      0.2,
      0.3
    ],
    "swingup": {
      "k_E": 100.0,
      "a_max": 4.0,
      "k_x": 10.0,
      "k_v": 6.0,
      "a_kick": 2.0,
      "E_target": 0.0
    },
    "catch_deg": 15.0,
    "release_deg": 30.0,
    "sensor": {
      "th_resolution": 0.0015339807878856412,
      "x_resolution": 0.000125,
      "noise_std": 0.0,
      "seed": 0
    },
    "initial_angles_deg": {
      "swing": 180.0,
      "balance": 5.0
    },
    "zero_scales": {
      "I": 0.018,
      "b": 0.1,
      "c": 0.0005,
      "motor.J_rotor": 8.064499999999998e-07,
      "motor.T_f": 0.0012699999999999999,
      "motor.B_m": 1.6128999999999998e-07
    },
    "substeps": 4,
    "vel_fc_hz": 25.0,
    "delay_steps": 0,
    "end_stops": true
  }
}
```

## Ölçüm öncelikleri

| Parametre | Sonuç | Ölçüm önerisi |
|---|---|---|
| `l` — Pivot–ağırlık merkezi mesafesi | KRİTİK (Çok Hassas) | Ölçümü ve model varsayımını öncelikle doğrulayın; küçük tarama sapmalarında başarısızlık var. |
| `I` — Sarkacın ağırlık merkezi etrafındaki ataleti | KRİTİK (Çok Hassas) | Ölçümü ve model varsayımını öncelikle doğrulayın; küçük tarama sapmalarında başarısızlık var. Nominal sıfır: bu, yüzde tolerans değil ek atalet/sürtünme deneyidir. |
| `motor.Ke` — Motor ters EMK sabiti | KRİTİK (Çok Hassas) | Ölçümü ve model varsayımını öncelikle doğrulayın; küçük tarama sapmalarında başarısızlık var. |
| `motor.gear` — Redüktör oranı | KRİTİK (Çok Hassas) | Ölçümü ve model varsayımını öncelikle doğrulayın; küçük tarama sapmalarında başarısızlık var. |
| `motor.r_pulley` — Kasnak yarıçapı | KRİTİK (Çok Hassas) | Ölçümü ve model varsayımını öncelikle doğrulayın; küçük tarama sapmalarında başarısızlık var. |
| `m` — Sarkaç kütlesi | TOLERANSLI (yalnız taranan koşullarda) | Bu koşullarda başarısızlık gözlenmedi; daha geniş tolerans için kanıt değildir. |
| `M` — Araba kütlesi | TOLERANSLI (yalnız taranan koşullarda) | Bu koşullarda başarısızlık gözlenmedi; daha geniş tolerans için kanıt değildir. |
| `b` — Araba viskoz sürtünmesi | TOLERANSLI (yalnız taranan koşullarda) | Bu koşullarda başarısızlık gözlenmedi; daha geniş tolerans için kanıt değildir. |
| `c` — Pivot viskoz sürtünmesi | TOLERANSLI (yalnız taranan koşullarda) | Bu koşullarda başarısızlık gözlenmedi; daha geniş tolerans için kanıt değildir. |
| `motor.R` — Motor direnci | TOLERANSLI (yalnız taranan koşullarda) | Bu koşullarda başarısızlık gözlenmedi; daha geniş tolerans için kanıt değildir. |
| `motor.Kt` — Motor tork sabiti | TOLERANSLI (yalnız taranan koşullarda) | Bu koşullarda başarısızlık gözlenmedi; daha geniş tolerans için kanıt değildir. |
| `motor.J_rotor` — Motor rotor ataleti | TOLERANSLI (yalnız taranan koşullarda) | Bu koşullarda başarısızlık gözlenmedi; daha geniş tolerans için kanıt değildir. Nominal sıfır: bu, yüzde tolerans değil ek atalet/sürtünme deneyidir. |
| `motor.T_f` — Motor Coulomb sürtünme torku | TOLERANSLI (yalnız taranan koşullarda) | Bu koşullarda başarısızlık gözlenmedi; daha geniş tolerans için kanıt değildir. Nominal sıfır: bu, yüzde tolerans değil ek atalet/sürtünme deneyidir. |
| `motor.B_m` — Motor viskoz sürtünmesi | TOLERANSLI (yalnız taranan koşullarda) | Bu koşullarda başarısızlık gözlenmedi; daha geniş tolerans için kanıt değildir. Nominal sıfır: bu, yüzde tolerans değil ek atalet/sürtünme deneyidir. |

## Parametre sapma tablosu

✓ iki deney başarılı; ⚠ başarılı fakat rayın %75'inden fazla kullanılmış; ✗ başarısız; — uygulanamaz.
Sıfır nominal satırlarda +% değerleri, aşağıda belirtilen mutlak referansın oranıdır.

| Parametre | Nominal | -30% | -20% | -10% | +10% | +20% | +30% |
|---|---|---|---|---|---|---|---|
| `l` | 0.3 m | ✗ | ✗ | ✗ | ✓ | ✓ | ✗ |
| `m` | 0.2 kg | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| `M` | 0.5 kg | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| `I` | 0 kg·m² | — | — | — | ✓ | ✗ | ✗ |
| `b` | 0.1 N·s/m | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| `c` | 0.0005 N·m·s/rad | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| `motor.R` | 2.2 Ω | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| `motor.Kt` | 0.0083 N·m/A | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| `motor.Ke` | 0.0109 V·s/rad | ✗ | ✗ | ✗ | ✓ | ✗ | ✗ |
| `motor.gear` | 10 1 | ✗ | ✗ | ✗ | ✓ | ✗ | ✗ |
| `motor.r_pulley` | 0.0127 m | ✗ | ✗ | ✓ | ✗ | ✗ | ✗ |
| `motor.J_rotor` | 0 kg·m² | — | — | — | ✓ | ✓ | ✓ |
| `motor.T_f` | 0 N·m | — | — | — | ✓ | ✓ | ✓ |
| `motor.B_m` | 0 N·m·s/rad | — | — | — | ✓ | ✓ | ✓ |

## Deney ayrıntıları

İlk dengeleme geçmişteki ilk kesintisiz bant girişidir; başarı kararı son denge penceresine dayanır.

### l — Pivot–ağırlık merkezi mesafesi


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 0.21 | swing-up | BAŞARISIZ | — | 0.000 | 0.1408 | 0.000 |
| -30% | 0.21 | denge | BAŞARILI | 0.086 | 11.914 | 0.0486 | 0.000 |
| -20% | 0.24 | swing-up | BAŞARISIZ | — | 0.000 | 0.2246 | 0.000 |
| -20% | 0.24 | denge | BAŞARILI | 0.082 | 11.918 | 0.0488 | 0.000 |
| -10% | 0.27 | swing-up | BAŞARISIZ | — | 0.000 | 0.1551 | 0.000 |
| -10% | 0.27 | denge | BAŞARILI | 0.078 | 11.922 | 0.0491 | 0.000 |
| +10% | 0.33 | swing-up | BAŞARILI | 5.334 | 6.666 | 0.1834 | 0.000 |
| +10% | 0.33 | denge | BAŞARILI | 0.068 | 11.932 | 0.0501 | 0.000 |
| +20% | 0.36 | swing-up | BAŞARILI | 5.430 | 6.570 | 0.2784 | 0.000 |
| +20% | 0.36 | denge | BAŞARILI | 0.066 | 11.934 | 0.0512 | 0.000 |
| +30% | 0.39 | swing-up | RAY SINIRI | — | 0.000 | 0.4000 | 0.218 |
| +30% | 0.39 | denge | BAŞARILI | 0.064 | 11.936 | 0.0523 | 0.000 |
### m — Sarkaç kütlesi


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 0.14 | swing-up | BAŞARILI | 4.286 | 7.714 | 0.1751 | 0.000 |
| -30% | 0.14 | denge | BAŞARILI | 0.072 | 11.928 | 0.0498 | 0.000 |
| -20% | 0.16 | swing-up | BAŞARILI | 4.002 | 7.998 | 0.1704 | 0.000 |
| -20% | 0.16 | denge | BAŞARILI | 0.072 | 11.928 | 0.0496 | 0.000 |
| -10% | 0.18 | swing-up | BAŞARILI | 4.086 | 7.914 | 0.1657 | 0.000 |
| -10% | 0.18 | denge | BAŞARILI | 0.072 | 11.928 | 0.0496 | 0.000 |
| +10% | 0.22 | swing-up | BAŞARILI | 5.208 | 6.792 | 0.2552 | 0.018 |
| +10% | 0.22 | denge | BAŞARILI | 0.072 | 11.928 | 0.0496 | 0.000 |
| +20% | 0.24 | swing-up | BAŞARILI | 4.500 | 7.500 | 0.1549 | 0.000 |
| +20% | 0.24 | denge | BAŞARILI | 0.072 | 11.928 | 0.0498 | 0.000 |
| +30% | 0.26 | swing-up | BAŞARILI | 6.130 | 5.870 | 0.2810 | 0.034 |
| +30% | 0.26 | denge | BAŞARILI | 0.072 | 11.928 | 0.0496 | 0.000 |
### M — Araba kütlesi


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 0.35 | swing-up | BAŞARILI | 3.910 | 8.090 | 0.1621 | 0.000 |
| -30% | 0.35 | denge | BAŞARILI | 0.072 | 11.928 | 0.0494 | 0.000 |
| -20% | 0.4 | swing-up | BAŞARILI | 3.980 | 8.020 | 0.1633 | 0.000 |
| -20% | 0.4 | denge | BAŞARILI | 0.072 | 11.928 | 0.0494 | 0.000 |
| -10% | 0.45 | swing-up | BAŞARILI | 4.020 | 7.980 | 0.1609 | 0.000 |
| -10% | 0.45 | denge | BAŞARILI | 0.072 | 11.928 | 0.0495 | 0.000 |
| +10% | 0.55 | swing-up | BAŞARILI | 5.390 | 6.610 | 0.2185 | 0.010 |
| +10% | 0.55 | denge | BAŞARILI | 0.072 | 11.928 | 0.0496 | 0.000 |
| +20% | 0.6 | swing-up | BAŞARILI | 5.150 | 6.850 | 0.2199 | 0.010 |
| +20% | 0.6 | denge | BAŞARILI | 0.074 | 11.926 | 0.0498 | 0.000 |
| +30% | 0.65 | swing-up | BAŞARILI | 4.598 | 7.402 | 0.1591 | 0.000 |
| +30% | 0.65 | denge | BAŞARILI | 0.074 | 11.926 | 0.0498 | 0.000 |
### I — Sarkacın ağırlık merkezi etrafındaki ataleti

Sıfır nominal için mutlak referans: 0.018 kg·m²; yalnız pozitif ekleme yapılır.

| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|

-30%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.


-20%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.


-10%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.

| +10% | 0.0018 | swing-up | BAŞARILI | 5.486 | 6.514 | 0.1695 | 0.000 |
| +10% | 0.0018 | denge | BAŞARILI | 0.068 | 11.932 | 0.0503 | 0.000 |
| +20% | 0.0036 | swing-up | RAY SINIRI | — | 0.000 | 0.4000 | 0.164 |
| +20% | 0.0036 | denge | BAŞARILI | 0.066 | 11.934 | 0.0515 | 0.000 |
| +30% | 0.0054 | swing-up | RAY SINIRI | — | 0.000 | 0.4000 | 0.188 |
| +30% | 0.0054 | denge | BAŞARILI | 0.066 | 11.934 | 0.0527 | 0.000 |
### b — Araba viskoz sürtünmesi


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 0.07 | swing-up | BAŞARILI | 3.800 | 8.200 | 0.1626 | 0.000 |
| -30% | 0.07 | denge | BAŞARILI | 0.072 | 11.928 | 0.0497 | 0.000 |
| -20% | 0.08 | swing-up | BAŞARILI | 3.790 | 8.210 | 0.1634 | 0.000 |
| -20% | 0.08 | denge | BAŞARILI | 0.072 | 11.928 | 0.0498 | 0.000 |
| -10% | 0.09 | swing-up | BAŞARILI | 3.992 | 8.008 | 0.1631 | 0.000 |
| -10% | 0.09 | denge | BAŞARILI | 0.072 | 11.928 | 0.0496 | 0.000 |
| +10% | 0.11 | swing-up | BAŞARILI | 4.234 | 7.766 | 0.1614 | 0.004 |
| +10% | 0.11 | denge | BAŞARILI | 0.072 | 11.928 | 0.0497 | 0.000 |
| +20% | 0.12 | swing-up | BAŞARILI | 4.262 | 7.738 | 0.1654 | 0.010 |
| +20% | 0.12 | denge | BAŞARILI | 0.072 | 11.928 | 0.0496 | 0.000 |
| +30% | 0.13 | swing-up | BAŞARILI | 4.268 | 7.732 | 0.1687 | 0.010 |
| +30% | 0.13 | denge | BAŞARILI | 0.072 | 11.928 | 0.0497 | 0.000 |
### c — Pivot viskoz sürtünmesi


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 0.00035 | swing-up | BAŞARILI | 3.812 | 8.188 | 0.1610 | 0.000 |
| -30% | 0.00035 | denge | BAŞARILI | 0.072 | 11.928 | 0.0496 | 0.000 |
| -20% | 0.0004 | swing-up | BAŞARILI | 3.786 | 8.214 | 0.1599 | 0.000 |
| -20% | 0.0004 | denge | BAŞARILI | 0.072 | 11.928 | 0.0497 | 0.000 |
| -10% | 0.00045 | swing-up | BAŞARILI | 3.780 | 8.220 | 0.1604 | 0.000 |
| -10% | 0.00045 | denge | BAŞARILI | 0.072 | 11.928 | 0.0495 | 0.000 |
| +10% | 0.00055 | swing-up | BAŞARILI | 4.276 | 7.724 | 0.1713 | 0.010 |
| +10% | 0.00055 | denge | BAŞARILI | 0.072 | 11.928 | 0.0497 | 0.000 |
| +20% | 0.0006 | swing-up | BAŞARILI | 4.386 | 7.614 | 0.2191 | 0.024 |
| +20% | 0.0006 | denge | BAŞARILI | 0.072 | 11.928 | 0.0497 | 0.000 |
| +30% | 0.00065 | swing-up | BAŞARILI | 4.446 | 7.554 | 0.2544 | 0.038 |
| +30% | 0.00065 | denge | BAŞARILI | 0.072 | 11.928 | 0.0497 | 0.000 |
### motor.R — Motor direnci


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 1.54 | swing-up | BAŞARILI | 3.572 | 8.428 | 0.1776 | 0.000 |
| -30% | 1.54 | denge | BAŞARILI | 0.070 | 11.930 | 0.0494 | 0.000 |
| -20% | 1.76 | swing-up | BAŞARILI | 3.530 | 8.470 | 0.1726 | 0.000 |
| -20% | 1.76 | denge | BAŞARILI | 0.072 | 11.928 | 0.0493 | 0.000 |
| -10% | 1.98 | swing-up | BAŞARILI | 3.968 | 8.032 | 0.1679 | 0.000 |
| -10% | 1.98 | denge | BAŞARILI | 0.070 | 11.930 | 0.0494 | 0.000 |
| +10% | 2.42 | swing-up | BAŞARILI | 4.888 | 7.112 | 0.1576 | 0.000 |
| +10% | 2.42 | denge | BAŞARILI | 0.072 | 11.928 | 0.0497 | 0.000 |
| +20% | 2.64 | swing-up | BAŞARILI | 4.710 | 7.290 | 0.1523 | 0.000 |
| +20% | 2.64 | denge | BAŞARILI | 0.074 | 11.926 | 0.0500 | 0.000 |
| +30% | 2.86 | swing-up | BAŞARILI | 5.114 | 6.886 | 0.1479 | 0.000 |
| +30% | 2.86 | denge | BAŞARILI | 0.076 | 11.924 | 0.0500 | 0.000 |
### motor.Kt — Motor tork sabiti


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 0.00581 | swing-up | BAŞARILI | 6.446 | 5.554 | 0.2521 | 0.036 |
| -30% | 0.00581 | denge | BAŞARILI | 0.076 | 11.924 | 0.0505 | 0.000 |
| -20% | 0.00664 | swing-up | BAŞARILI | 5.742 | 6.258 | 0.2083 | 0.014 |
| -20% | 0.00664 | denge | BAŞARILI | 0.074 | 11.926 | 0.0500 | 0.000 |
| -10% | 0.00747 | swing-up | BAŞARILI | 4.560 | 7.440 | 0.1565 | 0.000 |
| -10% | 0.00747 | denge | BAŞARILI | 0.072 | 11.928 | 0.0498 | 0.000 |
| +10% | 0.00913 | swing-up | BAŞARILI | 3.972 | 8.028 | 0.1655 | 0.000 |
| +10% | 0.00913 | denge | BAŞARILI | 0.072 | 11.928 | 0.0494 | 0.000 |
| +20% | 0.00996 | swing-up | BAŞARILI | 3.648 | 8.352 | 0.1693 | 0.008 |
| +20% | 0.00996 | denge | BAŞARILI | 0.072 | 11.928 | 0.0496 | 0.000 |
| +30% | 0.01079 | swing-up | BAŞARILI | 3.606 | 8.394 | 0.1742 | 0.000 |
| +30% | 0.01079 | denge | BAŞARILI | 0.072 | 11.928 | 0.0495 | 0.000 |
### motor.Ke — Motor ters EMK sabiti


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 0.00763 | swing-up | RAY SINIRI | — | 0.000 | 0.4000 | 0.128 |
| -30% | 0.00763 | denge | BAŞARILI | 0.062 | 11.938 | 0.0441 | 0.000 |
| -20% | 0.00872 | swing-up | FAULT | — | 0.000 | 0.4000 | 0.092 |
| -20% | 0.00872 | denge | BAŞARILI | 0.064 | 11.936 | 0.0458 | 0.000 |
| -10% | 0.00981 | swing-up | RAY SINIRI | — | 0.000 | 0.4000 | 0.000 |
| -10% | 0.00981 | denge | BAŞARILI | 0.070 | 11.930 | 0.0476 | 0.000 |
| +10% | 0.01199 | swing-up | BAŞARILI | 7.878 | 4.122 | 0.1459 | 0.000 |
| +10% | 0.01199 | denge | BAŞARILI | 0.076 | 11.924 | 0.0518 | 0.000 |
| +20% | 0.01308 | swing-up | BAŞARISIZ | — | 0.000 | 0.0736 | 0.000 |
| +20% | 0.01308 | denge | BAŞARILI | 0.080 | 11.920 | 0.0541 | 0.000 |
| +30% | 0.01417 | swing-up | BAŞARISIZ | — | 0.000 | 0.0579 | 0.000 |
| +30% | 0.01417 | denge | BAŞARILI | 0.084 | 11.916 | 0.0566 | 0.000 |
### motor.gear — Redüktör oranı


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 7 | swing-up | RAY SINIRI | — | 0.000 | 0.4000 | 0.104 |
| -30% | 7 | denge | BAŞARILI | 0.066 | 11.934 | 0.0448 | 0.000 |
| -20% | 8 | swing-up | FAULT | — | 0.000 | 0.4000 | 0.054 |
| -20% | 8 | denge | BAŞARILI | 0.068 | 11.932 | 0.0463 | 0.000 |
| -10% | 9 | swing-up | RAY SINIRI | — | 0.000 | 0.4000 | 0.000 |
| -10% | 9 | denge | BAŞARILI | 0.070 | 11.930 | 0.0478 | 0.000 |
| +10% | 11 | swing-up | BAŞARILI | 6.886 | 5.114 | 0.1082 | 0.000 |
| +10% | 11 | denge | BAŞARILI | 0.076 | 11.924 | 0.0518 | 0.000 |
| +20% | 12 | swing-up | BAŞARISIZ | — | 0.246 | 0.1293 | 0.000 |
| +20% | 12 | denge | BAŞARILI | 0.080 | 11.920 | 0.0539 | 0.000 |
| +30% | 13 | swing-up | BAŞARISIZ | — | 0.000 | 0.0623 | 0.000 |
| +30% | 13 | denge | BAŞARILI | 0.084 | 11.916 | 0.0563 | 0.000 |
### motor.r_pulley — Kasnak yarıçapı


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 0.00889 | swing-up | BAŞARISIZ | — | 0.000 | 0.0485 | 0.000 |
| -30% | 0.00889 | denge | BAŞARILI | 0.088 | 11.912 | 0.0599 | 0.000 |
| -20% | 0.01016 | swing-up | BAŞARISIZ | — | 0.000 | 0.0685 | 0.000 |
| -20% | 0.01016 | denge | BAŞARILI | 0.080 | 11.920 | 0.0550 | 0.000 |
| -10% | 0.01143 | swing-up | BAŞARILI | 8.044 | 3.956 | 0.1886 | 0.000 |
| -10% | 0.01143 | denge | BAŞARILI | 0.076 | 11.924 | 0.0518 | 0.000 |
| +10% | 0.01397 | swing-up | RAY SINIRI | — | 0.000 | 0.4000 | 0.000 |
| +10% | 0.01397 | denge | BAŞARILI | 0.070 | 11.930 | 0.0478 | 0.000 |
| +20% | 0.01524 | swing-up | RAY SINIRI | — | 0.000 | 0.4000 | 0.010 |
| +20% | 0.01524 | denge | BAŞARILI | 0.068 | 11.932 | 0.0467 | 0.000 |
| +30% | 0.01651 | swing-up | FAULT | — | 0.000 | 0.4000 | 0.080 |
| +30% | 0.01651 | denge | BAŞARILI | 0.066 | 11.934 | 0.0458 | 0.000 |
### motor.J_rotor — Motor rotor ataleti

Sıfır nominal için mutlak referans: 8.0645e-07 kg·m²; yalnız pozitif ekleme yapılır.

| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|

-30%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.


-20%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.


-10%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.

| +10% | 8.0645e-08 | swing-up | BAŞARILI | 5.390 | 6.610 | 0.2185 | 0.010 |
| +10% | 8.0645e-08 | denge | BAŞARILI | 0.072 | 11.928 | 0.0496 | 0.000 |
| +20% | 1.6129e-07 | swing-up | BAŞARILI | 5.150 | 6.850 | 0.2199 | 0.010 |
| +20% | 1.6129e-07 | denge | BAŞARILI | 0.074 | 11.926 | 0.0498 | 0.000 |
| +30% | 2.41935e-07 | swing-up | BAŞARILI | 4.598 | 7.402 | 0.1591 | 0.000 |
| +30% | 2.41935e-07 | denge | BAŞARILI | 0.074 | 11.926 | 0.0498 | 0.000 |
### motor.T_f — Motor Coulomb sürtünme torku

Sıfır nominal için mutlak referans: 0.00127 N·m; yalnız pozitif ekleme yapılır.

| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|

-30%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.


-20%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.


-10%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.

| +10% | 0.000127 | swing-up | BAŞARILI | 5.150 | 6.850 | 0.2280 | 0.010 |
| +10% | 0.000127 | denge | BAŞARILI | 0.072 | 11.928 | 0.0503 | 0.000 |
| +20% | 0.000254 | swing-up | BAŞARILI | 4.524 | 7.476 | 0.1388 | 0.000 |
| +20% | 0.000254 | denge | BAŞARILI | 0.074 | 11.926 | 0.0506 | 0.000 |
| +30% | 0.000381 | swing-up | BAŞARILI | 5.870 | 6.130 | 0.2104 | 0.010 |
| +30% | 0.000381 | denge | BAŞARILI | 0.074 | 11.926 | 0.0510 | 0.000 |
### motor.B_m — Motor viskoz sürtünmesi

Sıfır nominal için mutlak referans: 1.6129e-07 N·m·s/rad; yalnız pozitif ekleme yapılır.

| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|

-30%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.


-20%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.


-10%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.

| +10% | 1.6129e-08 | swing-up | BAŞARILI | 4.234 | 7.766 | 0.1614 | 0.004 |
| +10% | 1.6129e-08 | denge | BAŞARILI | 0.072 | 11.928 | 0.0497 | 0.000 |
| +20% | 3.2258e-08 | swing-up | BAŞARILI | 4.262 | 7.738 | 0.1654 | 0.010 |
| +20% | 3.2258e-08 | denge | BAŞARILI | 0.072 | 11.928 | 0.0496 | 0.000 |
| +30% | 4.8387e-08 | swing-up | BAŞARILI | 4.268 | 7.732 | 0.1687 | 0.010 |
| +30% | 4.8387e-08 | denge | BAŞARILI | 0.072 | 11.928 | 0.0497 | 0.000 |

## Sınırlar ve yeniden çalıştırma

- Sıfır nominal referansları keşif ölçekleridir; ölçülmüş motor verisi değildir.
- I için referans m·l²; rotor için arabaya yansıması M olan atalet; T_f için arabada 1 N; B_m için max(b, 0.1) N·s/m kullanılır.
- Gerçek I / motor kayıpları ölçüldüğünde params.json güncellenmeli; ardından ±% taraması yeniden çalıştırılmalı.
- Motor parametreleri yalnız voltage modunda değerlendirilir. V_max tek başına kuvvet modelini değiştirmediğinden taranmaz; giriş sınırı u_max sabittir.
- Kontrolcü yeniden tasarlanmaz; fiziksel parametre uyumsuzluğu ölçülür. Ölçüm ölçek hataları, gecikme ve birleşik sapmalar bu deneyin dışında kalır.
- Çalıştır: `python tools/hassasiyet_analizi.py`; makine tarafından okunabilir sonuçlar JSON dosyasına da yazılır.
