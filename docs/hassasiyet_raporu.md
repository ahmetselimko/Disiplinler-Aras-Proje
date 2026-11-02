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
  - swing: BAŞARILI; son denge süresi 7.784 s
  - balance: BAŞARILI; son denge süresi 11.930 s

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
      "R": 2.18,
      "Kt": 0.00892,
      "Ke": 0.01104,
      "gear": 10.0,
      "r_pulley": 0.0127,
      "V_max": 12.0,
      "J_rotor": 0.0,
      "T_f": 0.0,
      "B_m": 0.0
    },
    "note": "YER TUTUCU DEĞERLER — mekatronik ekibinden gerçek değerler gelince güncellenecek",
    "placeholder": true
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
      -9.545742134556576,
      -22.272551810525904,
      -56.22180714577046,
      -8.686515342958801
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
      "motor.T_f": 0.00892,
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
| `motor.T_f` — Motor Coulomb sürtünme torku | KRİTİK (Çok Hassas) | Ölçümü ve model varsayımını öncelikle doğrulayın; küçük tarama sapmalarında başarısızlık var. Nominal sıfır: bu, yüzde tolerans değil ek atalet/sürtünme deneyidir. |
| `m` — Sarkaç kütlesi | ORTA HASSASİYET | Ölçümü doğrulayın; taramada başarısızlık veya ray sınırına yaklaşma var. |
| `M` — Araba kütlesi | TOLERANSLI (yalnız taranan koşullarda) | Bu koşullarda başarısızlık gözlenmedi; daha geniş tolerans için kanıt değildir. |
| `b` — Araba viskoz sürtünmesi | TOLERANSLI (yalnız taranan koşullarda) | Bu koşullarda başarısızlık gözlenmedi; daha geniş tolerans için kanıt değildir. |
| `c` — Pivot viskoz sürtünmesi | TOLERANSLI (yalnız taranan koşullarda) | Bu koşullarda başarısızlık gözlenmedi; daha geniş tolerans için kanıt değildir. |
| `motor.R` — Motor direnci | TOLERANSLI (yalnız taranan koşullarda) | Bu koşullarda başarısızlık gözlenmedi; daha geniş tolerans için kanıt değildir. |
| `motor.Kt` — Motor tork sabiti | TOLERANSLI (yalnız taranan koşullarda) | Bu koşullarda başarısızlık gözlenmedi; daha geniş tolerans için kanıt değildir. |
| `motor.J_rotor` — Motor rotor ataleti | TOLERANSLI (yalnız taranan koşullarda) | Bu koşullarda başarısızlık gözlenmedi; daha geniş tolerans için kanıt değildir. Nominal sıfır: bu, yüzde tolerans değil ek atalet/sürtünme deneyidir. |
| `motor.B_m` — Motor viskoz sürtünmesi | TOLERANSLI (yalnız taranan koşullarda) | Bu koşullarda başarısızlık gözlenmedi; daha geniş tolerans için kanıt değildir. Nominal sıfır: bu, yüzde tolerans değil ek atalet/sürtünme deneyidir. |

## Parametre sapma tablosu

✓ iki deney başarılı; ⚠ başarılı fakat rayın %75'inden fazla kullanılmış; ✗ başarısız; — uygulanamaz.
Sıfır nominal satırlarda +% değerleri, aşağıda belirtilen mutlak referansın oranıdır.

| Parametre | Nominal | -30% | -20% | -10% | +10% | +20% | +30% |
|---|---|---|---|---|---|---|---|
| `l` | 0.3 m | ✗ | ✗ | ✗ | ✓ | ✓ | ✗ |
| `m` | 0.2 kg | ✓ | ✓ | ✓ | ✓ | ✓ | ✗ |
| `M` | 0.5 kg | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| `I` | 0 kg·m² | — | — | — | ✓ | ✗ | ✗ |
| `b` | 0.1 N·s/m | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| `c` | 0.0005 N·m·s/rad | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| `motor.R` | 2.18 Ω | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| `motor.Kt` | 0.00892 N·m/A | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| `motor.Ke` | 0.01104 V·s/rad | ✗ | ✗ | ✗ | ✓ | ✗ | ✗ |
| `motor.gear` | 10 1 | ✗ | ✗ | ✗ | ✓ | ✗ | ✗ |
| `motor.r_pulley` | 0.0127 m | ✗ | ✗ | ✓ | ✗ | ✗ | ✗ |
| `motor.J_rotor` | 0 kg·m² | — | — | — | ✓ | ✓ | ✓ |
| `motor.T_f` | 0 N·m | — | — | — | ✓ | ✗ | ✗ |
| `motor.B_m` | 0 N·m·s/rad | — | — | — | ✓ | ✓ | ✓ |

## Deney ayrıntıları

İlk dengeleme geçmişteki ilk kesintisiz bant girişidir; başarı kararı son denge penceresine dayanır.

### l — Pivot–ağırlık merkezi mesafesi


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 0.21 | swing-up | BAŞARISIZ | — | 0.000 | 0.1398 | 0.000 |
| -30% | 0.21 | denge | BAŞARILI | 0.086 | 11.914 | 0.0482 | 0.000 |
| -20% | 0.24 | swing-up | BAŞARISIZ | — | 0.000 | 0.1794 | 0.000 |
| -20% | 0.24 | denge | BAŞARILI | 0.082 | 11.918 | 0.0484 | 0.000 |
| -10% | 0.27 | swing-up | BAŞARISIZ | — | 0.000 | 0.1535 | 0.000 |
| -10% | 0.27 | denge | BAŞARILI | 0.076 | 11.924 | 0.0490 | 0.000 |
| +10% | 0.33 | swing-up | BAŞARILI | 5.330 | 6.670 | 0.1995 | 0.000 |
| +10% | 0.33 | denge | BAŞARILI | 0.068 | 11.932 | 0.0500 | 0.000 |
| +20% | 0.36 | swing-up | BAŞARILI | 5.402 | 6.598 | 0.2400 | 0.000 |
| +20% | 0.36 | denge | BAŞARILI | 0.064 | 11.936 | 0.0510 | 0.000 |
| +30% | 0.39 | swing-up | RAY SINIRI | — | 0.000 | 0.4000 | 0.212 |
| +30% | 0.39 | denge | BAŞARILI | 0.062 | 11.938 | 0.0522 | 0.000 |
### m — Sarkaç kütlesi


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 0.14 | swing-up | BAŞARILI | 4.184 | 7.816 | 0.1742 | 0.000 |
| -30% | 0.14 | denge | BAŞARILI | 0.070 | 11.930 | 0.0493 | 0.000 |
| -20% | 0.16 | swing-up | BAŞARILI | 4.022 | 7.978 | 0.1686 | 0.000 |
| -20% | 0.16 | denge | BAŞARILI | 0.070 | 11.930 | 0.0493 | 0.000 |
| -10% | 0.18 | swing-up | BAŞARILI | 4.178 | 7.822 | 0.1639 | 0.000 |
| -10% | 0.18 | denge | BAŞARILI | 0.072 | 11.928 | 0.0494 | 0.000 |
| +10% | 0.22 | swing-up | BAŞARILI | 5.096 | 6.904 | 0.2106 | 0.006 |
| +10% | 0.22 | denge | BAŞARILI | 0.070 | 11.930 | 0.0494 | 0.000 |
| +20% | 0.24 | swing-up | BAŞARILI | 4.496 | 7.504 | 0.1529 | 0.000 |
| +20% | 0.24 | denge | BAŞARILI | 0.072 | 11.928 | 0.0493 | 0.000 |
| +30% | 0.26 | swing-up | BAŞARISIZ | — | 0.000 | 0.1514 | 0.000 |
| +30% | 0.26 | denge | BAŞARILI | 0.072 | 11.928 | 0.0493 | 0.000 |
### M — Araba kütlesi


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 0.35 | swing-up | BAŞARILI | 3.940 | 8.060 | 0.1616 | 0.000 |
| -30% | 0.35 | denge | BAŞARILI | 0.070 | 11.930 | 0.0493 | 0.000 |
| -20% | 0.4 | swing-up | BAŞARILI | 4.016 | 7.984 | 0.1627 | 0.000 |
| -20% | 0.4 | denge | BAŞARILI | 0.070 | 11.930 | 0.0493 | 0.000 |
| -10% | 0.45 | swing-up | BAŞARILI | 3.824 | 8.176 | 0.1629 | 0.000 |
| -10% | 0.45 | denge | BAŞARILI | 0.072 | 11.928 | 0.0496 | 0.000 |
| +10% | 0.55 | swing-up | BAŞARILI | 5.328 | 6.672 | 0.2409 | 0.016 |
| +10% | 0.55 | denge | BAŞARILI | 0.072 | 11.928 | 0.0494 | 0.000 |
| +20% | 0.6 | swing-up | BAŞARILI | 4.960 | 7.040 | 0.1708 | 0.000 |
| +20% | 0.6 | denge | BAŞARILI | 0.072 | 11.928 | 0.0496 | 0.000 |
| +30% | 0.65 | swing-up | BAŞARILI | 4.586 | 7.414 | 0.1601 | 0.000 |
| +30% | 0.65 | denge | BAŞARILI | 0.074 | 11.926 | 0.0498 | 0.000 |
### I — Sarkacın ağırlık merkezi etrafındaki ataleti

Sıfır nominal için mutlak referans: 0.018 kg·m²; yalnız pozitif ekleme yapılır.

| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|

-30%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.


-20%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.


-10%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.

| +10% | 0.0018 | swing-up | BAŞARILI | 5.490 | 6.510 | 0.1719 | 0.000 |
| +10% | 0.0018 | denge | BAŞARILI | 0.068 | 11.932 | 0.0500 | 0.000 |
| +20% | 0.0036 | swing-up | RAY SINIRI | — | 0.000 | 0.4000 | 0.156 |
| +20% | 0.0036 | denge | BAŞARILI | 0.066 | 11.934 | 0.0511 | 0.000 |
| +30% | 0.0054 | swing-up | RAY SINIRI | — | 0.000 | 0.4000 | 0.186 |
| +30% | 0.0054 | denge | BAŞARILI | 0.064 | 11.936 | 0.0523 | 0.000 |
### b — Araba viskoz sürtünmesi


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 0.07 | swing-up | BAŞARILI | 4.104 | 7.896 | 0.1613 | 0.000 |
| -30% | 0.07 | denge | BAŞARILI | 0.072 | 11.928 | 0.0493 | 0.000 |
| -20% | 0.08 | swing-up | BAŞARILI | 4.144 | 7.856 | 0.1616 | 0.000 |
| -20% | 0.08 | denge | BAŞARILI | 0.072 | 11.928 | 0.0494 | 0.000 |
| -10% | 0.09 | swing-up | BAŞARILI | 4.172 | 7.828 | 0.1623 | 0.000 |
| -10% | 0.09 | denge | BAŞARILI | 0.072 | 11.928 | 0.0493 | 0.000 |
| +10% | 0.11 | swing-up | BAŞARILI | 4.242 | 7.758 | 0.1596 | 0.008 |
| +10% | 0.11 | denge | BAŞARILI | 0.070 | 11.930 | 0.0494 | 0.000 |
| +20% | 0.12 | swing-up | BAŞARILI | 4.294 | 7.706 | 0.1776 | 0.010 |
| +20% | 0.12 | denge | BAŞARILI | 0.072 | 11.928 | 0.0494 | 0.000 |
| +30% | 0.13 | swing-up | BAŞARILI | 4.284 | 7.716 | 0.1748 | 0.010 |
| +30% | 0.13 | denge | BAŞARILI | 0.070 | 11.930 | 0.0492 | 0.000 |
### c — Pivot viskoz sürtünmesi


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 0.00035 | swing-up | BAŞARILI | 3.796 | 8.204 | 0.1622 | 0.000 |
| -30% | 0.00035 | denge | BAŞARILI | 0.072 | 11.928 | 0.0493 | 0.000 |
| -20% | 0.0004 | swing-up | BAŞARILI | 3.788 | 8.212 | 0.1622 | 0.000 |
| -20% | 0.0004 | denge | BAŞARILI | 0.070 | 11.930 | 0.0494 | 0.000 |
| -10% | 0.00045 | swing-up | BAŞARILI | 4.096 | 7.904 | 0.1621 | 0.000 |
| -10% | 0.00045 | denge | BAŞARILI | 0.072 | 11.928 | 0.0493 | 0.000 |
| +10% | 0.00055 | swing-up | BAŞARILI | 4.296 | 7.704 | 0.1793 | 0.014 |
| +10% | 0.00055 | denge | BAŞARILI | 0.070 | 11.930 | 0.0494 | 0.000 |
| +20% | 0.0006 | swing-up | BAŞARILI | 4.356 | 7.644 | 0.2075 | 0.022 |
| +20% | 0.0006 | denge | BAŞARILI | 0.070 | 11.930 | 0.0494 | 0.000 |
| +30% | 0.00065 | swing-up | BAŞARILI | 5.400 | 6.600 | 0.2206 | 0.014 |
| +30% | 0.00065 | denge | BAŞARILI | 0.070 | 11.930 | 0.0494 | 0.000 |
### motor.R — Motor direnci


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 1.526 | swing-up | BAŞARILI | 3.604 | 8.396 | 0.1772 | 0.000 |
| -30% | 1.526 | denge | BAŞARILI | 0.070 | 11.930 | 0.0492 | 0.000 |
| -20% | 1.744 | swing-up | BAŞARILI | 3.292 | 8.708 | 0.1721 | 0.000 |
| -20% | 1.744 | denge | BAŞARILI | 0.070 | 11.930 | 0.0493 | 0.000 |
| -10% | 1.962 | swing-up | BAŞARILI | 3.980 | 8.020 | 0.1671 | 0.000 |
| -10% | 1.962 | denge | BAŞARILI | 0.072 | 11.928 | 0.0493 | 0.000 |
| +10% | 2.398 | swing-up | BAŞARILI | 4.556 | 7.444 | 0.1568 | 0.000 |
| +10% | 2.398 | denge | BAŞARILI | 0.072 | 11.928 | 0.0495 | 0.000 |
| +20% | 2.616 | swing-up | BAŞARILI | 6.126 | 5.874 | 0.2758 | 0.032 |
| +20% | 2.616 | denge | BAŞARILI | 0.072 | 11.928 | 0.0495 | 0.000 |
| +30% | 2.834 | swing-up | BAŞARILI | 5.110 | 6.890 | 0.1487 | 0.000 |
| +30% | 2.834 | denge | BAŞARILI | 0.074 | 11.926 | 0.0499 | 0.000 |
### motor.Kt — Motor tork sabiti


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 0.006244 | swing-up | BAŞARILI | 6.312 | 5.688 | 0.1896 | 0.014 |
| -30% | 0.006244 | denge | BAŞARILI | 0.076 | 11.924 | 0.0502 | 0.000 |
| -20% | 0.007136 | swing-up | BAŞARILI | 5.516 | 6.484 | 0.1494 | 0.000 |
| -20% | 0.007136 | denge | BAŞARILI | 0.074 | 11.926 | 0.0497 | 0.000 |
| -10% | 0.008028 | swing-up | BAŞARILI | 4.542 | 7.458 | 0.1550 | 0.000 |
| -10% | 0.008028 | denge | BAŞARILI | 0.072 | 11.928 | 0.0495 | 0.000 |
| +10% | 0.009812 | swing-up | BAŞARILI | 4.008 | 7.992 | 0.1646 | 0.000 |
| +10% | 0.009812 | denge | BAŞARILI | 0.072 | 11.928 | 0.0493 | 0.000 |
| +20% | 0.010704 | swing-up | BAŞARILI | 4.426 | 7.574 | 0.1839 | 0.000 |
| +20% | 0.010704 | denge | BAŞARILI | 0.072 | 11.928 | 0.0491 | 0.000 |
| +30% | 0.011596 | swing-up | BAŞARILI | 3.608 | 8.392 | 0.1746 | 0.000 |
| +30% | 0.011596 | denge | BAŞARILI | 0.070 | 11.930 | 0.0493 | 0.000 |
### motor.Ke — Motor ters EMK sabiti


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 0.007728 | swing-up | RAY SINIRI | — | 0.000 | 0.4000 | 0.136 |
| -30% | 0.007728 | denge | BAŞARILI | 0.060 | 11.940 | 0.0439 | 0.000 |
| -20% | 0.008832 | swing-up | FAULT | — | 0.000 | 0.4000 | 0.106 |
| -20% | 0.008832 | denge | BAŞARILI | 0.064 | 11.936 | 0.0455 | 0.000 |
| -10% | 0.009936 | swing-up | RAY SINIRI | — | 0.000 | 0.4000 | 0.000 |
| -10% | 0.009936 | denge | BAŞARILI | 0.066 | 11.934 | 0.0474 | 0.000 |
| +10% | 0.012144 | swing-up | BAŞARILI | 8.976 | 3.024 | 0.2485 | 0.020 |
| +10% | 0.012144 | denge | BAŞARILI | 0.076 | 11.924 | 0.0516 | 0.000 |
| +20% | 0.013248 | swing-up | BAŞARISIZ | — | 0.000 | 0.0701 | 0.000 |
| +20% | 0.013248 | denge | BAŞARILI | 0.080 | 11.920 | 0.0540 | 0.000 |
| +30% | 0.014352 | swing-up | BAŞARISIZ | — | 0.000 | 0.0535 | 0.000 |
| +30% | 0.014352 | denge | BAŞARILI | 0.084 | 11.916 | 0.0565 | 0.000 |
### motor.gear — Redüktör oranı


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 7 | swing-up | FAULT | — | 0.000 | 0.4000 | 0.118 |
| -30% | 7 | denge | BAŞARILI | 0.064 | 11.936 | 0.0446 | 0.000 |
| -20% | 8 | swing-up | FAULT | — | 0.000 | 0.4000 | 0.076 |
| -20% | 8 | denge | BAŞARILI | 0.066 | 11.934 | 0.0458 | 0.000 |
| -10% | 9 | swing-up | RAY SINIRI | — | 0.000 | 0.4000 | 0.000 |
| -10% | 9 | denge | BAŞARILI | 0.070 | 11.930 | 0.0475 | 0.000 |
| +10% | 11 | swing-up | BAŞARILI | 8.194 | 3.806 | 0.1846 | 0.000 |
| +10% | 11 | denge | BAŞARILI | 0.076 | 11.924 | 0.0513 | 0.000 |
| +20% | 12 | swing-up | BAŞARISIZ | — | 0.000 | 0.0721 | 0.000 |
| +20% | 12 | denge | BAŞARILI | 0.080 | 11.920 | 0.0538 | 0.000 |
| +30% | 13 | swing-up | BAŞARISIZ | — | 0.000 | 0.0590 | 0.000 |
| +30% | 13 | denge | BAŞARILI | 0.082 | 11.918 | 0.0563 | 0.000 |
### motor.r_pulley — Kasnak yarıçapı


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 0.00889 | swing-up | BAŞARISIZ | — | 0.000 | 0.0440 | 0.000 |
| -30% | 0.00889 | denge | BAŞARILI | 0.086 | 11.914 | 0.0598 | 0.000 |
| -20% | 0.01016 | swing-up | BAŞARISIZ | — | 0.000 | 0.0646 | 0.000 |
| -20% | 0.01016 | denge | BAŞARILI | 0.080 | 11.920 | 0.0550 | 0.000 |
| -10% | 0.01143 | swing-up | BAŞARILI | 7.352 | 4.648 | 0.0903 | 0.000 |
| -10% | 0.01143 | denge | BAŞARILI | 0.076 | 11.924 | 0.0516 | 0.000 |
| +10% | 0.01397 | swing-up | RAY SINIRI | — | 0.000 | 0.4000 | 0.000 |
| +10% | 0.01397 | denge | BAŞARILI | 0.070 | 11.930 | 0.0477 | 0.000 |
| +20% | 0.01524 | swing-up | FAULT | — | 0.000 | 0.4000 | 0.040 |
| +20% | 0.01524 | denge | BAŞARILI | 0.066 | 11.934 | 0.0463 | 0.000 |
| +30% | 0.01651 | swing-up | FAULT | — | 0.000 | 0.4000 | 0.098 |
| +30% | 0.01651 | denge | BAŞARILI | 0.066 | 11.934 | 0.0454 | 0.000 |
### motor.J_rotor — Motor rotor ataleti

Sıfır nominal için mutlak referans: 8.0645e-07 kg·m²; yalnız pozitif ekleme yapılır.

| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|

-30%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.


-20%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.


-10%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.

| +10% | 8.0645e-08 | swing-up | BAŞARILI | 5.328 | 6.672 | 0.2409 | 0.016 |
| +10% | 8.0645e-08 | denge | BAŞARILI | 0.072 | 11.928 | 0.0494 | 0.000 |
| +20% | 1.6129e-07 | swing-up | BAŞARILI | 4.960 | 7.040 | 0.1708 | 0.000 |
| +20% | 1.6129e-07 | denge | BAŞARILI | 0.072 | 11.928 | 0.0496 | 0.000 |
| +30% | 2.41935e-07 | swing-up | BAŞARILI | 4.586 | 7.414 | 0.1601 | 0.000 |
| +30% | 2.41935e-07 | denge | BAŞARILI | 0.074 | 11.926 | 0.0498 | 0.000 |
### motor.T_f — Motor Coulomb sürtünme torku

Sıfır nominal için mutlak referans: 0.00892 N·m; yalnız pozitif ekleme yapılır.

| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|

-30%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.


-20%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.


-10%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.

| +10% | 0.000892 | swing-up | BAŞARILI | 8.838 | 3.162 | 0.1846 | 0.010 |
| +10% | 0.000892 | denge | BAŞARILI | 0.076 | 11.924 | 0.0531 | 0.000 |
| +20% | 0.001784 | swing-up | BAŞARISIZ | — | 0.000 | 0.0013 | 0.000 |
| +20% | 0.001784 | denge | BAŞARILI | 0.080 | 11.920 | 0.0585 | 0.000 |
| +30% | 0.002676 | swing-up | BAŞARISIZ | — | 0.000 | 0.0013 | 0.000 |
| +30% | 0.002676 | denge | BAŞARILI | 0.086 | 11.914 | 0.0667 | 0.000 |
### motor.B_m — Motor viskoz sürtünmesi

Sıfır nominal için mutlak referans: 1.6129e-07 N·m·s/rad; yalnız pozitif ekleme yapılır.

| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|

-30%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.


-20%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.


-10%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.

| +10% | 1.6129e-08 | swing-up | BAŞARILI | 4.242 | 7.758 | 0.1596 | 0.008 |
| +10% | 1.6129e-08 | denge | BAŞARILI | 0.070 | 11.930 | 0.0494 | 0.000 |
| +20% | 3.2258e-08 | swing-up | BAŞARILI | 4.294 | 7.706 | 0.1776 | 0.010 |
| +20% | 3.2258e-08 | denge | BAŞARILI | 0.072 | 11.928 | 0.0494 | 0.000 |
| +30% | 4.8387e-08 | swing-up | BAŞARILI | 4.284 | 7.716 | 0.1748 | 0.010 |
| +30% | 4.8387e-08 | denge | BAŞARILI | 0.070 | 11.930 | 0.0492 | 0.000 |

## Sınırlar ve yeniden çalıştırma

- Sıfır nominal referansları keşif ölçekleridir; ölçülmüş motor verisi değildir.
- I için referans m·l²; rotor için arabaya yansıması M olan atalet; B_m için max(b, 0.1) N·s/m kullanılır.
- T_f için referans, Pololu 37D tahmininin (Kt·I₀, I₀ = 0.2 A) 5 katıdır: +%10/20/30 sütunları tahminin 0.5/1/1.5 katıdır. Kontrolcü T_f'yi bilmez (telafi kapalı, #30); arayüzdeki Sürtünme Ölç ile ölçülüp telafi açılırsa bu sonuç değişir. Sürtünme ölçülmeden gerçek denemeye geçilmemelidir.
- Gerçek I / motor kayıpları ölçüldüğünde params.json güncellenmeli; ardından ±% taraması yeniden çalıştırılmalı.
- Motor parametreleri yalnız voltage modunda değerlendirilir. V_max tek başına kuvvet modelini değiştirmediğinden taranmaz; giriş sınırı u_max sabittir.
- Kontrolcü yeniden tasarlanmaz; fiziksel parametre uyumsuzluğu ölçülür. Ölçüm ölçek hataları, gecikme ve birleşik sapmalar bu deneyin dışında kalır.
- Çalıştır: `python tools/hassasiyet_analizi.py`; makine tarafından okunabilir sonuçlar JSON dosyasına da yazılır.
