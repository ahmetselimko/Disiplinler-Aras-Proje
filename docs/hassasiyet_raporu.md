# Parametre Hassasiyet Analizi Raporu

Issue #13 — Tek seferde bir fiziksel parametre değiştirilir; nominal model ve K sabit kalır.
Gerçek donanım doğrulaması değildir. Parametreler yer tutucu olabilir; sonuçlar yalnız bu modele,
başlangıç durumlarına, sensöre ve deney süresine aittir. Birleşik hatalar ayrıca test edilmelidir.

## Tekrarlanabilir deney ayarları

- Her deney: 12 s; Ts = 0.002 s; sensör: AS5600 12-bit ve 48 CPR motor enkoderi (kuantalama), gürültü yok; hız dik bölgede Kalman filtresiyle, geniş açıda 25 Hz türev filtresiyle kestirilir (firmware gibi).
- Başarı: son en az 1 s boyunca BALANCE ve |θ| ≤ 2°; FAULT ve ray teması yok.
- Ayrı başlangıçlar: aşağıdan 180° swing-up; dik çevresinde 5° denge.
- Doyum: sol örnek sabit kabul edilerek |u| ≥ u_max − 0.001 integralidir.
- Nominal kontroller:
  - swing: BAŞARILI; son denge süresi 7.390 s
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
      "r_pulley": 0.009549,
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
      -9.515837853497176,
      -27.286601334789108,
      -61.49054803650327,
      -9.737243561213205
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
      "E_ratio": 1.0
    },
    "catch_deg": 15.0,
    "release_deg": 30.0,
    "sensor": {
      "th_resolution": 0.0015339807878856412,
      "x_resolution": 0.00012499611770470391,
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
      "motor.J_rotor": 4.5591700500000006e-07,
      "motor.T_f": 0.00892,
      "motor.B_m": 9.118340100000002e-08
    },
    "substeps": 4,
    "vel_fc_hz": 25.0,
    "delay_steps": 0,
    "end_stops": true,
    "use_kalman": true
  }
}
```

## Ölçüm öncelikleri

| Parametre | Sonuç | Ölçüm önerisi |
|---|---|---|
| `l` — Pivot–ağırlık merkezi mesafesi | KRİTİK (Çok Hassas) | Ölçümü ve model varsayımını öncelikle doğrulayın; küçük tarama sapmalarında başarısızlık var. |
| `motor.Kt` — Motor tork sabiti | KRİTİK (Çok Hassas) | Ölçümü ve model varsayımını öncelikle doğrulayın; küçük tarama sapmalarında başarısızlık var. |
| `motor.Ke` — Motor ters EMK sabiti | KRİTİK (Çok Hassas) | Ölçümü ve model varsayımını öncelikle doğrulayın; küçük tarama sapmalarında başarısızlık var. |
| `motor.gear` — Redüktör oranı | KRİTİK (Çok Hassas) | Ölçümü ve model varsayımını öncelikle doğrulayın; küçük tarama sapmalarında başarısızlık var. |
| `motor.r_pulley` — Kasnak yarıçapı | KRİTİK (Çok Hassas) | Ölçümü ve model varsayımını öncelikle doğrulayın; küçük tarama sapmalarında başarısızlık var. |
| `motor.T_f` — Motor Coulomb sürtünme torku | KRİTİK (Çok Hassas) | Ölçümü ve model varsayımını öncelikle doğrulayın; küçük tarama sapmalarında başarısızlık var. Nominal sıfır: bu, yüzde tolerans değil ek atalet/sürtünme deneyidir. |
| `m` — Sarkaç kütlesi | TOLERANSLI (yalnız taranan koşullarda) | Bu koşullarda başarısızlık gözlenmedi; daha geniş tolerans için kanıt değildir. |
| `M` — Araba kütlesi | TOLERANSLI (yalnız taranan koşullarda) | Bu koşullarda başarısızlık gözlenmedi; daha geniş tolerans için kanıt değildir. |
| `I` — Sarkacın ağırlık merkezi etrafındaki ataleti | TOLERANSLI (yalnız taranan koşullarda) | Bu koşullarda başarısızlık gözlenmedi; daha geniş tolerans için kanıt değildir. Nominal sıfır: bu, yüzde tolerans değil ek atalet/sürtünme deneyidir. |
| `b` — Araba viskoz sürtünmesi | TOLERANSLI (yalnız taranan koşullarda) | Bu koşullarda başarısızlık gözlenmedi; daha geniş tolerans için kanıt değildir. |
| `c` — Pivot viskoz sürtünmesi | TOLERANSLI (yalnız taranan koşullarda) | Bu koşullarda başarısızlık gözlenmedi; daha geniş tolerans için kanıt değildir. |
| `motor.R` — Motor direnci | TOLERANSLI (yalnız taranan koşullarda) | Bu koşullarda başarısızlık gözlenmedi; daha geniş tolerans için kanıt değildir. |
| `motor.J_rotor` — Motor rotor ataleti | TOLERANSLI (yalnız taranan koşullarda) | Bu koşullarda başarısızlık gözlenmedi; daha geniş tolerans için kanıt değildir. Nominal sıfır: bu, yüzde tolerans değil ek atalet/sürtünme deneyidir. |
| `motor.B_m` — Motor viskoz sürtünmesi | TOLERANSLI (yalnız taranan koşullarda) | Bu koşullarda başarısızlık gözlenmedi; daha geniş tolerans için kanıt değildir. Nominal sıfır: bu, yüzde tolerans değil ek atalet/sürtünme deneyidir. |

## Parametre sapma tablosu

✓ iki deney başarılı; ⚠ başarılı fakat rayın %75'inden fazla kullanılmış; ✗ başarısız; — uygulanamaz.
Sıfır nominal satırlarda +% değerleri, aşağıda belirtilen mutlak referansın oranıdır.

| Parametre | Nominal | -30% | -20% | -10% | +10% | +20% | +30% |
|---|---|---|---|---|---|---|---|
| `l` | 0.3 m | ✗ | ✗ | ✗ | ✓ | ✗ | ✓ |
| `m` | 0.2 kg | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| `M` | 0.5 kg | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| `I` | 0 kg·m² | — | — | — | ✓ | ✓ | ✓ |
| `b` | 0.1 N·s/m | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| `c` | 0.0005 N·m·s/rad | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| `motor.R` | 2.18 Ω | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| `motor.Kt` | 0.00892 N·m/A | ✓ | ✗ | ✓ | ✓ | ✓ | ✓ |
| `motor.Ke` | 0.01104 V·s/rad | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ |
| `motor.gear` | 10 1 | ✗ | ✗ | ✗ | ✓ | ✗ | ✗ |
| `motor.r_pulley` | 0.009549 m | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ |
| `motor.J_rotor` | 0 kg·m² | — | — | — | ✓ | ✓ | ✓ |
| `motor.T_f` | 0 N·m | — | — | — | ✓ | ✗ | ✗ |
| `motor.B_m` | 0 N·m·s/rad | — | — | — | ✓ | ✓ | ✓ |

## Deney ayrıntıları

İlk dengeleme geçmişteki ilk kesintisiz bant girişidir; başarı kararı son denge penceresine dayanır.

### l — Pivot–ağırlık merkezi mesafesi


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 0.21 | swing-up | BAŞARISIZ | — | 0.000 | 0.1337 | 0.000 |
| -30% | 0.21 | denge | BAŞARILI | 0.084 | 11.916 | 0.0472 | 0.000 |
| -20% | 0.24 | swing-up | BAŞARISIZ | — | 0.000 | 0.1688 | 0.000 |
| -20% | 0.24 | denge | BAŞARILI | 0.078 | 11.922 | 0.0476 | 0.000 |
| -10% | 0.27 | swing-up | BAŞARISIZ | — | 0.000 | 0.1664 | 0.000 |
| -10% | 0.27 | denge | BAŞARILI | 0.074 | 11.926 | 0.0478 | 0.000 |
| +10% | 0.33 | swing-up | BAŞARILI | 4.650 | 7.350 | 0.1642 | 0.000 |
| +10% | 0.33 | denge | BAŞARILI | 0.066 | 11.934 | 0.0487 | 0.000 |
| +20% | 0.36 | swing-up | RAY SINIRI | — | 0.000 | 0.4000 | 0.234 |
| +20% | 0.36 | denge | BAŞARILI | 0.064 | 11.936 | 0.0495 | 0.000 |
| +30% | 0.39 | swing-up | BAŞARILI | 6.186 | 5.814 | 0.1661 | 0.000 |
| +30% | 0.39 | denge | BAŞARILI | 0.062 | 11.938 | 0.0507 | 0.000 |
### m — Sarkaç kütlesi


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 0.14 | swing-up | BAŞARILI | 4.398 | 7.602 | 0.2052 | 0.042 |
| -30% | 0.14 | denge | BAŞARILI | 0.070 | 11.930 | 0.0484 | 0.000 |
| -20% | 0.16 | swing-up | BAŞARILI | 4.734 | 7.266 | 0.1644 | 0.000 |
| -20% | 0.16 | denge | BAŞARILI | 0.070 | 11.930 | 0.0482 | 0.000 |
| -10% | 0.18 | swing-up | BAŞARILI | 4.634 | 7.366 | 0.1608 | 0.000 |
| -10% | 0.18 | denge | BAŞARILI | 0.070 | 11.930 | 0.0484 | 0.000 |
| +10% | 0.22 | swing-up | BAŞARILI | 4.864 | 7.136 | 0.1631 | 0.012 |
| +10% | 0.22 | denge | BAŞARILI | 0.070 | 11.930 | 0.0483 | 0.000 |
| +20% | 0.24 | swing-up | BAŞARILI | 5.190 | 6.810 | 0.1486 | 0.000 |
| +20% | 0.24 | denge | BAŞARILI | 0.070 | 11.930 | 0.0481 | 0.000 |
| +30% | 0.26 | swing-up | BAŞARILI | 5.110 | 6.890 | 0.1465 | 0.000 |
| +30% | 0.26 | denge | BAŞARILI | 0.070 | 11.930 | 0.0481 | 0.000 |
### M — Araba kütlesi


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 0.35 | swing-up | BAŞARILI | 5.286 | 6.714 | 0.2289 | 0.014 |
| -30% | 0.35 | denge | BAŞARILI | 0.070 | 11.930 | 0.0483 | 0.000 |
| -20% | 0.4 | swing-up | BAŞARILI | 4.960 | 7.040 | 0.1854 | 0.002 |
| -20% | 0.4 | denge | BAŞARILI | 0.070 | 11.930 | 0.0481 | 0.000 |
| -10% | 0.45 | swing-up | BAŞARILI | 4.590 | 7.410 | 0.1587 | 0.000 |
| -10% | 0.45 | denge | BAŞARILI | 0.070 | 11.930 | 0.0481 | 0.000 |
| +10% | 0.55 | swing-up | BAŞARILI | 4.528 | 7.472 | 0.1561 | 0.000 |
| +10% | 0.55 | denge | BAŞARILI | 0.070 | 11.930 | 0.0484 | 0.000 |
| +20% | 0.6 | swing-up | BAŞARILI | 5.760 | 6.240 | 0.2155 | 0.010 |
| +20% | 0.6 | denge | BAŞARILI | 0.070 | 11.930 | 0.0482 | 0.000 |
| +30% | 0.65 | swing-up | BAŞARILI | 5.248 | 6.752 | 0.1508 | 0.000 |
| +30% | 0.65 | denge | BAŞARILI | 0.070 | 11.930 | 0.0485 | 0.000 |
### I — Sarkacın ağırlık merkezi etrafındaki ataleti

Sıfır nominal için mutlak referans: 0.018 kg·m²; yalnız pozitif ekleme yapılır.

| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|

-30%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.


-20%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.


-10%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.

| +10% | 0.0018 | swing-up | BAŞARILI | 5.162 | 6.838 | 0.1662 | 0.000 |
| +10% | 0.0018 | denge | BAŞARILI | 0.066 | 11.934 | 0.0487 | 0.000 |
| +20% | 0.0036 | swing-up | BAŞARILI | 5.264 | 6.736 | 0.1678 | 0.000 |
| +20% | 0.0036 | denge | BAŞARILI | 0.064 | 11.936 | 0.0495 | 0.000 |
| +30% | 0.0054 | swing-up | BAŞARILI | 5.308 | 6.692 | 0.1791 | 0.000 |
| +30% | 0.0054 | denge | BAŞARILI | 0.062 | 11.938 | 0.0510 | 0.000 |
### b — Araba viskoz sürtünmesi


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 0.07 | swing-up | BAŞARILI | 4.600 | 7.400 | 0.1593 | 0.000 |
| -30% | 0.07 | denge | BAŞARILI | 0.070 | 11.930 | 0.0482 | 0.000 |
| -20% | 0.08 | swing-up | BAŞARILI | 4.602 | 7.398 | 0.1587 | 0.000 |
| -20% | 0.08 | denge | BAŞARILI | 0.070 | 11.930 | 0.0482 | 0.000 |
| -10% | 0.09 | swing-up | BAŞARILI | 4.614 | 7.386 | 0.1562 | 0.000 |
| -10% | 0.09 | denge | BAŞARILI | 0.070 | 11.930 | 0.0481 | 0.000 |
| +10% | 0.11 | swing-up | BAŞARILI | 4.604 | 7.396 | 0.1569 | 0.000 |
| +10% | 0.11 | denge | BAŞARILI | 0.070 | 11.930 | 0.0483 | 0.000 |
| +20% | 0.12 | swing-up | BAŞARILI | 4.614 | 7.386 | 0.1576 | 0.000 |
| +20% | 0.12 | denge | BAŞARILI | 0.070 | 11.930 | 0.0482 | 0.000 |
| +30% | 0.13 | swing-up | BAŞARILI | 4.626 | 7.374 | 0.1570 | 0.000 |
| +30% | 0.13 | denge | BAŞARILI | 0.070 | 11.930 | 0.0483 | 0.000 |
### c — Pivot viskoz sürtünmesi


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 0.00035 | swing-up | BAŞARILI | 4.586 | 7.414 | 0.1558 | 0.000 |
| -30% | 0.00035 | denge | BAŞARILI | 0.070 | 11.930 | 0.0482 | 0.000 |
| -20% | 0.0004 | swing-up | BAŞARILI | 4.608 | 7.392 | 0.1559 | 0.000 |
| -20% | 0.0004 | denge | BAŞARILI | 0.070 | 11.930 | 0.0481 | 0.000 |
| -10% | 0.00045 | swing-up | BAŞARILI | 4.596 | 7.404 | 0.1566 | 0.000 |
| -10% | 0.00045 | denge | BAŞARILI | 0.070 | 11.930 | 0.0481 | 0.000 |
| +10% | 0.00055 | swing-up | BAŞARILI | 4.630 | 7.370 | 0.1566 | 0.000 |
| +10% | 0.00055 | denge | BAŞARILI | 0.070 | 11.930 | 0.0483 | 0.000 |
| +20% | 0.0006 | swing-up | BAŞARILI | 4.612 | 7.388 | 0.1565 | 0.000 |
| +20% | 0.0006 | denge | BAŞARILI | 0.070 | 11.930 | 0.0482 | 0.000 |
| +30% | 0.00065 | swing-up | BAŞARILI | 4.624 | 7.376 | 0.1569 | 0.000 |
| +30% | 0.00065 | denge | BAŞARILI | 0.070 | 11.930 | 0.0483 | 0.000 |
### motor.R — Motor direnci


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 1.526 | swing-up | BAŞARILI | 4.164 | 7.836 | 0.1733 | 0.000 |
| -30% | 1.526 | denge | BAŞARILI | 0.070 | 11.930 | 0.0481 | 0.000 |
| -20% | 1.744 | swing-up | BAŞARILI | 4.204 | 7.796 | 0.1684 | 0.000 |
| -20% | 1.744 | denge | BAŞARILI | 0.070 | 11.930 | 0.0483 | 0.000 |
| -10% | 1.962 | swing-up | BAŞARILI | 5.190 | 6.810 | 0.2554 | 0.012 |
| -10% | 1.962 | denge | BAŞARILI | 0.070 | 11.930 | 0.0484 | 0.000 |
| +10% | 2.398 | swing-up | BAŞARILI | 5.238 | 6.762 | 0.1605 | 0.000 |
| +10% | 2.398 | denge | BAŞARILI | 0.070 | 11.930 | 0.0482 | 0.000 |
| +20% | 2.616 | swing-up | BAŞARILI | 5.154 | 6.846 | 0.1450 | 0.000 |
| +20% | 2.616 | denge | BAŞARILI | 0.070 | 11.930 | 0.0482 | 0.000 |
| +30% | 2.834 | swing-up | BAŞARILI | 6.176 | 5.824 | 0.1562 | 0.002 |
| +30% | 2.834 | denge | BAŞARILI | 0.068 | 11.932 | 0.0481 | 0.000 |
### motor.Kt — Motor tork sabiti


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 0.006244 | swing-up | BAŞARILI | 7.018 | 4.982 | 0.2103 | 0.040 |
| -30% | 0.006244 | denge | BAŞARILI | 0.068 | 11.932 | 0.0483 | 0.000 |
| -20% | 0.007136 | swing-up | BAŞARISIZ | — | 0.000 | 0.1433 | 0.000 |
| -20% | 0.007136 | denge | BAŞARILI | 0.070 | 11.930 | 0.0482 | 0.000 |
| -10% | 0.008028 | swing-up | BAŞARILI | 5.218 | 6.782 | 0.1487 | 0.000 |
| -10% | 0.008028 | denge | BAŞARILI | 0.070 | 11.930 | 0.0484 | 0.000 |
| +10% | 0.009812 | swing-up | BAŞARILI | 4.974 | 7.026 | 0.1864 | 0.002 |
| +10% | 0.009812 | denge | BAŞARILI | 0.070 | 11.930 | 0.0482 | 0.000 |
| +20% | 0.010704 | swing-up | BAŞARILI | 4.080 | 7.920 | 0.1658 | 0.000 |
| +20% | 0.010704 | denge | BAŞARILI | 0.070 | 11.930 | 0.0482 | 0.000 |
| +30% | 0.011596 | swing-up | BAŞARILI | 4.244 | 7.756 | 0.1706 | 0.000 |
| +30% | 0.011596 | denge | BAŞARILI | 0.070 | 11.930 | 0.0482 | 0.000 |
### motor.Ke — Motor ters EMK sabiti


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 0.007728 | swing-up | FAULT | — | 0.000 | 0.4000 | 0.216 |
| -30% | 0.007728 | denge | BAŞARILI | 0.060 | 11.940 | 0.0453 | 0.000 |
| -20% | 0.008832 | swing-up | RAY SINIRI | — | 0.000 | 0.4000 | 0.224 |
| -20% | 0.008832 | denge | BAŞARILI | 0.064 | 11.936 | 0.0461 | 0.000 |
| -10% | 0.009936 | swing-up | RAY SINIRI | — | 0.000 | 0.4000 | 0.164 |
| -10% | 0.009936 | denge | BAŞARILI | 0.068 | 11.932 | 0.0473 | 0.000 |
| +10% | 0.012144 | swing-up | BAŞARISIZ | — | 0.212 | 0.1180 | 0.000 |
| +10% | 0.012144 | denge | BAŞARILI | 0.072 | 11.928 | 0.0495 | 0.000 |
| +20% | 0.013248 | swing-up | BAŞARISIZ | — | 0.000 | 0.0465 | 0.000 |
| +20% | 0.013248 | denge | BAŞARILI | 0.074 | 11.926 | 0.0502 | 0.000 |
| +30% | 0.014352 | swing-up | BAŞARISIZ | — | 0.000 | 0.0323 | 0.000 |
| +30% | 0.014352 | denge | BAŞARILI | 0.078 | 11.922 | 0.0514 | 0.000 |
### motor.gear — Redüktör oranı


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 7 | swing-up | FAULT | — | 0.000 | 0.4000 | 0.210 |
| -30% | 7 | denge | BAŞARILI | 0.060 | 11.940 | 0.0453 | 0.000 |
| -20% | 8 | swing-up | FAULT | — | 0.000 | 0.4000 | 0.214 |
| -20% | 8 | denge | BAŞARILI | 0.062 | 11.938 | 0.0462 | 0.000 |
| -10% | 9 | swing-up | RAY SINIRI | — | 0.000 | 0.4000 | 0.148 |
| -10% | 9 | denge | BAŞARILI | 0.068 | 11.932 | 0.0473 | 0.000 |
| +10% | 11 | swing-up | BAŞARILI | 10.542 | 1.458 | 0.1057 | 0.000 |
| +10% | 11 | denge | BAŞARILI | 0.072 | 11.928 | 0.0495 | 0.000 |
| +20% | 12 | swing-up | BAŞARISIZ | — | 0.000 | 0.0503 | 0.000 |
| +20% | 12 | denge | BAŞARILI | 0.076 | 11.924 | 0.0503 | 0.000 |
| +30% | 13 | swing-up | BAŞARISIZ | — | 0.000 | 0.0341 | 0.000 |
| +30% | 13 | denge | BAŞARILI | 0.078 | 11.922 | 0.0512 | 0.000 |
### motor.r_pulley — Kasnak yarıçapı


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 0.0066843 | swing-up | BAŞARISIZ | — | 0.000 | 0.0250 | 0.000 |
| -30% | 0.0066843 | denge | BAŞARILI | 0.082 | 11.918 | 0.0526 | 0.000 |
| -20% | 0.0076392 | swing-up | BAŞARISIZ | — | 0.000 | 0.0405 | 0.000 |
| -20% | 0.0076392 | denge | BAŞARILI | 0.076 | 11.924 | 0.0507 | 0.000 |
| -10% | 0.0085941 | swing-up | BAŞARISIZ | — | 0.184 | 0.1294 | 0.000 |
| -10% | 0.0085941 | denge | BAŞARILI | 0.074 | 11.926 | 0.0495 | 0.000 |
| +10% | 0.0105039 | swing-up | RAY SINIRI | — | 0.000 | 0.4000 | 0.126 |
| +10% | 0.0105039 | denge | BAŞARILI | 0.068 | 11.932 | 0.0474 | 0.000 |
| +20% | 0.0114588 | swing-up | RAY SINIRI | — | 0.000 | 0.4000 | 0.210 |
| +20% | 0.0114588 | denge | BAŞARILI | 0.064 | 11.936 | 0.0465 | 0.000 |
| +30% | 0.0124137 | swing-up | RAY SINIRI | — | 0.000 | 0.4000 | 0.214 |
| +30% | 0.0124137 | denge | BAŞARILI | 0.062 | 11.938 | 0.0459 | 0.000 |
### motor.J_rotor — Motor rotor ataleti

Sıfır nominal için mutlak referans: 4.5591701e-07 kg·m²; yalnız pozitif ekleme yapılır.

| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|

-30%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.


-20%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.


-10%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.

| +10% | 4.5591701e-08 | swing-up | BAŞARILI | 4.528 | 7.472 | 0.1561 | 0.000 |
| +10% | 4.5591701e-08 | denge | BAŞARILI | 0.070 | 11.930 | 0.0484 | 0.000 |
| +20% | 9.1183401e-08 | swing-up | BAŞARILI | 5.760 | 6.240 | 0.2155 | 0.010 |
| +20% | 9.1183401e-08 | denge | BAŞARILI | 0.070 | 11.930 | 0.0482 | 0.000 |
| +30% | 1.367751e-07 | swing-up | BAŞARILI | 5.248 | 6.752 | 0.1508 | 0.000 |
| +30% | 1.367751e-07 | denge | BAŞARILI | 0.070 | 11.930 | 0.0485 | 0.000 |
### motor.T_f — Motor Coulomb sürtünme torku

Sıfır nominal için mutlak referans: 0.00892 N·m; yalnız pozitif ekleme yapılır.

| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|

-30%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.


-20%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.


-10%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.

| +10% | 0.000892 | swing-up | BAŞARILI | 8.756 | 3.244 | 0.1124 | 0.000 |
| +10% | 0.000892 | denge | BAŞARILI | 0.072 | 11.928 | 0.0496 | 0.000 |
| +20% | 0.001784 | swing-up | BAŞARISIZ | — | 0.000 | 0.0013 | 0.000 |
| +20% | 0.001784 | denge | BAŞARILI | 0.074 | 11.926 | 0.0512 | 0.000 |
| +30% | 0.002676 | swing-up | BAŞARISIZ | — | 0.000 | 0.0012 | 0.000 |
| +30% | 0.002676 | denge | BAŞARILI | 0.076 | 11.924 | 0.0527 | 0.000 |
### motor.B_m — Motor viskoz sürtünmesi

Sıfır nominal için mutlak referans: 9.1183401e-08 N·m·s/rad; yalnız pozitif ekleme yapılır.

| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|

-30%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.


-20%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.


-10%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.

| +10% | 9.1183401e-09 | swing-up | BAŞARILI | 4.604 | 7.396 | 0.1569 | 0.000 |
| +10% | 9.1183401e-09 | denge | BAŞARILI | 0.070 | 11.930 | 0.0483 | 0.000 |
| +20% | 1.823668e-08 | swing-up | BAŞARILI | 4.614 | 7.386 | 0.1576 | 0.000 |
| +20% | 1.823668e-08 | denge | BAŞARILI | 0.070 | 11.930 | 0.0482 | 0.000 |
| +30% | 2.735502e-08 | swing-up | BAŞARILI | 4.626 | 7.374 | 0.1570 | 0.000 |
| +30% | 2.735502e-08 | denge | BAŞARILI | 0.070 | 11.930 | 0.0483 | 0.000 |

## Sınırlar ve yeniden çalıştırma

- Sıfır nominal referansları keşif ölçekleridir; ölçülmüş motor verisi değildir.
- I için referans m·l²; rotor için arabaya yansıması M olan atalet; B_m için max(b, 0.1) N·s/m kullanılır.
- T_f için referans, Pololu 37D tahmininin (Kt·I₀, I₀ = 0.2 A) 5 katıdır: +%10/20/30 sütunları tahminin 0.5/1/1.5 katıdır. Kontrolcü T_f'yi bilmez (telafi kapalı, #30); arayüzdeki Sürtünme Ölç ile ölçülüp telafi açılırsa bu sonuç değişir. Sürtünme ölçülmeden gerçek denemeye geçilmemelidir.
- Gerçek I / motor kayıpları ölçüldüğünde params.json güncellenmeli; ardından ±% taraması yeniden çalıştırılmalı.
- Motor parametreleri yalnız voltage modunda değerlendirilir. V_max tek başına kuvvet modelini değiştirmediğinden taranmaz; giriş sınırı u_max sabittir.
- Kontrolcü yeniden tasarlanmaz; fiziksel parametre uyumsuzluğu ölçülür. Ölçüm ölçek hataları, gecikme ve birleşik sapmalar bu deneyin dışında kalır.
- Çalıştır: `python tools/hassasiyet_analizi.py`; makine tarafından okunabilir sonuçlar JSON dosyasına da yazılır.
