# Parametre Hassasiyet Analizi Raporu

Issue #13 — Tek seferde bir fiziksel parametre değiştirilir; nominal model ve K sabit kalır.
Gerçek donanım doğrulaması değildir. Parametreler yer tutucu olabilir; sonuçlar yalnız bu modele,
başlangıç durumlarına, sensöre ve deney süresine aittir. Birleşik hatalar ayrıca test edilmelidir.

## Tekrarlanabilir deney ayarları

- Her deney: 12 s; Ts = 0.002 s; sensör: AS5600 12-bit ve 44 CPR motor enkoderi (kuantalama), gürültü yok; hız dik bölgede Kalman filtresiyle, geniş açıda 25 Hz türev filtresiyle kestirilir (firmware gibi).
- Başarı: son en az 1 s boyunca BALANCE ve |θ| ≤ 2°; FAULT ve ray teması yok.
- Ayrı başlangıçlar: aşağıdan 180° swing-up; dik çevresinde 5° denge.
- Doyum: sol örnek sabit kabul edilerek |u| ≥ u_max − 0.001 integralidir.
- Nominal kontroller:
  - swing: BAŞARILI; son denge süresi 7.444 s
  - balance: BAŞARILI; son denge süresi 11.886 s

Tüm fiziksel parametreler, K, Q/R ve kontrol ayarları:

```json
{
  "nominal": {
    "M": 1.28,
    "m": 0.105,
    "l": 0.0992,
    "I": 0.001203,
    "b": 0.1,
    "c": 0.0005,
    "g": 9.81,
    "rail_half": 0.4,
    "u_max": 12.0,
    "input_mode": "voltage",
    "motor": {
      "R": 9.478,
      "Kt": 0.0107654,
      "Ke": 0.0107654,
      "gear": 19.0,
      "r_pulley": 0.009549,
      "V_max": 12.0,
      "J_rotor": 3.4e-07,
      "T_f": 0.0015077,
      "B_m": 0.0
    },
    "note": "Mekatronik raporu 23.11, tasarım değeri, ölçülmedi — +20 g uç kütlesi (varsayılan); motor Tablo 16'dan, datasheet bekleniyor",
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
      -9.828690871450066,
      -46.64164119838098,
      101.46760012963092,
      14.359285523523514
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
      "E_ratio": 1.05
    },
    "catch_deg": 15.0,
    "release_deg": 30.0,
    "sensor": {
      "th_resolution": 0.0015339807878856412,
      "x_resolution": 7.176810585916014e-05,
      "noise_std": 0.0,
      "seed": 0
    },
    "initial_angles_deg": {
      "swing": 180.0,
      "balance": 5.0
    },
    "zero_scales": {
      "I": 0.0010332671999999999,
      "b": 0.1,
      "c": 0.0005,
      "motor.J_rotor": 3.233095658725762e-07,
      "motor.T_f": 0.010765400000000001,
      "motor.B_m": 2.5258559833795017e-08
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
| `m` — Sarkaç kütlesi | KRİTİK (Çok Hassas) | Ölçümü ve model varsayımını öncelikle doğrulayın; küçük tarama sapmalarında başarısızlık var. |
| `I` — Sarkacın ağırlık merkezi etrafındaki ataleti | KRİTİK (Çok Hassas) | Ölçümü ve model varsayımını öncelikle doğrulayın; küçük tarama sapmalarında başarısızlık var. |
| `motor.Ke` — Motor ters EMK sabiti | ORTA HASSASİYET | Ölçümü doğrulayın; taramada başarısızlık veya ray sınırına yaklaşma var. |
| `motor.gear` — Redüktör oranı | ORTA HASSASİYET | Ölçümü doğrulayın; taramada başarısızlık veya ray sınırına yaklaşma var. |
| `motor.r_pulley` — Kasnak yarıçapı | ORTA HASSASİYET | Ölçümü doğrulayın; taramada başarısızlık veya ray sınırına yaklaşma var. |
| `l` — Pivot–ağırlık merkezi mesafesi | TOLERANSLI (yalnız taranan koşullarda) | Bu koşullarda başarısızlık gözlenmedi; daha geniş tolerans için kanıt değildir. |
| `M` — Araba kütlesi | TOLERANSLI (yalnız taranan koşullarda) | Bu koşullarda başarısızlık gözlenmedi; daha geniş tolerans için kanıt değildir. |
| `b` — Araba viskoz sürtünmesi | TOLERANSLI (yalnız taranan koşullarda) | Bu koşullarda başarısızlık gözlenmedi; daha geniş tolerans için kanıt değildir. |
| `c` — Pivot viskoz sürtünmesi | TOLERANSLI (yalnız taranan koşullarda) | Bu koşullarda başarısızlık gözlenmedi; daha geniş tolerans için kanıt değildir. |
| `motor.R` — Motor direnci | TOLERANSLI (yalnız taranan koşullarda) | Bu koşullarda başarısızlık gözlenmedi; daha geniş tolerans için kanıt değildir. |
| `motor.Kt` — Motor tork sabiti | TOLERANSLI (yalnız taranan koşullarda) | Bu koşullarda başarısızlık gözlenmedi; daha geniş tolerans için kanıt değildir. |
| `motor.J_rotor` — Motor rotor ataleti | TOLERANSLI (yalnız taranan koşullarda) | Bu koşullarda başarısızlık gözlenmedi; daha geniş tolerans için kanıt değildir. |
| `motor.T_f` — Motor Coulomb sürtünme torku | TOLERANSLI (yalnız taranan koşullarda) | Bu koşullarda başarısızlık gözlenmedi; daha geniş tolerans için kanıt değildir. |
| `motor.B_m` — Motor viskoz sürtünmesi | TOLERANSLI (yalnız taranan koşullarda) | Bu koşullarda başarısızlık gözlenmedi; daha geniş tolerans için kanıt değildir. Nominal sıfır: bu, yüzde tolerans değil ek atalet/sürtünme deneyidir. |

## Parametre sapma tablosu

✓ iki deney başarılı; ⚠ başarılı fakat rayın %75'inden fazla kullanılmış; ✗ başarısız; — uygulanamaz.
Sıfır nominal satırlarda +% değerleri, aşağıda belirtilen mutlak referansın oranıdır.

| Parametre | Nominal | -30% | -20% | -10% | +10% | +20% | +30% |
|---|---|---|---|---|---|---|---|
| `l` | 0.0992 m | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| `m` | 0.105 kg | ✓ | ✗ | ✓ | ✓ | ✗ | ✗ |
| `M` | 1.28 kg | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| `I` | 0.001203 kg·m² | ✗ | ✗ | ✓ | ⚠ | ✓ | ✗ |
| `b` | 0.1 N·s/m | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| `c` | 0.0005 N·m·s/rad | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| `motor.R` | 9.478 Ω | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| `motor.Kt` | 0.0107654 N·m/A | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| `motor.Ke` | 0.0107654 V·s/rad | ✗ | ✓ | ✓ | ✓ | ✓ | ✗ |
| `motor.gear` | 19 1 | ✗ | ✓ | ✓ | ✓ | ✓ | ✗ |
| `motor.r_pulley` | 0.009549 m | ✗ | ✓ | ✓ | ✓ | ✓ | ✓ |
| `motor.J_rotor` | 3.4e-07 kg·m² | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| `motor.T_f` | 0.0015077 N·m | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| `motor.B_m` | 0 N·m·s/rad | — | — | — | ✓ | ✓ | ✓ |

## Deney ayrıntıları

İlk dengeleme geçmişteki ilk kesintisiz bant girişidir; başarı kararı son denge penceresine dayanır.

### l — Pivot–ağırlık merkezi mesafesi


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 0.06944 | swing-up | BAŞARILI | 7.098 | 4.902 | 0.2929 | 1.806 |
| -30% | 0.06944 | denge | BAŞARILI | 0.112 | 11.888 | 0.1397 | 0.000 |
| -20% | 0.07936 | swing-up | BAŞARILI | 5.468 | 6.532 | 0.1415 | 1.044 |
| -20% | 0.07936 | denge | BAŞARILI | 0.112 | 11.888 | 0.1424 | 0.000 |
| -10% | 0.08928 | swing-up | BAŞARILI | 4.696 | 7.304 | 0.1402 | 0.852 |
| -10% | 0.08928 | denge | BAŞARILI | 0.112 | 11.888 | 0.1422 | 0.000 |
| +10% | 0.10912 | swing-up | BAŞARILI | 4.732 | 7.268 | 0.1395 | 0.790 |
| +10% | 0.10912 | denge | BAŞARILI | 0.114 | 11.886 | 0.1423 | 0.000 |
| +20% | 0.11904 | swing-up | BAŞARILI | 3.790 | 8.210 | 0.1399 | 0.778 |
| +20% | 0.11904 | denge | BAŞARILI | 0.114 | 11.886 | 0.1408 | 0.000 |
| +30% | 0.12896 | swing-up | BAŞARILI | 3.884 | 8.116 | 0.1399 | 0.778 |
| +30% | 0.12896 | denge | BAŞARILI | 0.112 | 11.888 | 0.1415 | 0.000 |
### m — Sarkaç kütlesi


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 0.0735 | swing-up | BAŞARILI | 7.770 | 4.230 | 0.2148 | 2.374 |
| -30% | 0.0735 | denge | BAŞARILI | 0.112 | 11.888 | 0.1437 | 0.000 |
| -20% | 0.084 | swing-up | RAY SINIRI | — | 0.000 | 0.4000 | 2.094 |
| -20% | 0.084 | denge | BAŞARILI | 0.112 | 11.888 | 0.1427 | 0.000 |
| -10% | 0.0945 | swing-up | BAŞARILI | 4.524 | 7.476 | 0.1427 | 1.028 |
| -10% | 0.0945 | denge | BAŞARILI | 0.112 | 11.888 | 0.1418 | 0.000 |
| +10% | 0.1155 | swing-up | BAŞARILI | 3.738 | 8.262 | 0.1400 | 0.660 |
| +10% | 0.1155 | denge | BAŞARILI | 0.114 | 11.886 | 0.1415 | 0.000 |
| +20% | 0.126 | swing-up | BAŞARISIZ | — | 0.000 | 0.1334 | 0.606 |
| +20% | 0.126 | denge | BAŞARILI | 0.114 | 11.886 | 0.1405 | 0.000 |
| +30% | 0.1365 | swing-up | BAŞARISIZ | — | 0.000 | 0.1318 | 0.608 |
| +30% | 0.1365 | denge | BAŞARILI | 0.114 | 11.886 | 0.1421 | 0.000 |
### M — Araba kütlesi


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 0.896 | swing-up | BAŞARILI | 4.410 | 7.590 | 0.1472 | 1.064 |
| -30% | 0.896 | denge | BAŞARILI | 0.108 | 11.892 | 0.1402 | 0.000 |
| -20% | 1.024 | swing-up | BAŞARILI | 4.540 | 7.460 | 0.1444 | 0.912 |
| -20% | 1.024 | denge | BAŞARILI | 0.110 | 11.890 | 0.1408 | 0.000 |
| -10% | 1.152 | swing-up | BAŞARILI | 4.530 | 7.470 | 0.1418 | 0.858 |
| -10% | 1.152 | denge | BAŞARILI | 0.112 | 11.888 | 0.1418 | 0.000 |
| +10% | 1.408 | swing-up | BAŞARILI | 4.604 | 7.396 | 0.1418 | 0.772 |
| +10% | 1.408 | denge | BAŞARILI | 0.114 | 11.886 | 0.1415 | 0.000 |
| +20% | 1.536 | swing-up | BAŞARILI | 4.642 | 7.358 | 0.1415 | 0.726 |
| +20% | 1.536 | denge | BAŞARILI | 0.116 | 11.884 | 0.1412 | 0.000 |
| +30% | 1.664 | swing-up | BAŞARILI | 4.640 | 7.360 | 0.1425 | 0.696 |
| +30% | 1.664 | denge | BAŞARILI | 0.118 | 11.882 | 0.1416 | 0.000 |
### I — Sarkacın ağırlık merkezi etrafındaki ataleti


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 0.0008421 | swing-up | BAŞARISIZ | — | 0.000 | 0.1289 | 0.550 |
| -30% | 0.0008421 | denge | BAŞARILI | 0.116 | 11.884 | 0.1413 | 0.000 |
| -20% | 0.0009624 | swing-up | BAŞARISIZ | — | 0.000 | 0.1327 | 0.620 |
| -20% | 0.0009624 | denge | BAŞARILI | 0.114 | 11.886 | 0.1397 | 0.000 |
| -10% | 0.0010827 | swing-up | BAŞARILI | 4.048 | 7.952 | 0.1532 | 0.766 |
| -10% | 0.0010827 | denge | BAŞARILI | 0.114 | 11.886 | 0.1400 | 0.000 |
| +10% | 0.0013233 | swing-up | SINIRDA | 5.960 | 6.040 | 0.3324 | 1.490 |
| +10% | 0.0013233 | denge | BAŞARILI | 0.112 | 11.888 | 0.1422 | 0.000 |
| +20% | 0.0014436 | swing-up | BAŞARILI | 4.592 | 7.408 | 0.1449 | 1.086 |
| +20% | 0.0014436 | denge | BAŞARILI | 0.112 | 11.888 | 0.1423 | 0.000 |
| +30% | 0.0015639 | swing-up | RAY SINIRI | — | 0.000 | 0.4000 | 1.816 |
| +30% | 0.0015639 | denge | BAŞARILI | 0.112 | 11.888 | 0.1421 | 0.000 |
### b — Araba viskoz sürtünmesi


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 0.07 | swing-up | BAŞARILI | 4.560 | 7.440 | 0.1419 | 0.808 |
| -30% | 0.07 | denge | BAŞARILI | 0.114 | 11.886 | 0.1391 | 0.000 |
| -20% | 0.08 | swing-up | BAŞARILI | 4.554 | 7.446 | 0.1414 | 0.810 |
| -20% | 0.08 | denge | BAŞARILI | 0.114 | 11.886 | 0.1419 | 0.000 |
| -10% | 0.09 | swing-up | BAŞARILI | 4.554 | 7.446 | 0.1421 | 0.806 |
| -10% | 0.09 | denge | BAŞARILI | 0.114 | 11.886 | 0.1401 | 0.000 |
| +10% | 0.11 | swing-up | BAŞARILI | 4.550 | 7.450 | 0.1396 | 0.806 |
| +10% | 0.11 | denge | BAŞARILI | 0.114 | 11.886 | 0.1423 | 0.000 |
| +20% | 0.12 | swing-up | BAŞARILI | 4.542 | 7.458 | 0.1422 | 0.806 |
| +20% | 0.12 | denge | BAŞARILI | 0.114 | 11.886 | 0.1409 | 0.000 |
| +30% | 0.13 | swing-up | BAŞARILI | 4.552 | 7.448 | 0.1396 | 0.814 |
| +30% | 0.13 | denge | BAŞARILI | 0.114 | 11.886 | 0.1399 | 0.000 |
### c — Pivot viskoz sürtünmesi


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 0.00035 | swing-up | BAŞARILI | 4.056 | 7.944 | 0.1388 | 0.680 |
| -30% | 0.00035 | denge | BAŞARILI | 0.112 | 11.888 | 0.1426 | 0.000 |
| -20% | 0.0004 | swing-up | BAŞARILI | 3.984 | 8.016 | 0.1399 | 0.718 |
| -20% | 0.0004 | denge | BAŞARILI | 0.114 | 11.886 | 0.1414 | 0.000 |
| -10% | 0.00045 | swing-up | BAŞARILI | 4.348 | 7.652 | 0.2114 | 1.014 |
| -10% | 0.00045 | denge | BAŞARILI | 0.112 | 11.888 | 0.1426 | 0.000 |
| +10% | 0.00055 | swing-up | BAŞARILI | 4.530 | 7.470 | 0.1421 | 0.834 |
| +10% | 0.00055 | denge | BAŞARILI | 0.114 | 11.886 | 0.1422 | 0.000 |
| +20% | 0.0006 | swing-up | BAŞARILI | 4.466 | 7.534 | 0.1420 | 0.936 |
| +20% | 0.0006 | denge | BAŞARILI | 0.112 | 11.888 | 0.1425 | 0.000 |
| +30% | 0.00065 | swing-up | BAŞARILI | 5.164 | 6.836 | 0.1396 | 0.994 |
| +30% | 0.00065 | denge | BAŞARILI | 0.112 | 11.888 | 0.1413 | 0.000 |
### motor.R — Motor direnci


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 6.6346 | swing-up | BAŞARILI | 4.564 | 7.436 | 0.1640 | 1.398 |
| -30% | 6.6346 | denge | BAŞARILI | 0.096 | 11.904 | 0.0943 | 0.000 |
| -20% | 7.5824 | swing-up | BAŞARILI | 4.822 | 7.178 | 0.2222 | 1.618 |
| -20% | 7.5824 | denge | BAŞARILI | 0.102 | 11.898 | 0.1104 | 0.000 |
| -10% | 8.5302 | swing-up | BAŞARILI | 4.844 | 7.156 | 0.2109 | 1.226 |
| -10% | 8.5302 | denge | BAŞARILI | 0.108 | 11.892 | 0.1267 | 0.000 |
| +10% | 10.4258 | swing-up | BAŞARILI | 4.560 | 7.440 | 0.1578 | 0.766 |
| +10% | 10.4258 | denge | BAŞARILI | 0.118 | 11.882 | 0.1580 | 0.000 |
| +20% | 11.3736 | swing-up | BAŞARILI | 4.698 | 7.302 | 0.1682 | 0.846 |
| +20% | 11.3736 | denge | BAŞARILI | 0.124 | 11.876 | 0.1734 | 0.000 |
| +30% | 12.3214 | swing-up | BAŞARILI | 5.100 | 6.900 | 0.1845 | 0.620 |
| +30% | 12.3214 | denge | BAŞARILI | 0.132 | 11.868 | 0.1879 | 0.000 |
### motor.Kt — Motor tork sabiti


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 0.00753578 | swing-up | BAŞARILI | 5.788 | 6.212 | 0.2090 | 0.496 |
| -30% | 0.00753578 | denge | BAŞARILI | 0.140 | 11.860 | 0.2061 | 0.000 |
| -20% | 0.00861232 | swing-up | BAŞARILI | 5.230 | 6.770 | 0.1818 | 0.698 |
| -20% | 0.00861232 | denge | BAŞARILI | 0.128 | 11.872 | 0.1814 | 0.000 |
| -10% | 0.00968886 | swing-up | BAŞARILI | 4.562 | 7.438 | 0.1590 | 0.764 |
| -10% | 0.00968886 | denge | BAŞARILI | 0.120 | 11.880 | 0.1600 | 0.000 |
| +10% | 0.01184194 | swing-up | BAŞARILI | 4.736 | 7.264 | 0.1493 | 0.872 |
| +10% | 0.01184194 | denge | BAŞARILI | 0.108 | 11.892 | 0.1273 | 0.000 |
| +20% | 0.01291848 | swing-up | BAŞARILI | 3.744 | 8.256 | 0.1591 | 1.102 |
| +20% | 0.01291848 | denge | BAŞARILI | 0.104 | 11.896 | 0.1154 | 0.000 |
| +30% | 0.01399502 | swing-up | BAŞARILI | 4.656 | 7.344 | 0.1610 | 1.240 |
| +30% | 0.01399502 | denge | BAŞARILI | 0.100 | 11.900 | 0.1017 | 0.000 |
### motor.Ke — Motor ters EMK sabiti


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 0.00753578 | swing-up | RAY SINIRI | — | 0.000 | 0.4000 | 0.200 |
| -30% | 0.00753578 | denge | BAŞARILI | 0.098 | 11.902 | 0.1253 | 0.000 |
| -20% | 0.00861232 | swing-up | BAŞARILI | 3.572 | 8.428 | 0.2232 | 0.530 |
| -20% | 0.00861232 | denge | BAŞARILI | 0.102 | 11.898 | 0.1303 | 0.000 |
| -10% | 0.00968886 | swing-up | BAŞARILI | 3.670 | 8.330 | 0.1749 | 0.756 |
| -10% | 0.00968886 | denge | BAŞARILI | 0.108 | 11.892 | 0.1350 | 0.000 |
| +10% | 0.01184194 | swing-up | BAŞARILI | 5.476 | 6.524 | 0.1455 | 1.128 |
| +10% | 0.01184194 | denge | BAŞARILI | 0.118 | 11.882 | 0.1462 | 0.000 |
| +20% | 0.01291848 | swing-up | BAŞARILI | 7.690 | 4.310 | 0.1472 | 0.788 |
| +20% | 0.01291848 | denge | BAŞARILI | 0.124 | 11.876 | 0.1494 | 0.000 |
| +30% | 0.01399502 | swing-up | BAŞARISIZ | — | 0.000 | 0.0770 | 0.000 |
| +30% | 0.01399502 | denge | BAŞARILI | 0.132 | 11.868 | 0.1505 | 0.000 |
### motor.gear — Redüktör oranı


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 13.3 | swing-up | RAY SINIRI | — | 0.000 | 0.4000 | 0.198 |
| -30% | 13.3 | denge | BAŞARILI | 0.102 | 11.898 | 0.1256 | 0.000 |
| -20% | 15.2 | swing-up | BAŞARILI | 3.586 | 8.414 | 0.2237 | 0.512 |
| -20% | 15.2 | denge | BAŞARILI | 0.104 | 11.896 | 0.1301 | 0.000 |
| -10% | 17.1 | swing-up | BAŞARILI | 3.612 | 8.388 | 0.1732 | 0.718 |
| -10% | 17.1 | denge | BAŞARILI | 0.108 | 11.892 | 0.1353 | 0.000 |
| +10% | 20.9 | swing-up | BAŞARILI | 5.514 | 6.486 | 0.1457 | 1.118 |
| +10% | 20.9 | denge | BAŞARILI | 0.118 | 11.882 | 0.1467 | 0.000 |
| +20% | 22.8 | swing-up | BAŞARILI | 7.886 | 4.114 | 0.1489 | 0.740 |
| +20% | 22.8 | denge | BAŞARILI | 0.126 | 11.874 | 0.1491 | 0.000 |
| +30% | 24.7 | swing-up | BAŞARISIZ | — | 0.000 | 0.0766 | 0.000 |
| +30% | 24.7 | denge | BAŞARILI | 0.132 | 11.868 | 0.1506 | 0.000 |
### motor.r_pulley — Kasnak yarıçapı


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 0.0066843 | swing-up | BAŞARISIZ | — | 0.000 | 0.0637 | 0.000 |
| -30% | 0.0066843 | denge | BAŞARILI | 0.142 | 11.858 | 0.1623 | 0.000 |
| -20% | 0.0076392 | swing-up | BAŞARILI | 10.078 | 1.922 | 0.1250 | 0.000 |
| -20% | 0.0076392 | denge | BAŞARILI | 0.128 | 11.872 | 0.1489 | 0.000 |
| -10% | 0.0085941 | swing-up | BAŞARILI | 5.392 | 6.608 | 0.1411 | 1.152 |
| -10% | 0.0085941 | denge | BAŞARILI | 0.120 | 11.880 | 0.1465 | 0.000 |
| +10% | 0.0105039 | swing-up | BAŞARILI | 3.348 | 8.652 | 0.1703 | 0.706 |
| +10% | 0.0105039 | denge | BAŞARILI | 0.108 | 11.892 | 0.1355 | 0.000 |
| +20% | 0.0114588 | swing-up | BAŞARILI | 3.550 | 8.450 | 0.2024 | 0.514 |
| +20% | 0.0114588 | denge | BAŞARILI | 0.106 | 11.894 | 0.1315 | 0.000 |
| +30% | 0.0124137 | swing-up | BAŞARILI | 3.526 | 8.474 | 0.2499 | 0.480 |
| +30% | 0.0124137 | denge | BAŞARILI | 0.104 | 11.896 | 0.1288 | 0.000 |
### motor.J_rotor — Motor rotor ataleti


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 2.38e-07 | swing-up | BAŞARILI | 4.400 | 7.600 | 0.1476 | 1.078 |
| -30% | 2.38e-07 | denge | BAŞARILI | 0.108 | 11.892 | 0.1383 | 0.000 |
| -20% | 2.72e-07 | swing-up | BAŞARILI | 4.542 | 7.458 | 0.1446 | 0.914 |
| -20% | 2.72e-07 | denge | BAŞARILI | 0.110 | 11.890 | 0.1408 | 0.000 |
| -10% | 3.06e-07 | swing-up | BAŞARILI | 4.534 | 7.466 | 0.1411 | 0.854 |
| -10% | 3.06e-07 | denge | BAŞARILI | 0.112 | 11.888 | 0.1417 | 0.000 |
| +10% | 3.74e-07 | swing-up | BAŞARILI | 4.612 | 7.388 | 0.1427 | 0.760 |
| +10% | 3.74e-07 | denge | BAŞARILI | 0.114 | 11.886 | 0.1408 | 0.000 |
| +20% | 4.08e-07 | swing-up | BAŞARILI | 4.640 | 7.360 | 0.1417 | 0.728 |
| +20% | 4.08e-07 | denge | BAŞARILI | 0.116 | 11.884 | 0.1408 | 0.000 |
| +30% | 4.42e-07 | swing-up | BAŞARILI | 4.644 | 7.356 | 0.1426 | 0.686 |
| +30% | 4.42e-07 | denge | BAŞARILI | 0.118 | 11.882 | 0.1403 | 0.000 |
### motor.T_f — Motor Coulomb sürtünme torku


| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|
| -30% | 0.00105539 | swing-up | BAŞARILI | 4.020 | 7.980 | 0.1556 | 0.734 |
| -30% | 0.00105539 | denge | BAŞARILI | 0.106 | 11.894 | 0.0961 | 0.000 |
| -20% | 0.00120616 | swing-up | BAŞARILI | 4.034 | 7.966 | 0.1509 | 0.714 |
| -20% | 0.00120616 | denge | BAŞARILI | 0.108 | 11.892 | 0.1083 | 0.000 |
| -10% | 0.00135693 | swing-up | BAŞARILI | 3.870 | 8.130 | 0.1450 | 0.746 |
| -10% | 0.00135693 | denge | BAŞARILI | 0.110 | 11.890 | 0.1237 | 0.000 |
| +10% | 0.00165847 | swing-up | BAŞARILI | 4.570 | 7.430 | 0.1544 | 0.856 |
| +10% | 0.00165847 | denge | BAŞARILI | 0.116 | 11.884 | 0.1571 | 0.000 |
| +20% | 0.00180924 | swing-up | BAŞARILI | 5.286 | 6.714 | 0.1709 | 0.966 |
| +20% | 0.00180924 | denge | BAŞARILI | 0.118 | 11.882 | 0.1709 | 0.000 |
| +30% | 0.00196001 | swing-up | BAŞARILI | 4.996 | 7.004 | 0.1808 | 1.014 |
| +30% | 0.00196001 | denge | BAŞARILI | 0.120 | 11.880 | 0.1813 | 0.000 |
### motor.B_m — Motor viskoz sürtünmesi

Sıfır nominal için mutlak referans: 2.525856e-08 N·m·s/rad; yalnız pozitif ekleme yapılır.

| Sapma | Gerçek değer | Deney | Durum | İlk denge [s] | Son denge [s] | max abs(x) [m] | Doyum [s] |
|---|---|---|---|---|---|---|---|

-30%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.


-20%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.


-10%: Sıfır nominalde negatif sapma fiziksel değil; pozitif mutlak tarama kullanılır.

| +10% | 2.525856e-09 | swing-up | BAŞARILI | 4.550 | 7.450 | 0.1396 | 0.806 |
| +10% | 2.525856e-09 | denge | BAŞARILI | 0.114 | 11.886 | 0.1423 | 0.000 |
| +20% | 5.051712e-09 | swing-up | BAŞARILI | 4.542 | 7.458 | 0.1422 | 0.806 |
| +20% | 5.051712e-09 | denge | BAŞARILI | 0.114 | 11.886 | 0.1412 | 0.000 |
| +30% | 7.577568e-09 | swing-up | BAŞARILI | 4.552 | 7.448 | 0.1396 | 0.814 |
| +30% | 7.577568e-09 | denge | BAŞARILI | 0.114 | 11.886 | 0.1399 | 0.000 |

## Sınırlar ve yeniden çalıştırma

- Sıfır nominal referansları keşif ölçekleridir; ölçülmüş motor verisi değildir.
- I için referans m·l²; rotor için arabaya yansıması M olan atalet; B_m için max(b, 0.1) N·s/m kullanılır.
- T_f için referans, Pololu 37D tahmininin (Kt·I₀, I₀ = 0.2 A) 5 katıdır: +%10/20/30 sütunları tahminin 0.5/1/1.5 katıdır. Kontrolcü T_f'yi bilmez (telafi kapalı, #30); arayüzdeki Sürtünme Ölç ile ölçülüp telafi açılırsa bu sonuç değişir. Sürtünme ölçülmeden gerçek denemeye geçilmemelidir.
- Gerçek I / motor kayıpları ölçüldüğünde params.json güncellenmeli; ardından ±% taraması yeniden çalıştırılmalı.
- Motor parametreleri yalnız voltage modunda değerlendirilir. V_max tek başına kuvvet modelini değiştirmediğinden taranmaz; giriş sınırı u_max sabittir.
- Kontrolcü yeniden tasarlanmaz; fiziksel parametre uyumsuzluğu ölçülür. Ölçüm ölçek hataları, gecikme ve birleşik sapmalar bu deneyin dışında kalır.
- Çalıştır: `python tools/hassasiyet_analizi.py`; makine tarafından okunabilir sonuçlar JSON dosyasına da yazılır.
