# Matlab/Simulink Modeliyle Karşılaştırma (#10)

Bu belge #10 görevi kapsamında yapılan karşılaştırmayı, bulunan farkları ve bunları denetleyen testleri özetler.
Kod: [`pendulum_lab/sim/matlab_compat.py`](../pendulum_lab/sim/matlab_compat.py) ·
Testler: [`tests/test_matlab_compat.py`](../tests/test_matlab_compat.py) ·
Matlab dosyaları: [`matlab/`](../matlab/README.md) · PR: #37

---

## Amaç
Mekatronik ekibi modeli ve LQR kontrolcüsünü Matlab/Simulink'te (`cart_pendulum_lqr.m`, `cart_pendulum_application.m`,
`cart_pendulum_lqr_sim.slx`), bilgisayar ekibi ise Python kütüphanesinde (`pendulum_lab`) geliştirdi. İki tarafın aynı
sistemi modellediğini göstermek ve ileride birinin değişip diğerinin geride kalmasını otomatik olarak yakalamak gerekiyordu.

## Bulunan farklar

### 1. θ işareti
Matlab'da sarkaç **sola** yatınca θ > 0; Python'da, ESP32 firmware'inde ve [`protokol.md`](protokol.md)'de **sağa (+x)**
yatınca θ > 0. Durum dönüşümü:

$$s_{\text{Matlab}} = T\,s_{\text{proje}}, \qquad T = \operatorname{diag}(1,\ 1,\ -1,\ -1), \qquad T^{-1} = T$$

Buna göre modeller ve kazançlar arasındaki ilişki:

$$A_{\text{Matlab}} = T A T, \qquad B_{\text{Matlab}} = T B, \qquad K_{\text{Matlab}} = K\,T$$

Yani K'nın θ ve θ̇ kazançlarının işareti terstir:

| | $k_x$ | $k_{\dot x}$ | $k_\theta$ | $k_{\dot\theta}$ |
|---|---|---|---|---|
| Matlab (θ sola +) | −50.00 | −45.37 | **+168.66** | **+33.25** |
| Proje (θ sağa +), ESP32'ye giden | −50.00 | −45.37 | **−168.66** | **−33.25** |

**Risk:** Matlab'da hesaplanan K dönüştürülmeden ESP32'ye verilirse kapalı çevrim kararsızdır (en büyük özdeğerin gerçel
kısmı ≈ +237 s⁻¹). Simülasyonda 5°'den bırakılan sarkaçta araba **0.15 s** içinde ray sınırına çarpıyor.
Bunu önlemek için `gain_from_matlab()` / `gain_to_matlab()` dönüşüm fonksiyonları eklendi ve bu durum testle belgelendi.

### 2. Deneme biçimi
`cart_pendulum_application.m`'deki deneme, kütüphanenin varsayılan simülasyonundan farklıdır:

| | Matlab denemesi | Kütüphane (`simulate_closed_loop`) |
|---|---|---|
| Kontrolcü | Sürekli zamanlı | Ayrık, $T_s$ = 2 ms (ESP32 gibi) |
| Giriş | Kuvvet, ±10 N sınırlı | Motor gerilimi (veya kuvvet) |
| Sensör | İdeal | Kuantalama, gürültü, gecikme seçilebilir |
| İntegrasyon | RK4, dt = 2.5 ms | RK4 |
| Durma | \|x\| ≥ 0.2 m veya \|θ\| ≥ 90° | Ray uç durdurucuları |

`simulate_matlab_compatible()` Matlab'daki denemeyi Python'da birebir tekrarlar. Arayüzde
**Analiz ve Simülasyon → Matlab uyumlu** seçeneği bunu kullanır ve K'yı iki işaret kuralında da gösterir.

## Doğrulama sonuçları
Matlab referans parametreleri: M = 0.5 kg, m = 0.2 kg, l = 0.3 m, I = 0.006 kg·m², b = 0.1 N·s/m, pivot sürtünmesi yok;
Q = diag(1/0.2², 1/0.5², 1/(10°)², 1) = diag(25, 4, 32.83, 1), R = 1/Fmax² = 0.01.

| Karşılaştırma | Sonuç |
|---|---|
| Doğrusal model (A, B) | θ işareti dışında birebir aynı (fark < 10⁻¹²) |
| Doğrusal olmayan hareket denklemleri | θ işareti dışında aynı |
| LQR kazancı K | Matlab sonucu bağımsız modelden yeniden üretildi (fark < 0.01) |
| Kapalı çevrim kutupları | −58.39, −5.65, −2.40 ± 0.99j (iki tarafta aynı) |
| Sürekli (Matlab) ve 500 Hz ayrık (kütüphane) simülasyon, θ₀ = 8°, x₀ = 5 cm, 4 s | En büyük fark θ'da 0.08°, x'te 0.6 mm |

### Matlab uyumlu deneme: başlangıç açısına göre
Başlangıç hızı sıfır, araba merkezde, 10 s.

| θ₀ | Sonuç | Max \|x\| | Kuvvet sınırında geçen süre |
|---|---|---|---|
| 5° | Dengede kaldı | 0.061 m | 0.010 s |
| 10° | Dengede kaldı | 0.131 m | 0.033 s |
| 20° | Ray sınırı (t = 0.18 s) | 0.202 m | 0.092 s |
| 30° | Ray sınırı (t = 0.16 s) | 0.206 m | 0.163 s |

Bu kontrolcünün ray sınırına çarpmadan kurtarabildiği en büyük başlangıç açısı ±10 N sınırıyla **≈ 14.1°**,
kuvvet sınırsız olsa bile ≈ 15.5°'dir. Asıl kısıt kuvvet değil **±0.2 m ray boyudur**. 10° başlangıçta sınırsız
kontrolcü ≈ 29 N isterdi; ±10 N sınırı yalnız ilk ≈ 30 ms'de devreye giriyor. Daha büyük açılar için swing-up
ve mod geçişi gerekir (bkz. [`proje_durumu.md`](proje_durumu.md)).

## Testler
`tests/test_matlab_compat.py`'deki 14 test her push'ta GitHub Actions'ta çalışır:

* **Model:** Matlab'ın kapalı formülleriyle kütüphaneden bağımsız kurulan A, B'den Matlab K'sı yeniden üretiliyor;
  kütüphanenin doğrusal ve doğrusal olmayan modeli θ işareti dışında Matlab'ınkiyle aynı; kutuplar iki tarafta aynı.
* **İşaret:** Dönüşümler kendi tersi; dönüştürülmemiş Matlab K'sı kararsız, dönüştürülmüş K kararlı.
* **Deneme:** Matlab uyumlu simülasyon dengeye getiriyor; açı ve ray sınırında duruyor; motor modundan bağımsız
  olarak kuvvet girişiyle çalışıyor; ±10 N sınırı ve Q'nun tam değeri Matlab'la aynı; 500 Hz ayrık simülasyonla aynı
  yörüngeyi veriyor; hatalı K'yı reddediyor.

Yeniden üretmek için:

```bash
python -m pytest tests/test_matlab_compat.py -v
```

## Dikkat: Matlab K'sı ESP32 için yeterli değil
İşaret dönüşümü yalnız işaret farkını giderir. Matlab modeli kuvvet girişlidir ve motor içermez; ESP32 ise motora
gerilim uygular ve 2 ms örneklemeyle çalışır. Aynı Q, R ile bile motorlu modelden çıkan K farklıdır
(proje parametreleriyle K ≈ [−50, −53.06, −148.25, −25.76]). Gerçek düzenekte kullanılacak K, arayüzün
**Model ve LQR Tasarımı** sekmesinde motorlu modelle hesaplanmalıdır.

## Katkılar
* **Melihcan Arslaner:** karşılaştırma testi, işaret dönüşüm fonksiyonları, `simulate_matlab_compatible`,
  Analiz sekmesindeki "Matlab uyumlu" seçeneği, `matlab/README.md`, bu belge.
* **Ahmet Selim (inceleme ve tamamlama):** Matlab dosyalarının eklenmesi ve dosyalarla doğrulama; ±10 N kuvvet
  sınırı, Q'nun tam değeri ve doğrusal olmayan model testi; "Matlab'ın kendi ayarlarıyla" seçeneği; başlangıç
  hızlarının Matlab modunda simülasyona verilmesi.
