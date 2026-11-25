# Matlab / Simulink dosyaları (mekatronik ekibi)

Bu klasörde mekatronik ekibinin Matlab dosyaları bulunur (issue #10):

| Dosya | Ne yapar |
|---|---|
| `cart_pendulum_lqr.m` | Model (A, B) ve LQR tasarımı. **Simulink modelinden önce çalıştırılmalı**, çünkü modelin kullandığı değişkenleri (A, B, K, …) çalışma alanına yazar |
| `cart_pendulum_application.m` | Deneme: sürekli LQR, kuvvet girişi (±10 N sınırlı), RK4 dt = 2.5 ms; ±0.2 m veya \|θ\| ≥ 90°'de durur. Dış itme (Apply push) arabaya uygulanır |
| `cart_pendulum_lqr_sim.slx` | Simulink modeli (ode45, RelTol 1e-7, ±Fmax kuvvet sınırı) |

## θ işareti
Matlab dosyalarında, mekatronik raporunda (Bölüm 8.1.3) ve **#50'den beri** Python kütüphanesinde, ESP32 firmware'inde ve
[`docs/protokol.md`](../docs/protokol.md)'de θ, sarkaç **sola** yatınca pozitiftir. A, B ve K iki tarafta aynıdır:

```
Matlab K = [−50, −45.37, +168.66, +33.25]
Proje  K = [−50, −45.37, +168.66, +33.25]   (aynı parametre, Q, R ile)
```

Matlab'da hesaplanan K'nın işareti değiştirilmez. **Dikkat:** #50'den önce proje θ'yı sağa pozitif sayıyordu ve
K'nın θ, θ̇ kazançları ters işaretliydi. O dönemde üretilmiş bir K (ör. eski `gains.h`) yeni firmware'e verilirse
sarkaç düşer; `gains.h`'yi arayüzden yeniden dışa aktarın. `gain_from_matlab` / `gain_to_matlab` geriye uyumluluk için
duruyor ve artık K'yı değiştirmez.

İşaret aynı olsa da Matlab'ın K'sı ESP32 için doğrudan uygun değildir: Matlab modeli kuvvet girişlidir (motor yok),
ESP32 ise motora gerilim uygular ve 2 ms örneklemeyle çalışır. Gerçek düzenekte kullanılacak K'yı arayüzün
**Model ve LQR Tasarımı** sekmesinde, motorlu modelle hesaplayın (aynı Q, R ile bile Matlab'ınkinden farklı çıkar).

## Karşılaştırma
* `tests/test_matlab_compat.py`: Matlab'ın A, B formülleriyle kurulan modelden çıkan K'nın Matlab sonucuyla,
  Python kütüphanesinin K'sının da (aynı işaretle) bununla aynı olduğunu her push'ta denetler.
* Arayüz → **Analiz ve Simülasyon** → **Matlab uyumlu** kutusu: `cart_pendulum_application.m`'deki denemeyi
  Python'da tekrarlar ve K'yı gösterir. **"Matlab'ın kendi ayarlarıyla"** seçiliyken (varsayılan)
  `cart_pendulum_lqr.m`'nin parametreleri ve Q, R'si kullanılır; sonuç Matlab'ınkiyle karşılaştırılabilir.
  Seçili değilken projenin parametreleri ve Tasarım sekmesindeki Q, R kullanılır; o zaman çıkan K Matlab'ın K'sı
  değildir.
