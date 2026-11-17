# Matlab / Simulink dosyaları (mekatronik ekibi)

Bu klasöre mekatronik ekibinin Matlab dosyaları konacak (issue #10):

| Dosya | Ne yapar |
|---|---|
| `cart_pendulum_lqr.m` | Model (A, B) ve LQR tasarımı. **Simulink modelinden önce çalıştırılmalı**, çünkü modelin kullandığı değişkenleri (A, B, K, …) çalışma alanına yazar |
| `cart_pendulum_application.m` | Deneme: sürekli LQR, kuvvet girişi, RK4 dt = 2.5 ms; ±0.2 m veya \|θ\| ≥ 90°'de durur |
| `cart_pendulum_lqr_sim.slx` | Simulink modeli (ode45, RelTol 1e-7) |

## θ işareti — dikkat
Matlab dosyalarında θ, sarkaç **sola** yatınca pozitiftir. Python kütüphanesinde, ESP32 firmware'inde ve
[`docs/protokol.md`](../docs/protokol.md)'de ise sarkaç **sağa (+x)** yatınca pozitiftir. Bu yüzden K'nın θ ve θ̇
kazançlarının işareti terstir:

```
Matlab K = [−50, −45.37, +168.66, +33.25]
Proje  K = [−50, −45.37, −168.66, −33.25]   ← ESP32'ye giden budur
```

**Matlab'da hesaplanan bir K'yı doğrudan ESP32'ye göndermeyin, sarkaç anında düşer.** Önce dönüştürün:

```python
from pendulum_lab.sim.matlab_compat import gain_from_matlab
K = gain_from_matlab([-50, -45.37, 168.66, 33.25])   # → [-50, -45.37, -168.66, -33.25]
```

## Karşılaştırma
* `tests/test_matlab_compat.py`: Matlab'ın A, B formülleriyle kurulan modelden çıkan K'nın Matlab sonucuyla,
  Python kütüphanesinin K'sının da (θ işareti dışında) bununla aynı olduğunu her push'ta denetler.
* Arayüz → **Analiz ve Simülasyon** → **Matlab uyumlu** kutusu: `cart_pendulum_application.m`'deki denemeyi
  Python'da tekrarlar ve K'yı iki işarette de gösterir.
