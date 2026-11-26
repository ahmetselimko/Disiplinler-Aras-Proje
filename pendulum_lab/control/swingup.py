"""Enerji bazlı swing-up (Åström & Furuta, 2000).

Sarkaç enerjisi  E = ½ J θ̇² + m g l (cosθ − 1)   (dik-durağan: 0, aşağı-durağan: −2mgl)
θ > 0 sola yatık iken (mekatronik raporu 8.2.2) araba ivmesi a için  dE/dt = m l a θ̇ cosθ
(sürtünme ihmal) olduğundan

    a = sat( −k_E · E · θ̇ · cosθ )        (mekatronik raporu 8.2.3 ile aynı yasa)

seçimi dE/dt = −m l k_E E (θ̇cosθ)² verir → E monoton olarak 0'a (dik konum enerjisine) gider.
Rayın dışına taşmamak için araba merkezleme terimi eklenir: −k_x x − k_v ẋ.

Hedef enerji oranı E_ratio, hedefi aşağı-durağan konumdan ölçülen enerjinin dik konumdakine (2mgl)
oranı olarak verir: 1 → tam dik konum enerjisi (E = 0), 1.1 → %10 fazlası (E = 0.2 mgl).
DİKKAT, tanımlar farklı: mekatroniğin E_ref = 1.02·mgl'si (enerji dikte +mgl, aşağıda −mgl; rapor 8.2.4) bizim
tanımımızda E_ratio = 1.01'e karşılık gelir (fazla enerji 0.02·mgl / 2mgl). Varsayılan 1.05 (#50 Aşama 2):
mekatronik tasarım değerleriyle (+20 g / +50 g uç kütlesi, 3 N sürtünme) 1.0'da sarkaç kalkmıyor, 1.05'te
~4 s'de dengeye geliyor (docs/mekatronik_on_degerlendirme.md).
Çıkış İVMEdir; motor girişine dynamics.accel_to_input ile çevrilir.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..model.dynamics import energy, wrap_angle
from ..model.params import PendulumParams


@dataclass
class EnergySwingUp:
    # Varsayılanlar yer tutucu parametrelerle simülasyon taramasından seçildi (±0.4 m rayda en çok
    # ~0.2 m sapma, ~4 s'de yakalama). Gerçek parametreler gelince Analiz sekmesinden tekrar denenmeli.
    k_E: float = 100.0      # enerji kazancı (büyük → neredeyse bang-bang)
    a_max: float = 4.0      # ivme sınırı [m/s^2] — ray kısa olduğu için düşük tutuldu
    k_x: float = 10.0       # merkezleme (konum) kazancı
    k_v: float = 6.0        # merkezleme (hız) kazancı
    a_kick: float = 2.0     # sarkaç tam aşağıda hareketsizken ilk dürtme ivmesi
    E_ratio: float = 1.05   # hedef enerji oranı (1 = dik konum enerjisi; mekatroniğin 1.02·mgl'si = 1.01)

    def target_energy(self, p: PendulumParams) -> float:
        """Hedef enerji [J], energy() ile aynı sıfır noktasında (dik-durağan = 0)."""
        return (self.E_ratio - 1.0) * 2 * p.m * p.g * p.l

    def accel(self, p: PendulumParams, s) -> float:
        x, dx, th, dth = s
        E = energy(p, th, dth) - self.target_energy(p)
        E_bottom = 2 * p.m * p.g * p.l
        if abs(dth) < 0.05 and abs(wrap_angle(th)) > 2.8 and energy(p, th, dth) < -0.99 * E_bottom:
            # aşağıda hareketsiz: θ̇cosθ = 0 olduğundan enerji kuralı tek başına başlatamaz → rayın
            # ortasına doğru küçük bir dürtme
            a = self.a_kick * (-1.0 if x > 1e-3 else 1.0)
        else:
            a = -self.k_E * E * dth * math.cos(th)
        a = max(-self.a_max, min(self.a_max, a))
        a += -self.k_x * x - self.k_v * dx
        return max(-self.a_max, min(self.a_max, a))
