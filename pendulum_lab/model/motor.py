"""Motor modelleri ve transfer fonksiyonları (tahtadaki "Pololu / NEMA17 bilgileri (Transfer Fonksiyonu)" maddesi).

DC motor:     ω(s)/V(s) = Kt / ((J s + b)(L s + R) + Kt Ke)
Step motor:   ideal konum kaynağı kabul edilir → input_mode='acceleration'; sınır, torktan gelen
              en büyük ivmedir (stepper_max_accel).

Katalog değerleri yaklaşıktır; satın alınacak modelin datasheet'inden doğrulanmalıdır.
"""

from __future__ import annotations

import numpy as np

from .params import MotorParams, PendulumParams

# Pololu 37D 12V ailesi (motor mili değerleri tüm redüktör oranlarında aynı). Issue #7'deki yöntemle datasheet'ten
# (12 V, 10000 rpm ve 0.2 A boşta, 5.5 A ve 0.5 kg·cm durma): R = V/I_durma, Kt = T_durma/I_durma, Ke = (V − R·I₀)/ω₀.
# Karşılaştırma (tools/motor_karsilastirma.py, #7): ÖNERİLEN 10:1 + GT2 40 diş; 30:1 + 40 diş swing-up'ta yetersiz.
# T_f ≈ Kt·I₀ ≈ 0.0018 N·m yalnız tahmindir; ölçülmeden girilmez (telafi fazla tahminle çalışırsa swing-up bozulur).
# NEMA17 bu tabloya girmez: step motor ivme kaynağıdır (input_mode='acceleration', bkz. stepper_max_accel).
# Farklı motor eklemek için aynı alanlarla yeni satır ekleyin.
MOTOR_CATALOG: dict[str, dict] = {
    "pololu_37d_12v_10to1": dict(R=2.18, Kt=0.00892, Ke=0.01104, gear=10.0, r_pulley=0.0127, V_max=12.0),
    "pololu_37d_12v_19to1": dict(R=2.18, Kt=0.00892, Ke=0.01104, gear=19.0, r_pulley=0.0127, V_max=12.0),
    "pololu_37d_12v_30to1": dict(R=2.18, Kt=0.00892, Ke=0.01104, gear=30.0, r_pulley=0.0127, V_max=12.0),
}


def motor_from_catalog(name: str) -> MotorParams:
    return MotorParams(**MOTOR_CATALOG[name])


def dc_motor_tf(R: float, L: float, Kt: float, Ke: float, J: float, b: float):
    """Gerilim → mil açısal hızı transfer fonksiyonu (pay, payda katsayıları, azalan s kuvveti)."""
    num = np.array([Kt])
    den = np.polymul([J, b], [L, R]) + np.array([0, 0, Kt * Ke])
    return num, den


def cart_velocity_tf(p: PendulumParams):
    """Gerilim → araba hızı (sarkaç kütlesi arabaya eklenmiş, endüktans ihmal):
    v(s)/V(s) = alpha / ((M+m) s + b + beta)."""
    mp = p.motor
    return np.array([mp.alpha]), np.array([p.M_eff + p.m, p.b_cart + mp.beta])


def max_cart_speed(p: PendulumParams) -> float:
    """Tam gerilimde ulaşılabilecek kararlı araba hızı [m/s] (Coulomb sürtünmesi düşülmüş)."""
    return max(0.0, p.motor.alpha * p.motor.V_max - p.F_coulomb) / (p.b_cart + p.motor.beta)


def max_cart_force(p: PendulumParams) -> float:
    """Durağan halde tam gerilimdeki kuvvet [N]."""
    return p.motor.alpha * p.motor.V_max


def stepper_max_accel(torque_Nm: float, r_pulley: float, moving_mass: float, safety: float = 0.5) -> float:
    """Step motorun adım kaçırmadan verebileceği yaklaşık ivme [m/s^2].
    safety: hızla düşen tork eğrisi için pay (tutma torkunun ~%50'si)."""
    return safety * torque_Nm / (r_pulley * moving_mass)
