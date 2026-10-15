"""Fiziksel parametreler.

Mekanik tasarım bitince mekatronik ekibi değerleri config/params.json dosyasına yazar;
kodun geri kalanı hiç değişmez.

Konvansiyonlar (tüm proje boyunca aynı):
    durum  s = [x, dx, th, dth]
    x   : araba konumu [m], rayın ortası 0, sağa (+)
    th  : sarkaç açısı [rad], DİK konum 0, sarkaç +x yönüne yattığında (+)
    u   : giriş — input_mode'a göre kuvvet [N], motor gerilimi [V] veya araba ivmesi [m/s^2]
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field, asdict
from pathlib import Path

INPUT_MODES = ("force", "voltage", "acceleration")
BOB_SHAPES = ("point", "sphere", "cube")


@dataclass
class MotorParams:
    """Redüktörlü DC motor + kayış kasnağı (input_mode='voltage' için).

    Varsayılanlar Pololu 37D 12V katalog değerlerinden kabaca türetilmiş YER TUTUCULARDIR.
    Not: 30:1 redüktör + 20 dişli kasnakla araba en fazla ~0.2 m/s gidebilir; sarkaç için
    yetersiz. Bu yüzden yer tutucu olarak 10:1 + GT2 40 diş (~1.4 m/s) seçildi.
    """
    R: float = 2.2          # sargı direnci [ohm]
    Kt: float = 0.0083      # tork sabiti, motor mili [N·m/A]
    Ke: float = 0.0109      # zıt EMK sabiti, motor mili [V·s/rad]
    gear: float = 10.0      # redüktör oranı
    r_pulley: float = 0.0127   # kasnak yarıçapı [m] (GT2 40 diş)
    V_max: float = 12.0     # besleme gerilimi [V]
    J_rotor: float = 0.0    # rotor ataleti, motor mili [kg·m^2] — datasheet'ten girilmeli (0 = ihmal)

    @property
    def reflected_mass(self) -> float:
        """Rotor ataletinin arabaya yansıyan eşdeğer kütlesi: J_rotor·N²/r² [kg]."""
        return self.J_rotor * self.gear ** 2 / self.r_pulley ** 2

    @property
    def alpha(self) -> float:
        """F = alpha*V - beta*dx  → gerilimden kuvvete kazanç [N/V]."""
        return self.Kt * self.gear / (self.R * self.r_pulley)

    @property
    def beta(self) -> float:
        """Zıt EMK'nın araba hızına yansıyan sönüm katsayısı [N·s/m]."""
        return self.Kt * self.Ke * self.gear ** 2 / (self.R * self.r_pulley ** 2)


@dataclass
class PendulumParams:
    M: float = 0.5          # araba kütlesi [kg]
    m: float = 0.2          # sarkacın toplam kütlesi [kg]
    l: float = 0.3          # pivot → sarkaç ağırlık merkezi [m]
    I: float = 0.0          # sarkacın KENDİ ağırlık merkezi etrafındaki ataleti [kg·m^2]
    b: float = 0.1          # araba viskoz sürtünmesi [N·s/m]
    c: float = 0.0005       # pivot sürtünmesi [N·m·s/rad]
    g: float = 9.81
    rail_half: float = 0.40  # rayın yarı boyu (orta → limit switch) [m]
    u_max: float = 12.0     # giriş doyumu (N, V veya m/s^2)
    input_mode: str = "voltage"
    motor: MotorParams = field(default_factory=MotorParams)
    note: str = "YER TUTUCU DEĞERLER — mekatronik ekibinden gerçek değerler gelince güncellenecek"

    def __post_init__(self):
        if self.input_mode not in INPUT_MODES:
            raise ValueError(f"input_mode {INPUT_MODES} içinden olmalı, verilen: {self.input_mode!r}")
        if isinstance(self.motor, dict):
            self.motor = MotorParams(**self.motor)
        for name in ("M", "m", "l"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} pozitif olmalı")
        if self.I < 0:
            raise ValueError("I negatif olamaz")
        if self.input_mode == "voltage" and self.u_max > self.motor.V_max + 1e-9:
            raise ValueError(f"u_max ({self.u_max} V) besleme gerilimini ({self.motor.V_max} V) aşamaz")

    @property
    def M_eff(self) -> float:
        """Arabanın eşdeğer kütlesi: DC motorda rotor ataleti de arabaya eklenir."""
        return self.M + (self.motor.reflected_mass if self.input_mode == "voltage" else 0.0)

    @property
    def J(self) -> float:
        """Pivot etrafındaki toplam atalet (paralel eksen): I + m*l^2."""
        return self.I + self.m * self.l ** 2

    @property
    def omega_n(self) -> float:
        """Sarkacın küçük açı doğal frekansı [rad/s]."""
        return math.sqrt(self.m * self.g * self.l / self.J)

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "PendulumParams":
        d = dict(d)
        geom = d.pop("geometry", None)
        if geom:
            m, l, I = pendulum_from_geometry(**geom)
            d.update(m=m, l=l, I=I)
        d.pop("_aciklama", None)
        return cls(**d)


def bob_inertia(shape: str, mass: float, size: float) -> float:
    """Uç kütlenin kendi merkezi etrafındaki ataleti.

    shape='sphere': size = yarıçap  → 2/5 m r^2 (dolu küre)
    shape='cube'  : size = kenar    → 1/6 m a^2 (dolu küp, yüzeye paralel eksen)
    shape='point' : 0
    """
    if shape == "point":
        return 0.0
    if shape == "sphere":
        return 0.4 * mass * size ** 2
    if shape == "cube":
        return mass * size ** 2 / 6.0
    raise ValueError(f"shape {BOB_SHAPES} içinden olmalı")


def pendulum_from_geometry(rod_mass: float, rod_length: float, bob_mass: float,
                           bob_shape: str = "sphere", bob_size: float = 0.0,
                           bob_center: float | None = None) -> tuple[float, float, float]:
    """Çubuk + uç kütleden (küre/küp) eşdeğer (m, l, I) hesaplar.

    rod_length: pivottan çubuk ucuna [m]; bob_center: pivottan uç kütle merkezine
    (verilmezse çubuğun ucu). Mekanizma ekibinin 'küre mi küp mü' kararını
    karşılaştırmak için kullanılabilir (bkz. tests/test_model.py).
    """
    if bob_center is None:
        bob_center = rod_length
    m = rod_mass + bob_mass
    l = (rod_mass * rod_length / 2 + bob_mass * bob_center) / m
    # pivot etrafındaki atalet, sonra ağırlık merkezine taşı
    J_pivot = rod_mass * rod_length ** 2 / 3 + bob_inertia(bob_shape, bob_mass, bob_size) + bob_mass * bob_center ** 2
    I = J_pivot - m * l ** 2
    return m, l, I


def load_params(path: str | Path) -> PendulumParams:
    with open(path, encoding="utf-8") as f:
        return PendulumParams.from_dict(json.load(f))


def save_params(p: PendulumParams, path: str | Path) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(p.to_dict(), f, indent=2, ensure_ascii=False)
