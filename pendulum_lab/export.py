"""Tasarlanan kazançları ESP32 firmware'ine gömülecek C başlık dosyasına (gains.h) aktarır.

Böylece kart PC'ye bağlı olmadan da son tasarlanan K ile açılır; PC bağlanınca set_gain ile
çalışma anında değiştirilebilir.
"""

from __future__ import annotations

import math
import time
from pathlib import Path

import numpy as np

from .control.supervisor import CATCH_DTH, CATCH_DX, CATCH_X
from .control.swingup import EnergySwingUp
from .filters import design_kalman


PLACEHOLDER_COMMENT = "// UYARI: YER TUTUCU parametrelerle üretildi — gerçek ölçüler girilip yeniden üretilmeli."
PLACEHOLDER_WARNING = ('#warning "gains.h YER TUTUCU parametrelerle uretildi: olculmus degerleri girip '
                       'arayuzden yeniden disa aktarin"')


def _c_vec(v) -> str:
    return "{" + ", ".join(f"{float(x):.8e}f" for x in v) + "}"


def _c_mat(M) -> list[str]:
    return [f"  {_c_vec(row)}," for row in M]


def gains_header(p, K, Ts: float, Q=None, R=None, swingup=None, catch_deg: float = 15.0,
                 release_deg: float = 30.0, catch_dth: float = CATCH_DTH, catch_x: float = CATCH_X,
                 catch_dx: float = CATCH_DX) -> str:
    K = np.asarray(K, dtype=float).ravel()
    period_ms = Ts * 1000
    if abs(period_ms - round(period_ms)) > 1e-9 or not 1 <= round(period_ms) <= 20:
        # firmware vTaskDelayUntil ile 1 ms'lik FreeRTOS tikinde çalışır; protokol.md §7: 1–20 ms
        raise ValueError(f"Ts tam sayı milisaniye ve 1–20 ms olmalı, verilen {period_ms:g} ms")
    if swingup is None:   # firmware SU_* değerlerini gains.h'den bekler (varsayılanı yok): eksik kalmasın
        swingup = EnergySwingUp()
    Ad, Bd, L, _ = design_kalman(p, Ts)
    lines = [
        "// Otomatik üretildi: pendulum_lab.export  —  elle düzenlemeyin, arayüzden yeniden üretin",
        f"// {time.strftime('%Y-%m-%d %H:%M:%S')}",
        f"// Q = {np.ravel(Q).astype(float).tolist() if Q is not None else '-'}   R = {np.ravel(R).tolist() if R is not None else '-'}",
        f"// Parametreler: M={p.M} m={p.m} l={p.l} I={p.I} b={p.b} c={p.c} giriş={p.input_mode}"
        f" T_f={p.motor.T_f} B_m={p.motor.B_m}",
        "#pragma once",
        *([PLACEHOLDER_COMMENT, PLACEHOLDER_WARNING] if p.placeholder else []),
        "",
        "// θ işaret kuralı: θ > 0 sarkaç SOLA yatık (#50). Firmware bu makro yoksa derlenmez: eski (θ sağa +)",
        "// gains.h'de K'nın θ, θ̇ kazançları ve Kalman matrisleri ters işaretlidir.",
        "#define THETA_POSITIVE_LEFT 1",
        f"#define CTRL_TS_S          {Ts:.6f}f",
        f"#define CTRL_PERIOD_MS     {round(period_ms)}",
        f"#define CTRL_RATE_HZ       {round(1 / Ts)}",
        "// u = -(K0*(x - x_ref) + K1*dx + K2*th + K3*dth)",
        "static const float LQR_K[4] = {" + ", ".join(f"{k:.6f}f" for k in K) + "};",
        f"#define CATCH_RAD          {math.radians(catch_deg):.6f}f",
        f"#define RELEASE_RAD        {math.radians(release_deg):.6f}f",
        f"#define CATCH_DTH          {catch_dth:.4f}f   // LQR'a girişte |θ̇| sınırı [rad/s]",
        f"#define CATCH_X            {catch_x:.4f}f   // LQR'a girişte |x| sınırı [m]",
        f"#define CATCH_DX           {catch_dx:.4f}f   // LQR'a girişte |ẋ| sınırı [m/s]",
        f"#define U_MAX              {p.u_max:.4f}f",
        f"#define RAIL_HALF_M        {p.rail_half:.4f}f",
        "",
        "// Model sabitleri (swing-up ivme → giriş dönüşümü için)",
        f"#define P_M_CART           {p.M_eff:.6f}f   // rotor ataleti dahil",
        f"#define P_M_POLE           {p.m:.6f}f",
        f"#define P_L                {p.l:.6f}f",
        f"#define P_J                {p.J:.8f}f",
        f"#define P_B                {p.b_cart:.6f}f   // motor mili viskoz sürtünmesi (B_m) dahil",
        f"#define P_C                {p.c:.6f}f",
        f"#define P_FC               {p.F_coulomb:.6f}f   // Coulomb sürtünme kuvveti T_f·N/r [N] (telafi)",
        f"#define P_G                {p.g:.4f}f",
        f"#define MOTOR_ALPHA        {p.motor.alpha:.6f}f   // N/V",
        f"#define MOTOR_BETA         {p.motor.beta:.6f}f   // N·s/m",
        f"#define MOTOR_V_MAX        {p.motor.V_max:.3f}f   // besleme gerilimi [V] (PWM ölçeği)",
        f"#define MOTOR_GEAR         {p.motor.gear:.4f}f   // redüktör oranı (enkoder sayımı → araba konumu)",
        f"#define PULLEY_R_M         {p.motor.r_pulley:.6f}f   // kasnak yarıçapı [m]",
        f"#define INPUT_MODE_{p.input_mode.upper()} 1",
    ]
    if swingup is not None:
        lines += [
            "",
            f"#define SU_K_E             {swingup.k_E:.4f}f",
            f"#define SU_A_MAX           {swingup.a_max:.4f}f",
            f"#define SU_K_X             {swingup.k_x:.4f}f",
            f"#define SU_K_V             {swingup.k_v:.4f}f",
            f"#define SU_A_KICK          {swingup.a_kick:.4f}f",
            f"#define SU_E_RATIO         {swingup.E_ratio:.4f}f   // hedef enerji oranı (1 = dik konum)",
        ]
    lines += [
        "",
        "// Ayrık kararlı hal Kalman filtresi (dik konum etrafında: s+ = Ad*s + Bd*u, s += L*(y - C*s))",
        "#define KALMAN_AVAILABLE 1",
        "static const float KALMAN_AD[4][4] = {",
        *_c_mat(Ad),
        "};",
        f"static const float KALMAN_BD[4] = {_c_vec(Bd.ravel())};",
        "static const float KALMAN_L[4][2] = {",
        *_c_mat(L),
        "};",
    ]
    return "\n".join(lines) + "\n"


def write_gains_header(path: str | Path, *args, **kw) -> Path:
    path = Path(path)
    path.write_text(gains_header(*args, **kw), encoding="utf-8")
    return path
