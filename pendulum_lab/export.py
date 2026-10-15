"""Tasarlanan kazançları ESP32 firmware'ine gömülecek C başlık dosyasına (gains.h) aktarır.

Böylece kart PC'ye bağlı olmadan da son tasarlanan K ile açılır; PC bağlanınca set_gain ile
çalışma anında değiştirilebilir.
"""

from __future__ import annotations

import math
import time
from pathlib import Path

import numpy as np


def gains_header(p, K, Ts: float, Q=None, R=None, swingup=None, catch_deg: float = 15.0,
                 release_deg: float = 30.0) -> str:
    K = np.asarray(K, dtype=float).ravel()
    period_ms = Ts * 1000
    if abs(period_ms - round(period_ms)) > 1e-9 or round(period_ms) < 1:
        # firmware vTaskDelayUntil ile 1 ms'lik FreeRTOS tikinde çalışır
        raise ValueError(f"Ts tam sayı milisaniye olmalı (≥ 1 ms), verilen {period_ms:g} ms")
    lines = [
        "// Otomatik üretildi: pendulum_lab.export  —  elle düzenlemeyin, arayüzden yeniden üretin",
        f"// {time.strftime('%Y-%m-%d %H:%M:%S')}",
        f"// Q = {list(np.ravel(Q)) if Q is not None else '-'}   R = {np.ravel(R).tolist() if R is not None else '-'}",
        f"// Parametreler: M={p.M} m={p.m} l={p.l} I={p.I} b={p.b} c={p.c} giriş={p.input_mode}",
        "#pragma once",
        "",
        f"#define CTRL_TS_S          {Ts:.6f}f",
        f"#define CTRL_PERIOD_MS     {round(period_ms)}",
        f"#define CTRL_RATE_HZ       {round(1 / Ts)}",
        "// u = -(K0*(x - x_ref) + K1*dx + K2*th + K3*dth)",
        "static const float LQR_K[4] = {" + ", ".join(f"{k:.6f}f" for k in K) + "};",
        f"#define CATCH_RAD          {math.radians(catch_deg):.6f}f",
        f"#define RELEASE_RAD        {math.radians(release_deg):.6f}f",
        f"#define U_MAX              {p.u_max:.4f}f",
        f"#define RAIL_HALF_M        {p.rail_half:.4f}f",
        "",
        "// Model sabitleri (swing-up ivme → giriş dönüşümü için)",
        f"#define P_M_CART           {p.M_eff:.6f}f   // rotor ataleti dahil",
        f"#define P_M_POLE           {p.m:.6f}f",
        f"#define P_L                {p.l:.6f}f",
        f"#define P_J                {p.J:.8f}f",
        f"#define P_B                {p.b:.6f}f",
        f"#define P_C                {p.c:.6f}f",
        f"#define P_G                {p.g:.4f}f",
        f"#define MOTOR_ALPHA        {p.motor.alpha:.6f}f   // N/V",
        f"#define MOTOR_BETA         {p.motor.beta:.6f}f   // N·s/m",
        f"#define MOTOR_V_MAX        {p.motor.V_max:.3f}f   // besleme gerilimi [V] (PWM ölçeği)",
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
        ]
    return "\n".join(lines) + "\n"


def write_gains_header(path: str | Path, *args, **kw) -> Path:
    path = Path(path)
    path.write_text(gains_header(*args, **kw), encoding="utf-8")
    return path
