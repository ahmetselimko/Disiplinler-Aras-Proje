// Otomatik üretildi: pendulum_lab.export  —  elle düzenlemeyin, arayüzden yeniden üretin
// 2026-10-15 19:12:42
// Q = [10, 1, 100, 1]   R = [0.1]
// Parametreler: M=0.5 m=0.2 l=0.3 I=0.0 b=0.1 c=0.0005 giriş=voltage
#pragma once

#define CTRL_TS_S          0.002000f
#define CTRL_PERIOD_MS     2
#define CTRL_RATE_HZ       500
// u = -(K0*(x - x_ref) + K1*dx + K2*th + K3*dth)
static const float LQR_K[4] = {-9.568725f, -22.128074f, -56.441844f, -8.714121f};
#define CATCH_RAD          0.261799f
#define RELEASE_RAD        0.523599f
#define U_MAX              12.0000f
#define RAIL_HALF_M        0.4000f

// Model sabitleri (swing-up ivme → giriş dönüşümü için)
#define P_M_CART           0.500000f   // rotor ataleti dahil
#define P_M_POLE           0.200000f
#define P_L                0.300000f
#define P_J                0.01800000f
#define P_B                0.100000f
#define P_C                0.000500f
#define P_G                9.8100f
#define MOTOR_ALPHA        2.970651f   // N/V
#define MOTOR_BETA         25.496142f   // N·s/m
#define MOTOR_V_MAX        12.000f   // besleme gerilimi [V] (PWM ölçeği)
#define INPUT_MODE_VOLTAGE 1

#define SU_K_E             100.0000f
#define SU_A_MAX           4.0000f
#define SU_K_X             10.0000f
#define SU_K_V             6.0000f
#define SU_A_KICK          2.0000f
