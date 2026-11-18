// Otomatik üretildi: pendulum_lab.export  —  elle düzenlemeyin, arayüzden yeniden üretin
// 2026-11-18 09:45:46
// Q = [10.0, 1.0, 100.0, 1.0]   R = [0.1]
// Parametreler: M=0.5 m=0.2 l=0.3 I=0.0 b=0.1 c=0.0005 giriş=voltage T_f=0.0 B_m=0.0
#pragma once
// UYARI: YER TUTUCU parametrelerle üretildi — gerçek ölçüler girilip yeniden üretilmeli.
#warning "gains.h YER TUTUCU parametrelerle uretildi: olculmus degerleri girip arayuzden yeniden disa aktarin"

#define CTRL_TS_S          0.002000f
#define CTRL_PERIOD_MS     2
#define CTRL_RATE_HZ       500
// u = -(K0*(x - x_ref) + K1*dx + K2*th + K3*dth)
static const float LQR_K[4] = {-9.545742f, -22.272552f, -56.221807f, -8.686515f};
#define CATCH_RAD          0.261799f
#define RELEASE_RAD        0.523599f
#define U_MAX              12.0000f
#define RAIL_HALF_M        0.4000f

// Model sabitleri (swing-up ivme → giriş dönüşümü için)
#define P_M_CART           0.500000f   // rotor ataleti dahil
#define P_M_POLE           0.200000f
#define P_L                0.300000f
#define P_J                0.01800000f
#define P_B                0.100000f   // motor mili viskoz sürtünmesi (B_m) dahil
#define P_C                0.000500f
#define P_FC               0.000000f   // Coulomb sürtünme kuvveti T_f·N/r [N] (telafi)
#define P_G                9.8100f
#define MOTOR_ALPHA        3.221845f   // N/V
#define MOTOR_BETA         28.007219f   // N·s/m
#define MOTOR_V_MAX        12.000f   // besleme gerilimi [V] (PWM ölçeği)
#define MOTOR_GEAR         10.0000f   // redüktör oranı (enkoder sayımı → araba konumu)
#define PULLEY_R_M         0.012700f   // kasnak yarıçapı [m]
#define INPUT_MODE_VOLTAGE 1

#define SU_K_E             100.0000f
#define SU_A_MAX           4.0000f
#define SU_K_X             10.0000f
#define SU_K_V             6.0000f
#define SU_A_KICK          2.0000f

// Ayrık kararlı hal Kalman filtresi (dik konum etrafında: s+ = Ad*s + Bd*u, s += L*(y - C*s))
#define KALMAN_AVAILABLE 1
static const float KALMAN_AD[4][4] = {
  {1.00000000e+00f, 1.89166905e-03f, -7.56189039e-06f, 1.33548753e-09f},
  {0.00000000e+00f, 8.93661137e-01f, -7.42284820e-03f, -1.25637884e-06f},
  {0.00000000e+00f, 3.61100428e-04f, 1.00009061e+00f, 1.99998359e-03f},
  {0.00000000e+00f, 3.54460792e-01f, 9.01422908e-02f, 1.00001403e+00f},
};
static const float KALMAN_BD[4] = {1.24176475e-05f, 1.21893001e-02f, -4.13918426e-05f, -4.06307614e-02f};
static const float KALMAN_L[4][2] = {
  {5.23508705e-01f, 1.52847365e-03f},
  {2.23613369e+01f, 5.93072909e-01f},
  {1.30140686e-01f, 2.76969464e-01f},
  {3.68478539e+01f, 2.13141726e+01f},
};
