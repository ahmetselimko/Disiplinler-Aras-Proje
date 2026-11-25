// Otomatik üretildi: pendulum_lab.export  —  elle düzenlemeyin, arayüzden yeniden üretin
// 2026-11-25 19:30:31
// Q = [10.0, 1.0, 100.0, 1.0]   R = [0.1]
// Parametreler: M=0.5 m=0.2 l=0.3 I=0.0 b=0.1 c=0.0005 giriş=voltage T_f=0.0 B_m=0.0
#pragma once
// UYARI: YER TUTUCU parametrelerle üretildi — gerçek ölçüler girilip yeniden üretilmeli.
#warning "gains.h YER TUTUCU parametrelerle uretildi: olculmus degerleri girip arayuzden yeniden disa aktarin"

#define CTRL_TS_S          0.002000f
#define CTRL_PERIOD_MS     2
#define CTRL_RATE_HZ       500
// u = -(K0*(x - x_ref) + K1*dx + K2*th + K3*dth)
static const float LQR_K[4] = {-9.515838f, -27.286601f, 61.490548f, 9.737244f};
#define CATCH_RAD          0.261799f
#define RELEASE_RAD        0.523599f
#define CATCH_DTH          5.0000f   // LQR'a girişte |θ̇| sınırı [rad/s]
#define CATCH_X            0.4000f   // LQR'a girişte |x| sınırı [m]
#define CATCH_DX           2.0000f   // LQR'a girişte |ẋ| sınırı [m/s]
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
#define MOTOR_ALPHA        4.284996f   // N/V
#define MOTOR_BETA         49.540644f   // N·s/m
#define MOTOR_V_MAX        12.000f   // besleme gerilimi [V] (PWM ölçeği)
#define MOTOR_GEAR         10.0000f   // redüktör oranı (enkoder sayımı → araba konumu)
#define PULLEY_R_M         0.009549f   // kasnak yarıçapı [m]
#define INPUT_MODE_VOLTAGE 1

#define SU_K_E             100.0000f
#define SU_A_MAX           4.0000f
#define SU_K_X             10.0000f
#define SU_K_V             6.0000f
#define SU_A_KICK          2.0000f
#define SU_E_RATIO         1.0000f   // hedef enerji oranı (1 = dik konum)

// Ayrık kararlı hal Kalman filtresi (dik konum etrafında: s+ = Ad*s + Bd*u, s += L*(y - C*s))
#define KALMAN_AVAILABLE 1
static const float KALMAN_AD[4][4] = {
  {1.00000000e+00f, 1.81395300e-03f, 7.35327737e-06f, -1.26420140e-09f},
  {0.00000000e+00f, 8.19908828e-01f, 7.11789370e-03f, 1.30681653e-06f},
  {0.00000000e+00f, -6.20151926e-04f, 1.00008991e+00f, 1.99998383e-03f},
  {0.00000000e+00f, -6.00300420e-01f, 8.91257836e-02f, 1.00001420e+00f},
};
static const float KALMAN_BD[4] = {1.60596371e-05f, 1.55455283e-02f, 5.35317151e-05f, 5.18181266e-02f};
static const float KALMAN_L[4][2] = {
  {6.00114924e-01f, -1.00930496e-03f},
  {1.81326672e+01f, -5.37426724e-01f},
  {-1.52008964e-01f, 2.86539414e-01f},
  {-4.84340427e+01f, 2.30280969e+01f},
};
