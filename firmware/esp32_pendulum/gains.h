// Otomatik üretildi: pendulum_lab.export  —  elle düzenlemeyin, arayüzden yeniden üretin
// 2026-11-25 21:12:42
// Q = [10.0, 1.0, 100.0, 1.0]   R = [0.1]
// Parametreler: M=1.28 m=0.105 l=0.0992 I=0.001203 b=0.1 c=0.0005 giriş=voltage T_f=0.0015077 B_m=0.0
#pragma once
// UYARI: YER TUTUCU parametrelerle üretildi — gerçek ölçüler girilip yeniden üretilmeli.
#warning "gains.h YER TUTUCU parametrelerle uretildi: olculmus degerleri girip arayuzden yeniden disa aktarin"

// θ işaret kuralı: θ > 0 sarkaç SOLA yatık (#50). Firmware bu makro yoksa derlenmez: eski (θ sağa +)
// gains.h'de K'nın θ, θ̇ kazançları ve Kalman matrisleri ters işaretlidir.
#define THETA_POSITIVE_LEFT 1
#define CTRL_TS_S          0.002000f
#define CTRL_PERIOD_MS     2
#define CTRL_RATE_HZ       500
// u = -(K0*(x - x_ref) + K1*dx + K2*th + K3*dth)
static const float LQR_K[4] = {-9.828691f, -46.641641f, 101.467600f, 14.359286f};
#define CATCH_RAD          0.261799f
#define RELEASE_RAD        0.523599f
#define CATCH_DTH          5.0000f   // LQR'a girişte |θ̇| sınırı [rad/s]
#define CATCH_X            0.4000f   // LQR'a girişte |x| sınırı [m]
#define CATCH_DX           2.0000f   // LQR'a girişte |ẋ| sınırı [m/s]
#define U_MAX              12.0000f
#define RAIL_HALF_M        0.4000f

// Model sabitleri (swing-up ivme → giriş dönüşümü için)
#define P_M_CART           2.626078f   // rotor ataleti dahil
#define P_M_POLE           0.105000f
#define P_L                0.099200f
#define P_J                0.00223627f
#define P_B                0.100000f   // motor mili viskoz sürtünmesi (B_m) dahil
#define P_C                0.000500f
#define P_FC               2.999927f   // Coulomb sürtünme kuvveti T_f·N/r [N] (telafi)
#define P_G                9.8100f
#define MOTOR_ALPHA        2.260004f   // N/V
#define MOTOR_BETA         48.409997f   // N·s/m
#define MOTOR_V_MAX        12.000f   // besleme gerilimi [V] (PWM ölçeği)
#define MOTOR_GEAR         19.0000f   // redüktör oranı (enkoder sayımı → araba konumu)
#define PULLEY_R_M         0.009549f   // kasnak yarıçapı [m]
#define INPUT_MODE_VOLTAGE 1

#define SU_K_E             100.0000f
#define SU_A_MAX           4.0000f
#define SU_K_X             10.0000f
#define SU_K_V             6.0000f
#define SU_A_KICK          2.0000f
#define SU_E_RATIO         1.0500f   // hedef enerji oranı (1 = dik konum)

// Ayrık kararlı hal Kalman filtresi (dik konum etrafında: s+ = Ad*s + Bd*u, s += L*(y - C*s))
#define KALMAN_AVAILABLE 1
static const float KALMAN_AD[4][4] = {
  {1.00000000e+00f, 1.96426529e-03f, 3.50548443e-07f, -1.48092243e-09f},
  {0.00000000e+00f, 9.64479431e-01f, 3.48426833e-04f, -1.35440145e-06f},
  {0.00000000e+00f, -1.66421453e-04f, 1.00009301e+00f, 1.99960691e-03f},
  {0.00000000e+00f, -1.65414227e-01f, 9.29902148e-02f, 9.99637978e-01f},
};
static const float KALMAN_BD[4] = {1.66482367e-06f, 1.65484700e-03f, 7.75331160e-06f, 7.70638655e-03f};
static const float KALMAN_L[4][2] = {
  {7.90797241e-01f, -2.84440161e-04f},
  {5.63916784e+01f, -4.23626194e-01f},
  {-1.29947458e-01f, 2.69104789e-01f},
  {-3.68309445e+01f, 1.98907756e+01f},
};
