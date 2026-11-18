# Matlab → pendulum_lab Karşılıkları

Gereksinim belgesindeki "Sadece Ters Sarkaç Sisteminde Kullanılacak Matlab fonksiyonları" tablosunun projedeki karşılıkları:

| Matlab | pendulum_lab | Dayandığı kütüphane | Not |
|---|---|---|---|
| `A = [1 2; 3 4]`, `A*B`, `inv(A)`, `det(A)` | NumPy: `np.array`, `A @ B`, `np.linalg.inv`, `np.linalg.det` | NumPy | |
| `eig(A)` | `StateSpace.poles()`, `np.linalg.eigvals` | NumPy | Kompleks sonuç döner |
| `ss(A,B,C,D)` | `StateSpace(A,B,C,D)`, `StateSpace.from_params(p)` | — | `analysis/linear.py` |
| `tf(num,den)`, `ss2tf` | `StateSpace.tf(output)` | SciPy | θ/u için `output=1` |
| `ctrb`, `obsv` | `StateSpace.ctrb()`, `.obsv()`, `.is_controllable()` | NumPy | |
| `jacobian(f,v)` | `linearize(p)` (analitik), `numerical_jacobian(f, s0, u0)` | NumPy | İkisi testte karşılaştırılır |
| `lqr(A,B,Q,R)` | `lqr(A,B,Q,R)` → `(K, P, kutuplar)` | SciPy (CARE) | `control/lqr.py` |
| `c2d`, `dlqr` | `c2d(A,B,Ts)`, `dlqr(Ad,Bd,Q,R)`, `design_lqr(p,Q,R,Ts)` | SciPy (DARE) | ESP32'ye giden K budur |
| `pid(Kp,Ki,Kd)` | `PID(Kp, Ki, Kd, dt)` | — | anti-windup + filtreli türev |
| `ode45(f,t,y0)` | `simulate_open_loop(p, s0, T)` | SciPy `solve_ivp(RK45)` | |
| (Simulink kapalı çevrim) | `simulate_closed_loop(p, controller, s0, T, sensor=...)` | — | 500 Hz ZOH, sensör kuantalama, gecikme |
| `step(sys)` | `StateSpace.step(T)`, `step_info(t,y)` | SciPy | |
| `rlocus(sys)` | `rlocus(num, den, gains)` | NumPy | |
| `kalman(sys,Q,R)` | `KalmanFilter(Ad,Bd,C,Qn,Rn)` | NumPy | Opsiyonel |
| `butter(n,Wn)` + `filtfilt` | `butter_lowpass(data, fc, fs)` | SciPy | Kayıt analizi için |
| `serialport("COM3",115200)` | `SerialTransport("COM3", 921600)` + `TelemetryReader` | pySerial | Ayrı thread |
| `plot(t,y)` (gerçek zamanlı) | Arayüz → Canlı İzleme | PyQtGraph | Matplotlib bu hızda yetmez |

Fonksiyonların tümü `tests/` altında test edilmiştir. `lqr` ve `dlqr`, `python-control` kütüphanesinin sonuçlarıyla karşılaştırılır.

## Mekatronik ekibinin Matlab modeliyle karşılaştırma
Issue #1'deki ön karşılaştırmaya göre mekatronik ekibinin `cart_pendulum_lqr.m` modeli bu kütüphaneyle aynı denklemleri kullanır (Matlab dosyaları `matlab/` klasörüne eklenince doğrudan doğrulanacak). İki fark vardır:

1. **θ işareti:** Matlab'da sarkaç sola yatınca θ > 0, burada sağa (+x) yatınca. Bu yüzden K'nın θ ve θ̇ kazançları ters işaretlidir.
2. **Deneme biçimi:** Matlab denemesi sürekli zamanlı kontrolcü, kuvvet girişi (motor yok) ve ideal sensör kullanır; bu projede ESP32'ye giden kontrolcü 2 ms örneklemeli, gerilim girişlidir ve sensör kuantalaması vardır. Aynı Q, R ile bile projenin motorlu modeli için çıkan K, Matlab'ınkinden farklıdır.

Dönüşüm `pendulum_lab/sim/matlab_compat.py`'dedir (`gain_from_matlab`, `gain_to_matlab`); ayrıntı [`matlab/README.md`](../matlab/README.md).

| Matlab | pendulum_lab | Not |
|---|---|---|
| `cart_pendulum_lqr.m` (M=0.5, m=0.2, l=0.3, I=0.006, b=0.1, Q=diag(25,4,32.83,1), R=0.01) | `matlab_reference_params()`, `MATLAB_Q`, `MATLAB_R` | K = [−50, −45.37, ∓168.66, ∓33.25]; `tests/test_matlab_compat.py` denetler |
| `cart_pendulum_application.m` | `simulate_matlab_compatible(p, K, s0, T)` | Sürekli LQR, kuvvet girişi, RK4 2.5 ms, ±0.2 m / 90°'de durur. Arayüzde Analiz → **Matlab uyumlu** (varsayılan olarak Matlab'ın parametreleri ve Q, R'siyle) |

## Örnek: Matlab betiğinin Python karşılığı
```python
from pendulum_lab import load_params, StateSpace, Supervisor, simulate_closed_loop, SensorModel
from pendulum_lab.control.lqr import design_lqr
import math

p = load_params("config/params.json")
ss = StateSpace.from_params(p)                 # sys = ss(A,B,C,D)
print(ss.poles())                              # eig(A)
K, P, cl = design_lqr(p, Q=[10, 1, 100, 1], R=[[0.1]], Ts=0.002)   # c2d + dlqr
sup = Supervisor(p, K); sup.start()            # swing-up + LQR geçişi
r = simulate_closed_loop(p, sup, [0, 0, math.pi, 0], T=15, sensor=SensorModel())
print(r.mode[-1], abs(r.x).max())              # 'BALANCE', ~0.16 m
```
