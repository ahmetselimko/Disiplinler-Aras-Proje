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
