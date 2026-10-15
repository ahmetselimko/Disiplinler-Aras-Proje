"""pendulum_lab — Ters sarkaç projesi için modelleme, kontrol, simülasyon ve haberleşme kütüphanesi.

Matlab karşılıkları için bkz. docs/matlab_karsiliklari.md
"""

from .model.params import PendulumParams, MotorParams, load_params, save_params
from .model.dynamics import dynamics, accel_to_input
from .model.linearize import linearize, numerical_jacobian
from .control.lqr import lqr, dlqr, c2d
from .control.swingup import EnergySwingUp
from .control.supervisor import Supervisor, Mode
from .control.pid import PID
from .analysis.linear import StateSpace, rlocus, step_info
from .sim.simulator import simulate_open_loop, simulate_closed_loop, SensorModel

__all__ = [
    "PendulumParams", "MotorParams", "load_params", "save_params",
    "dynamics", "accel_to_input", "linearize", "numerical_jacobian",
    "lqr", "dlqr", "c2d", "EnergySwingUp", "Supervisor", "Mode", "PID",
    "StateSpace", "rlocus", "step_info",
    "simulate_open_loop", "simulate_closed_loop", "SensorModel",
]
