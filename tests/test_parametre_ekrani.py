"""Parametre ekranındaki yeni alanlar (Issue #5): ẋ₀, θ̇₀, dürtme süresi ve g."""

import math
import os

import numpy as np
import pytest
from PyQt5 import QtWidgets

from gui.analysis_tab import SimulationPage, pulse_disturbance
from gui.design_tab import DesignTab
from gui.state import AppState


@pytest.fixture(scope="module")
def qapp():
    os.environ["QT_QPA_PLATFORM"] = "offscreen"
    app = QtWidgets.QApplication.instance()
    if app is None:
        app = QtWidgets.QApplication(["test"])
    yield app


def test_pulse_disturbance_duration():
    assert pulse_disturbance(1.0, 0.0, 0.05) is None          # kuvvet yoksa bozucu da yok
    d = pulse_disturbance(1.0, 5.0, 0.2)
    assert d(0.99) == 0.0
    assert d(1.0) == 5.0
    assert d(1.19) == 5.0                                      # 50 ms sınırı artık sabit değil
    assert d(1.2) == 0.0


def test_simulation_uses_initial_velocities(qapp):
    page = SimulationPage(AppState("config/params.json"))
    page.T.setValue(0.5)
    page.real_cb.setChecked(False)
    page.dx0.setValue(0.3)
    page.dth0.setValue(90.0)
    page.run()
    s0 = page.result.s[0]
    assert s0[1] == pytest.approx(0.3)
    assert s0[3] == pytest.approx(math.radians(90.0))


def test_longer_disturbance_pushes_cart_further(qapp):
    def final_x(duration_ms):
        page = SimulationPage(AppState("config/params.json"))
        page.th0.setValue(0.0)                                 # dik başla, LQR dengede tutsun
        page.T.setValue(1.0)
        page.real_cb.setChecked(False)
        page.dist_t.setValue(0.1)
        page.dist_F.setValue(2.0)
        page.dist_dur.setValue(duration_ms)
        page.run()
        return np.max(np.abs(page.result.x))

    assert final_x(200) > final_x(20)


def test_design_tab_g_roundtrip(qapp):
    st = AppState("config/params.json")
    tab = DesignTab(st)
    assert tab.w_par["g"].value() == pytest.approx(st.p.g)
    tab.w_par["g"].setValue(3.71)                              # Mars :)
    assert tab._params_from_widgets().g == pytest.approx(3.71)
