"""Arayüz grafik bileşenleri ve enerji/mod çizim testleri (Issue #3)."""

import math
import os

import numpy as np
import pytest
from PyQt5 import QtWidgets

from gui.analysis_tab import SimulationPage
from gui.live_tab import LiveTab
from gui.state import AppState
from pendulum_lab.comm.reader import RingBuffer
from pendulum_lab.comm.reader import TelemetryReader
from pendulum_lab.comm.protocol import encode_telemetry
from pendulum_lab.model.dynamics import energy


@pytest.fixture(scope="module")
def qapp():
    os.environ["QT_QPA_PLATFORM"] = "offscreen"
    app = QtWidgets.QApplication.instance()
    if app is None:
        app = QtWidgets.QApplication(["test"])
    yield app


def test_gui_plots_instantiation(qapp):
    st = AppState("config/params.json")
    live = LiveTab(st)
    sim = SimulationPage(st)

    # Canlı İzleme sekmelerinde p_e (enerji) ve p_m (mod) grafikleri mevcut mu?
    assert hasattr(live, "p_e")
    assert hasattr(live, "p_m")
    assert hasattr(live, "c_e")
    assert hasattr(live, "c_m")

    # Analiz sekmesinde p_e (enerji) ve p_m (mod) grafikleri mevcut mu?
    assert hasattr(sim, "p_e")
    assert hasattr(sim, "p_m")
    assert hasattr(sim, "c_e")
    assert hasattr(sim, "c_m")


def test_live_tab_refresh_with_energy_and_mode(qapp):
    st = AppState("config/params.json")
    live = LiveTab(st)

    class DummyReader:
        rate_hz = 500.0
        last = None
        buffer = RingBuffer(100, 7)

    reader = DummyReader()
    # Satır veri ekle: t, x, dx, th, dth, u, mode
    rows = []
    for i in range(10):
        t = i * 0.01
        rows.append([t, 0.0, 0.0, 0.1, 0.5, 2.0, 4.0])
    reader.buffer.extend(rows)

    # Refresh çağrıldığında enerji ve mod eğrileri hatasız güncellenmeli
    live.refresh(reader)
    x_data, y_data = live.c_e.getData()
    assert len(x_data) > 0
    assert len(y_data) > 0
    expected = energy(st.p, .1, .5)
    np.testing.assert_allclose(y_data, expected)
    _, mode_data = live.c_m.getData()
    np.testing.assert_array_equal(mode_data, 4.)


def test_simulation_run_populates_energy_summary_and_playback(qapp):
    st = AppState("config/params.json")
    page = SimulationPage(st)
    page.T.setValue(.5)
    page.mismatch.setValue(20.)
    page.run()
    r = page.result
    _, actual = page.c_e.getData()
    import copy
    real = copy.deepcopy(st.p)
    real.m *= 1.2
    real.l *= 1.2
    expected = [energy(real, th, dth) for th, dth in zip(r.th, r.s[:, 3])]
    np.testing.assert_allclose(actual, expected)
    assert page.slider.maximum() == len(r.t) - 1
    assert "İlk dengeleme zamanı" in page.info.text()
    assert len(page.cursor) == 5
    page.close()


def test_simulation_matlab_compatible_mode(qapp):
    """'Matlab uyumlu' seçeneği: kuvvet girişli sürekli LQR, Matlab işaretli K bilgisi, ±0.2 m / 90° durma."""
    st = AppState("config/params.json")
    page = SimulationPage(st)
    page.matlab_cb.setChecked(True)
    assert abs(page.th0.value()) < 90            # 180° varsayılanı Matlab sınırının içine alındı
    assert not page.swing_cb.isEnabled() and not page.real_cb.isEnabled()
    page.T.setValue(2.0)
    page.run()
    r = page.result
    assert r is not None and r.stop_reason in ("time", "rail", "angle")
    assert np.diff(r.t) == pytest.approx(0.0025)
    text = page.info.text()
    assert "Matlab uyumlu deneme" in text and "K (θ sola +, Matlab ile aynı işaret)" in text
    assert page.slider.maximum() == len(r.t) - 1
    page.matlab_cb.setChecked(False)
    assert page.swing_cb.isEnabled()
    page.close()


def test_matlab_mode_uses_initial_velocities_and_disables_unused_options(qapp):
    """Matlab modunda ẋ₀ ve θ̇₀ simülasyona girmeli; kullanılmayan ayarlar (Kalman, dürtme süresi) kapanmalı."""
    st = AppState("config/params.json")
    page = SimulationPage(st)
    page.matlab_cb.setChecked(True)
    assert not page.kalman_cb.isEnabled() and not page.dist_at.isEnabled()
    assert page.dist_dur.isEnabled() and page.dist_at.currentData() == "cart"   # Matlab'daki "Apply push"
    assert page.dx0.isEnabled() and page.dth0.isEnabled()
    page.th0.setValue(10.0)
    page.T.setValue(0.5)
    page.dx0.setValue(0.1)
    page.dth0.setValue(20.0)
    page.run()
    assert page.result.s[0] == pytest.approx([0.0, 0.1, math.radians(10.0), math.radians(20.0)])
    page.matlab_cb.setChecked(False)
    assert page.kalman_cb.isEnabled() and page.dist_at.isEnabled()
    page.close()


def test_matlab_mode_reproduces_matlab_K_with_matlab_settings(qapp):
    """Matlab ayarları seçiliyken arayüz Matlab'ın K'sını vermeli; kapalıyken proje ayarlarını kullandığını söylemeli."""
    from pendulum_lab.sim.matlab_compat import MATLAB_K_REFERENCE

    st = AppState("config/params.json")
    page = SimulationPage(st)
    assert not page.matlab_ref_cb.isEnabled()
    page.matlab_cb.setChecked(True)
    assert page.matlab_ref_cb.isEnabled() and page.matlab_ref_cb.isChecked()
    page.th0.setValue(10.0)
    page.T.setValue(0.5)
    page.run()
    line = next(s for s in page.info.text().splitlines() if s.startswith("K (θ sola +"))
    K_shown = [float(v) for v in line.split("[")[1].rstrip("]").split(",")]
    assert K_shown == pytest.approx(MATLAB_K_REFERENCE, abs=0.01)
    assert page.result.s[0][2] == pytest.approx(math.radians(10.0))
    page.matlab_ref_cb.setChecked(False)
    page.run()
    text = page.info.text()
    assert "Matlab'ın K'sı DEĞİL" in text
    line = next(s for s in text.splitlines() if s.startswith("K (θ sola +"))
    K_proj = [float(v) for v in line.split("[")[1].rstrip("]").split(",")]
    assert K_proj != pytest.approx(MATLAB_K_REFERENCE, abs=0.5)
    page.close()


def test_live_summary_collects_all_samples_without_drawing_and_while_frozen(qapp):
    st = AppState("config/params.json")
    live = LiveTab(st)
    from types import SimpleNamespace
    reader = TelemetryReader(SimpleNamespace())
    st.set_reader(reader)
    live.pause_cb.setChecked(True)
    lines = [encode_telemetry(i, i*.002, .36 if i == 25 else 0., 0., 0., 0.,
                              12. if i == 25 else 0., "BALANCE") for i in range(1001)]
    lines.append(encode_telemetry(1001, 2.002, .38, 0., .2, 0., 0., "FAULT"))
    reader._handle_lines(lines)
    live.refresh(reader)
    s, active = reader.trials.snapshot()
    assert s.samples == 1002 and not active
    assert s.sat_time == pytest.approx(.002)
    assert "FAULT" in live.summary_lbl.text()
    assert "0.380" in live.summary_lbl.text()
    reader.trials.finish()
    st.set_reader(None)
    assert "FAULT" in live.summary_lbl.text()
    live.close()


def test_mode_axis_lists_every_mode(qapp):
    """Mod grafiğinin ekseni tüm modları (FRICTION dahil) göstermeli."""
    import pyqtgraph as pg
    from gui.widgets import setup_mode_axis
    from pendulum_lab.control.supervisor import MODE_CODES
    plot = pg.PlotWidget()
    setup_mode_axis(plot.getPlotItem())
    ticks = dict(plot.getPlotItem().getAxis("left")._tickLevels[0])
    assert set(ticks) == set(MODE_CODES.values()) and ticks[MODE_CODES[max(MODE_CODES, key=MODE_CODES.get)]] == "SÜRT"
    assert plot.getPlotItem().getViewBox().viewRange()[1][1] >= max(MODE_CODES.values())
