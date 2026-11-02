"""Arayüz grafik bileşenleri ve enerji/mod çizim testleri (Issue #3)."""

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
