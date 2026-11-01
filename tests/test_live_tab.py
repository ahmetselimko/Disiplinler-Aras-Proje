"""Canlı İzleme sekmesi: simülasyona özel butonlar ve "Durduruldu" gösterimi (ekransız Qt)."""

import os
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
QtWidgets = pytest.importorskip("PyQt5.QtWidgets")

from gui.live_tab import LiveTab  # noqa: E402
from gui.state import AppState  # noqa: E402

PARAMS = Path(__file__).resolve().parents[1] / "config" / "params.json"


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.fixture
def tab(app):
    return LiveTab(AppState(PARAMS))


def test_sim_only_buttons_disabled_without_simulator(tab):
    for w in (tab.b_kick, tab.b_pause, tab.b_reset, tab.th0):
        assert not w.isEnabled()


def test_idle_after_stop_shown_as_stopped(tab):
    tab._show_mode("IDLE")
    assert tab.mode_lbl.text() == "BEKLEMEDE"
    tab._stopped = True
    tab._show_mode("IDLE")
    assert tab.mode_lbl.text() == "DURDURULDU"
    tab._show_mode("FAULT")                     # diğer modlar değişmez
    assert tab.mode_lbl.text() == "HATA / STOP"


def test_pause_button_reverts_when_command_cannot_be_sent(tab):
    tab.b_pause.setChecked(True)                # bağlı değil → gönderilemez
    assert not tab.b_pause.isChecked() and tab.b_pause.text() == "Duraklat"


def test_paused_mode_label_marked(tab):
    tab._set_paused(True)
    tab._show_mode("BALANCE")
    assert tab.mode_lbl.text() == "⏸ DENGE (LQR)" and tab.b_pause.text() == "Devam Et"
