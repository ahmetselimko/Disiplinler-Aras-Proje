"""Ortak arayüz bileşenleri."""

from __future__ import annotations

import math

import pyqtgraph as pg
from PyQt5 import QtGui, QtWidgets

MODE_COLORS = {
    "IDLE": "#7f8c8d", "HOMING": "#8e44ad", "WAIT": "#2980b9",
    "SWING_UP": "#e67e22", "BALANCE": "#27ae60", "FAULT": "#c0392b",
}
MODE_TR = {
    "IDLE": "BEKLEMEDE", "HOMING": "KALİBRASYON", "WAIT": "ELLE KALDIRIN",
    "SWING_UP": "SWING-UP", "BALANCE": "DENGE (LQR)", "FAULT": "HATA / STOP",
}


class PendulumView(pg.PlotWidget):
    """Araba-sarkaç çizimi. θ=0 dik."""

    def __init__(self, rail_half: float = 0.4, length: float = 0.3, parent=None):
        super().__init__(parent)
        self.setAspectLocked(True)
        self.hideButtons()
        self.setMenuEnabled(False)
        self.setMouseEnabled(False, False)
        self.getPlotItem().hideAxis("left")
        self.setLabel("bottom", "x [m]")
        self.getAxis("bottom").enableAutoSIPrefix(False)
        self.rail = self.plot(pen=pg.mkPen("#95a5a6", width=4))
        self.ends = pg.ScatterPlotItem(symbol="s", size=12, brush=pg.mkBrush("#c0392b"), pen=None)
        self.addItem(self.ends)
        self.cart = QtWidgets.QGraphicsRectItem(0, 0, 1, 1)
        self.cart.setBrush(QtGui.QBrush(QtGui.QColor("#34495e")))
        self.cart.setPen(pg.mkPen(None))
        self.addItem(self.cart)
        self.rod = self.plot(pen=pg.mkPen("#2c3e50", width=4))
        self.bob = pg.ScatterPlotItem(size=18, brush=pg.mkBrush("#e67e22"), pen=pg.mkPen("#2c3e50"))
        self.addItem(self.bob)
        self.ref = pg.InfiniteLine(angle=90, pen=pg.mkPen("#27ae60", style=2))
        self.addItem(self.ref)
        self.set_geometry(rail_half, length)
        self.set_state(0.0, math.pi)

    def set_geometry(self, rail_half: float, length: float):
        self.rail_half, self.length = rail_half, max(length, 0.1)
        self.cw, self.ch = 0.12, 0.06
        self.cart.setRect(-self.cw / 2, -self.ch / 2, self.cw, self.ch)
        self.rail.setData([-rail_half, rail_half], [0, 0])
        self.ends.setData([-rail_half, rail_half], [0, 0])
        m = 0.08
        self.setXRange(-rail_half - m, rail_half + m, padding=0)
        self.setYRange(-self.length - m, self.length + m, padding=0)

    def set_state(self, x: float, th: float, x_ref: float | None = None):
        self.cart.setPos(x, 0)
        tx, ty = x + self.length * math.sin(th), self.length * math.cos(th)
        self.rod.setData([x, tx], [0, ty])
        self.bob.setData([tx], [ty])
        if x_ref is not None:
            self.ref.setValue(x_ref)


def spin(value: float, lo: float, hi: float, decimals: int = 3, step: float | None = None,
         suffix: str = "") -> QtWidgets.QDoubleSpinBox:
    s = QtWidgets.QDoubleSpinBox()
    s.setRange(lo, hi)
    s.setDecimals(decimals)
    s.setSingleStep(step if step is not None else 10 ** -max(0, decimals - 1))
    if suffix:
        s.setSuffix(" " + suffix)
    s.setValue(value)
    s.setKeyboardTracking(False)
    return s


def mono_font() -> QtGui.QFont:
    f = QtGui.QFont("Consolas")
    f.setStyleHint(QtGui.QFont.Monospace)
    f.setPointSize(9)
    return f
