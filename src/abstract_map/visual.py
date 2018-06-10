from __future__ import absolute_import
from enum import Enum
import pdb
import numpy as np
import operator
import sys
import time
import warnings

import pyqtgraph as pg
from pyqtgraph.Qt import QtGui

import abstract_map.spatial_layout as sl

warnings.filterwarnings('ignore',
                        'Treat the new Tool classes introduced in v1.5.*')
warnings.filterwarnings('ignore', '.*GUI is implemented')

# Pens used in visualisations (only initialise once)
_C1 = '#1f77b4'
_C2 = '#ff7f0e'
_C3 = '#2ca02c'

_SL_LINES_PEN = pg.mkPen(_C1, width=2)
_SL_NODES_PEN = pg.mkPen(_C1)
_SL_NODES_BRUSH = pg.mkBrush(_C1)

_EL_TOTAL_PEN = pg.mkPen(_C1)
_EL_KINETIC_PEN = pg.mkPen(_C2)
_EL_POTENTIAL_PEN = pg.mkPen(_C3)

# pyplotgraph global configuration settings
pg.setConfigOptions(antialias=True)


class WindowType(Enum):
    """Enum for representing a window type request"""
    DEFAULT = 0
    IMMERSIVE = 1


class Visualiser(object):
    PAUSE = 1e-6

    def __init__(self, window_type=WindowType.DEFAULT, rate=10):
        """Constructs a visualiser which controls visualisation rate"""
        self._win_type = window_type
        self._delay = 1.0 / rate
        self._last_time = 0

        # Setup the window, and save some handles to it
        self._configureWindow()

    def _configureWindow(self):
        """Configures the window based on the selected window type"""
        if self._win_type == WindowType.IMMERSIVE:
            pg.setConfigOptions(foreground='d', background='k')
            self._win = pg.plot(title="Abstact Map Visualisation")
            self._plt = self._win.plotItem
            self._plt.hideAxis('left')
            self._plt.hideAxis('bottom')
        else:  # DEFAULT
            pg.setConfigOptions(foreground='k', background='w')
            self._win = pg.plot(title="Abstact Map Visualisation")
            self._plt = self._win.plotItem

    def _visualiseEnergyLog(self, energy_log):
        """Visualises the energy log of a spatial layout"""
        self._plt.plot(
            energy_log.t,
            list(map(operator.add, energy_log.kinetic, energy_log.potential)),
            pen=_EL_TOTAL_PEN)
        self._plt.plot(energy_log.t, energy_log.kinetic, pen=_EL_KINETIC_PEN)
        self._plt.plot(
            energy_log.t, energy_log.potential, pen=_EL_POTENTIAL_PEN)

        # Perform any first plot initialisation that may be necessary
        if self._plt.legend is None:
            li = self._plt.addLegend(offset=(-30, 30))
            li.addItem(self._plt.items[0], 'Total')
            li.addItem(self._plt.items[1], 'Kinetic')
            li.addItem(self._plt.items[2], 'Potential')
            self._plt.setLabels(
                title="<b>Spatial Layout Energy Tracking</b>",
                left='Energy (J)',
                bottom='System time (s)')

    def _visualiseSpatialLayout(self, layout):
        """Visualises a spatial layout"""
        for c in layout._constraints:
            ps = np.concatenate([m.pos for m in c.masses()])
            self._plt.plot(ps[::2], ps[1::2], pen=_SL_LINES_PEN)

        ps = np.concatenate([m.pos for m in layout._masses])
        pi = self._plt.plot(
            ps[::2],
            ps[1::2],
            pen=None,
            symbol='o',
            symbolPen=_SL_NODES_PEN,
            symbolBrush=_SL_NODES_BRUSH)
        for i, m in enumerate(layout._masses):
            t = pg.TextItem(text=m.name, color='w', anchor=(0.5, 0))
            t.setParentItem(pg.CurvePoint(pi, i))

        layout._log['e'].append(len(self._plt.items))

    def visualise(self, obj, delay=PAUSE):
        """Callback for visualising an object, if ready to visualise"""
        # Bail if the time has not elapsed
        if time.time() < self._last_time + self._delay:
            return

        # Determine what visualisation function to call
        if isinstance(obj, sl.SpatialLayout):
            fn = self._visualiseSpatialLayout
        elif isinstance(obj, sl.EnergyLog):
            fn = self._visualiseEnergyLog
        else:
            raise TypeError(
                "visualise() cannot visualise an object of class: %s" %
                (type(obj).__name__))

        # Perform the visualisation (and update the time)
        self._plt.clear()
        fn(obj)
        QtGui.QGuiApplication.processEvents()
        self._last_time = time.time()

    def waitForPress(self):
        """Waits until a key is pressed while focus is on the axis"""
        self._fig.waitforbuttonpress()
