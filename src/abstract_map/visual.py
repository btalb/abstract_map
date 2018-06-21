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
_SL_NODES_FIXED_PEN = pg.mkPen(_C3)
_SL_NODES_FIXED_BRUSH = pg.mkBrush(_C3)

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
            self._plt.setAspectLocked(True, 1)
            # self._plt.hideAxis('left')
            # self._plt.hideAxis('bottom')
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

        ms_fixed = [m for m in layout._masses if m.fixed]
        if ms_fixed:
            ps_fixed = np.concatenate([m.pos for m in ms_fixed])
            pi_fixed = self._plt.plot(
                ps_fixed[::2],
                ps_fixed[1::2],
                pen=None,
                symbol='s',
                symbolPen=_SL_NODES_FIXED_PEN,
                symbolBrush=_SL_NODES_FIXED_BRUSH)
            for i, m in enumerate(ms_fixed):
                t = pg.TextItem(text=m.name, color='w', anchor=(0.5, 0))
                t.setParentItem(pg.CurvePoint(pi_fixed, i))
        ms_unfixed = [m for m in layout._masses if not m.fixed]
        if ms_unfixed:
            ps_unfixed = np.concatenate([m.pos for m in ms_unfixed])
            pi_unfixed = self._plt.plot(
                ps_unfixed[::2],
                ps_unfixed[1::2],
                pen=None,
                symbol='o',
                symbolPen=_SL_NODES_PEN,
                symbolBrush=_SL_NODES_BRUSH)
            for i, m in enumerate(ms_unfixed):
                t = pg.TextItem(text=m.name, color='w', anchor=(0.5, 0))
                t.setParentItem(pg.CurvePoint(pi_unfixed, i))

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
