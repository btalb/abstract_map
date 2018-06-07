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

# pyplotgraph global configuration settings
pg.setConfigOptions(background='w', foreground='k', antialias=True)


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
        self._props = []

        # Setup the window, and save some handles to it
        self._configureWindow()

    def _applyProps(self):
        """Applies the list of properties to the axes"""
        # for p, v in self._props:
        #     if p == 'title':
        #         self._ax.set_title(v)
        #     elif p == 'xlabel':
        #         self._ax.set_xlabel(v)
        #     elif p == 'ylabel':
        #         self._ax.set_ylabel(v)
        #     else:
        #         raise ValueError(
        #             "property with name '%s' is no supported" % (p))
        pass

    def _configureWindow(self):
        """Configures the window based on the selected window type"""
        self._win = pg.plot(title="Abstact Map Visualisation")
        self._plt = self._win.plotItem
        if self._win_type == WindowType.IMMERSIVE:
            pass
        else:  # DEFAULT
            pass
        pass

    def _visualiseEnergyLog(self, energy_log):
        """Visualises the energy log of a spatial layout"""
        self._plt.plot(
            energy_log.t,
            list(map(operator.add, energy_log.kinetic, energy_log.potential)))
        self._plt.plot(energy_log.t, energy_log.kinetic)
        self._plt.plot(energy_log.t, energy_log.potential)

    def _visualiseSpatialLayout(self, layout):
        """Visualises a spatial layout"""
        for c in layout._constraints:
            ps = np.concatenate([m.pos for m in c.masses()], 1)
            self._plt.plot(ps[0, :], ps[1, :])

        ps = np.concatenate([m.pos for m in layout._masses], 1)
        self._plt.plot(ps[0, :], ps[1, :], pen=None, symbol='o')
        # for m in layout._masses:
        #     self._ax.annotate(m.name, m.pos)

    def addProp(self, name, value):
        """Adds a persistent property to the visualiser"""
        self._props.append((name, value))

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
        # ta = time.time()
        self._plt.clear()
        self._applyProps()
        fn(obj)
        QtGui.QGuiApplication.processEvents()
        # plt.pause(delay)
        self._last_time = time.time()
        # print("\tVis took: %fms (%s)" % (1000 * (self._last_time - ta), obj))

    def waitForPress(self):
        """Waits until a key is pressed while focus is on the axis"""
        self._fig.waitforbuttonpress()
