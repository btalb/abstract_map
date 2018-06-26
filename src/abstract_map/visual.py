from __future__ import absolute_import
import abc
from enum import Enum
import multiprocessing as mp
import pdb
import numpy as np
import operator
import os
import sys
import threading
import time
import warnings

import pyqtgraph as pg
from pyqtgraph.Qt import QtGui

import abstract_map.spatial_layout as sl
import abstract_map.tools as tools

warnings.filterwarnings('ignore',
                        'Treat the new Tool classes introduced in v1.5.*')
warnings.filterwarnings('ignore', '.*GUI is implemented')

# Abstract class compatibility across python 2 and python 3
ABC = abc.ABCMeta('ABC', (object,), {'__slots__': ()})

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
    """Class containing all supported visualisation methods"""

    def __init__(self, window_type=WindowType.DEFAULT):
        """Constructs a visualiser which controls visualisation rate"""
        self._win_type = window_type
        self._objs = []  # Tuple of (obj, visual_handles, is_up_to_date)

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

        self._plt.setTitle("t = %f" % (layout._energy_log.t[-1]))

    def close(self):
        """Closes the visualiser"""
        self._win.close()

    def visualise(self, obj):
        """Callback for visualising an object"""
        # Determine what visualisation function to call
        if type(obj) is sl.SpatialLayout:
            fn = self._visualiseSpatialLayout
        elif type(obj) is sl.EnergyLog:
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


class _Controller(ABC):
    """Class to handle interfacing with an underlying visualiser"""

    def __init__(self, rate=10):
        """Super constructor for setting up the basic controller mechanics"""
        self._delay = 1.0 / rate
        self._last_time = 0
        self.key_handler = None

    @abc.abstractmethod
    def close(self):
        """Closes the visualiser"""
        pass

    @tools.abstractstatic
    def create(window_type=WindowType.DEFAULT, rate=10):
        """Generic function for creating a controller from basic info"""
        pass

    def processKey(self, key):
        """Passes a key event to the key handler if it is attached"""
        if self.key_handler is not None:
            self.key_handler(key)

    def update(self, obj):
        """Method where the controller is asked to update the visualiser"""
        if not self.updateDue():
            return
        self._update_method(obj)
        self._last_time = time.time()

    def updateDue(self):
        """Checks whether an update of the visualiser is due"""
        return time.time() > self._last_time + self._delay


class BasicController(_Controller):
    """Class for the basic, same thread, interface to a visualiser"""

    def __init__(self, visualiser, rate=10):
        """Constructor which attaches to a specified visualiser"""
        super(BasicController, self).__init__(rate)
        self._visualiser = visualiser
        self._update_method = visualiser.visualise
        self._visualiser._win.keyPressEvent = (
            lambda ev: self.processKey(ev.key()))

    def close(self):
        """Closes the visualiser"""
        self._visualiser.close()

    @staticmethod
    def create(window_type=WindowType.DEFAULT, rate=10):
        """Function for creating a basic controller from basic info"""
        return BasicController(Visualiser(window_type=window_type))


class AsyncController(_Controller):
    """Class for interfacing with a visualiser on another thread"""

    def __init__(self, pipe, rate=10):
        """Constructor for attaching the controller to a specified pipe"""
        super(AsyncController, self).__init__(rate)
        self._pipe = pipe
        self._pipe_thread = threading.Thread(target=self._pipe_monitor)
        self._update_method = self._pipe.send
        self._is_closed = False

        self._pipe_thread.start()

    def _pipe_monitor(self):
        """Monitors the pipe for any received data"""
        while self._pipe_thread.isAlive() and not self._is_closed:
            if self._pipe.poll():
                recv_obj = self._pipe.recv()
                self.processKey(recv_obj)

    def close(self):
        """Closes the visualiser"""
        self._pipe.send("close")
        self._is_closed = True

    @staticmethod
    def create(window_type=WindowType.DEFAULT, rate=10):
        """Function for creating an async controller from basic info"""
        pipe_target, pipe_controller = mp.Pipe(duplex=True)
        controller = AsyncController(pipe_controller)
        p = mp.Process(target=asyncTarget, args=(pipe_target, window_type))
        p.start()
        return controller


def asyncTarget(pipe, window_type=WindowType.DEFAULT):
    """The target of async requests, where an async visualiser is run"""
    v = Visualiser(window_type=window_type)
    v._win.keyPressEvent = lambda ev: pipe.send(ev.key())
    quit = False
    while not quit:
        if pipe.poll():
            recv_obj = pipe.recv()
            if recv_obj == "close":
                quit = True
            else:
                v.visualise(recv_obj)
        time.sleep(0.1)
    v.close()
