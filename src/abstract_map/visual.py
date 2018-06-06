from __future__ import absolute_import
from enum import Enum
import operator
import time
import sys
import warnings

import matplotlib as mpl
import matplotlib.pyplot as plt

import abstract_map.spatial_layout as sl

warnings.filterwarnings('ignore',
                        'Treat the new Tool classes introduced in v1.5.*')
warnings.filterwarnings('ignore', '.*GUI is implemented')


class FigureType(Enum):
    """Enum for representing a figure type request"""
    DEFAULT = 0
    IMMERSIVE = 1


class Visualiser(object):
    PAUSE = 1e-6

    def __init__(self, figure_type=FigureType.DEFAULT, rate=10):
        """Constructs a visualiser which controls visualisation rate"""
        self._fig_type = figure_type
        self._delay = 1.0 / rate
        self._last_time = 0
        self._props = []

        # Setup the figure, and save some handles to it
        self._configureFigure()

    def _applyProps(self):
        """Applies the list of properties to the axes"""
        for p, v in self._props:
            if p == 'title':
                self._ax.set_title(v)
            elif p == 'xlabel':
                self._ax.set_xlabel(v)
            elif p == 'ylabel':
                self._ax.set_ylabel(v)
            else:
                raise ValueError(
                    "property with name '%s' is no supported" % (p))

    def _configureFigure(self):
        """Configures the figure based on the selected figure type"""
        if self._fig_type == FigureType.IMMERSIVE:
            mpl.rcParams['toolbar'] = 'None'
            self._fig = plt.figure()
            self._ax = self._fig.add_axes([0, 0, 1, 1])
            self._ax.tick_params(direction='in', pad=-22)
            self._ax.set_aspect('equal', 'datalim')
        else:  # DEFAULT
            self._fig = plt.figure()
            self._ax = self._fig.add_subplot(1, 1, 1)
            self._ax.set_aspect('auto')
        pass

    def _visualiseEnergyLog(self, energy_log):
        """Visualises the energy log of a spatial layout"""
        self._ax.set_prop_cycle(None)
        self._ax.plot(
            energy_log.t,
            map(operator.add, energy_log.kinetic, energy_log.potential))
        self._ax.plot(energy_log.t, energy_log.kinetic)
        self._ax.plot(energy_log.t, energy_log.potential)

    def _visualiseSpatialLayout(self, layout):
        """Visualises a spatial layout"""
        for c in layout._constraints:
            ms = c.masses()
            self._ax.set_prop_cycle(None)
            self._ax.plot([ms[0].pos[0], ms[1].pos[0]],
                          [ms[0].pos[1], ms[1].pos[1]])
            if len(ms) > 2:
                self._ax.set_prop_cycle(None)
                self._ax.plot([ms[2].pos[0], ms[1].pos[0]],
                              [ms[2].pos[1], ms[1].pos[1]])

        self._ax.set_prop_cycle(None)
        self._ax.scatter([m.pos[0] for m in layout._masses],
                         [m.pos[1] for m in layout._masses])
        for m in layout._masses:
            self._ax.annotate(m.name, m.pos)

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
        ta = time.time()
        self._ax.clear()
        self._applyProps()
        fn(obj)
        plt.pause(delay)
        self._last_time = time.time()
        sys.stdout.write("deltaVis = %fms\t" % (1000 * (self._last_time - ta)))
        sys.stdout.flush()

    def waitForPress(self):
        """Waits until a key is pressed while focus is on the axis"""
        self._fig.waitforbuttonpress()
