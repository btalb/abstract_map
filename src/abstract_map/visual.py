from __future__ import absolute_import
import time
import warnings

import matplotlib as mpl
import matplotlib.pyplot as plt

import abstract_map.spatial_layout as sl

warnings.filterwarnings('ignore',
                        'Treat the new Tool classes introduced in v1.5.*')
warnings.filterwarnings('ignore', '.*GUI is implemented')


class Visualiser(object):
    PAUSE = 1e-6

    def __init__(self, rate=10):
        """Constructs a visualiser which controls visualisation rate"""
        self._delay = 1.0 / rate
        self._last_time = time.time()

        # Setup the figure, and save some handles to it
        mpl.rcParams['toolbar'] = 'None'
        self._fig = plt.figure()
        self._ax = self._fig.add_axes([0, 0, 1, 1])
        self._ax.tick_params(direction='in', pad=-22)
        self._ax.set_aspect('equal', 'datalim')

    def visualise(self, obj):
        """Callback for visualising an object, if ready to visualise"""
        # Bail if the time has not elapsed
        if time.time() < self._last_time + self._delay:
            return

        # Determine what visualisation function to call
        if isinstance(obj, sl.SpatialLayout):
            fn = self._visualiseSpatialLayout
        else:
            raise TypeError(
                "visualise() cannot visualise an object of class: %s" %
                (type(obj).__name__))

        # Perform the visualisation (and update the time)
        self._ax.clear()
        fn(obj)
        plt.pause(Visualiser.PAUSE)
        self._last_time = time.time()

    def _visualiseSpatialLayout(self, layout):
        """Visualises a spatial layout"""
        self._ax.scatter([m.pos[0] for m in layout._masses],
                         [m.pos[1] for m in layout._masses])
        for m in layout._masses:
            self._ax.annotate(m.name, m.pos)
