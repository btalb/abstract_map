from __future__ import absolute_import
import abc
from enum import Enum
import itertools
import multiprocessing as mp
import numpy as np
import operator
import os
import sys
import threading
import time
import warnings

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

import abstract_map_lib.spatial_layout as sl
import abstract_map_lib.tools as tools

warnings.filterwarnings('ignore',
                        'Treat the new Tool classes introduced in v1.5.*')
warnings.filterwarnings('ignore', '.*GUI is implemented')

# Abstract class compatibility across python 2 and python 3
ABC = abc.ABCMeta('ABC', (object,), {'__slots__': ()})

# Robot configuration parameters (TODO do this properly...)
_ROBOT_RADIUS = 0.35

# Colours (category10 colour palette)
_BG_COLOUR = 'w'
_TEXT_COLOUR = 'k'
_C1 = '#1f77b4'
_C2 = '#ff7f0e'
_C3 = '#2ca02c'
_C4 = '#d62728'

_OCC_LOOKUP_TABLE = pg.ColorMap(
    [-1, 0, 100],
    [[0, 0, 0, 0], [96, 96, 96, 255], [32, 32, 32, 255]]).getLookupTable(
        start=-1, stop=100, alpha=True, mode='byte')

_OVERLAY_COLOUR = '#ffffffa0'

# Pens used in visualisations (only initialise once)
_EL_TOTAL_PEN = pg.mkPen(_C1)
_EL_KINETIC_PEN = pg.mkPen(_C2)
_EL_POTENTIAL_PEN = pg.mkPen(_C3)

_GOAL_SIZE = 20
_GOAL_BRUSH = pg.mkBrush(_C4)
_GOAL_PEN = pg.mkPen(_C4)

_PO_PEN = pg.mkPen('k')
_PO_BRUSH = pg.mkBrush(_C4)

_PT_PEN = pg.mkPen(_C2, width=2)

_SL_LINES_PEN = pg.mkPen(_C1, width=2)
_SL_NODES_PEN = pg.mkPen(_C1)
_SL_NODES_BRUSH = pg.mkBrush(_C1)
_SL_NODES_FIXED_PEN = pg.mkPen(_C3)
_SL_NODES_FIXED_BRUSH = pg.mkBrush(_C3)
_SL_GROWTH_FACTOR = 2

# Random settings
_OVERLAY_WIDTH = 1000
_OVERLAY_HEIGHT = 1000

# pyplotgraph global configuration settings
pg.setConfigOptions(antialias=True, imageAxisOrder='row-major')


class GoalPrimitive(object):
    """Primitive representation of a goal position for visualisation"""

    def __init__(self, x, y):
        """Basic constructor with x, y"""
        self.x = x
        self.y = y


class MapPrimitive(object):
    """Primitive representation of map information for visualisation"""

    def __init__(self, data, resolution, top_left_pose):
        """Basic constructor for the map primitive"""
        self.data = data
        self.resolution = resolution
        self.top_left_pose = top_left_pose


class PathPrimitive(object):
    """Primitive representation of robot path information for visualisation"""

    def __init__(self, xs, ys):
        """Basic constructor with x and y coordinates"""
        self.xs = xs
        self.ys = ys


class PosePrimitive(object):
    """Primitive representation of robot pose information for visualisation"""

    def __init__(self, x, y, th):
        """Basic constructor with x, y, th (radians)"""
        self.x = x
        self.y = y
        self.th = th


class WindowType(Enum):
    """Enum for representing a window type request"""
    DEFAULT = 0
    IMMERSIVE = 1


class Visualiser(object):
    """Class containing all supported visualisation methods"""

    def __init__(self, window_type=WindowType.DEFAULT):
        """Constructs a visualiser which controls visualisation rate"""
        self._win_type = window_type
        self._layer_items = {}  # Dict of items for each visual "layer"

        self._overlay_items = []  # List of items for an overlay over the graph

        # Setup the window, and save some handles to it
        self._configureWindow()

    def _clearLayer(self, layer=0):
        """Clears all existing items for the requested layer"""
        for i in self._existingLayerItems(layer):
            self._plt.removeItem(i)

    def _configureWindow(self):
        """Configures the window based on the selected window type"""
        if self._win_type == WindowType.IMMERSIVE:
            pg.setConfigOptions(foreground='d', background=_BG_COLOUR)
            self._win = pg.plot(title="Abstact Map Visualisation")
            self._plt = self._win.plotItem
            self._plt.setAspectLocked(True, 1)
            self._plt.hideAxis('left')
            self._plt.hideAxis('bottom')
        else:  # DEFAULT
            pg.setConfigOptions(foreground='k', background='w')
            self._win = pg.plot(title="Abstact Map Visualisation")
            self._plt = self._win.plotItem

        # Set up the overlay objects as they are static
        self._overlay_items = [
            QtWidgets.QGraphicsRectItem(-_OVERLAY_WIDTH / 2,
                                        -_OVERLAY_HEIGHT / 2, _OVERLAY_WIDTH,
                                        _OVERLAY_HEIGHT)
        ]
        self._overlay_items[0].setBrush(pg.mkBrush(_OVERLAY_COLOUR))
        self._overlay_items[0].setZValue(1000)
        self._win.addItem(self._overlay_items[0])
        self.toggleOverlay(enable=False)

        # Do any last settings in the window
        # self._win.parentWidget().showMaximized()
        limit = 30
        self._win.setRange(xRange=[-limit, limit], yRange=[-limit, limit])

    def _drawEnergyLog(self, energy_log, layer=0, existing=[]):
        """Draws the energy log of a spatial layout"""
        self._clearLayer(layer)

        self._plt.plot(energy_log.t,
                       list(
                           map(operator.add, energy_log.kinetic,
                               energy_log.potential)),
                       pen=_EL_TOTAL_PEN)
        self._plt.plot(energy_log.t, energy_log.kinetic, pen=_EL_KINETIC_PEN)
        self._plt.plot(energy_log.t,
                       energy_log.potential,
                       pen=_EL_POTENTIAL_PEN)

        # Perform any first plot initialisation that may be necessary
        if self._plt.legend is None:
            li = self._plt.addLegend(offset=(-30, 30))
            li.addItem(self._plt.items[0], 'Total')
            li.addItem(self._plt.items[1], 'Kinetic')
            li.addItem(self._plt.items[2], 'Potential')
            self._plt.setLabels(title="<b>Spatial Layout Energy Tracking</b>",
                                left='Energy (J)',
                                bottom='System time (s)')

    def _drawFnFromType(self, obj):
        """Attempts to determine a drawing function based on the type"""
        if type(obj) is sl.SpatialLayout:
            fn = self._drawSpatialLayout
        elif type(obj) is sl.EnergyLog:
            fn = self._drawEnergyLog
        elif type(obj) is GoalPrimitive:
            fn = self._drawGoal
        elif type(obj) is MapPrimitive:
            fn = self._drawOccupancyGrid
        elif type(obj) is PathPrimitive:
            fn = self._drawPath
        elif type(obj) is PosePrimitive:
            fn = self._drawPose
        else:
            raise TypeError("draw() cannot draw an object of class: %s" %
                            (type(obj).__name__))
        return fn

    def _drawGoal(self, goal, layer=0, existing=[]):
        """Draws a pose assuming the coodinate frame matches the plot"""
        items = existing
        if not items:
            items.append(
                self._plt.plot([goal.x], [goal.y],
                               symbolSize=_GOAL_SIZE,
                               symbol='x',
                               symbolPen=_GOAL_PEN,
                               symbolBrush=_GOAL_BRUSH))
        else:
            items[-1].setData([goal.x], [goal.y])
        Visualiser._setLayer(items, layer)
        return items

    def _drawOccupancyGrid(self, occ_grid, layer=0, existing=[]):
        """Draws a map assuming coordinate frame matches the plot"""
        items = existing
        if not items:
            items.append(
                pg.ImageItem(image=occ_grid.data, lut=_OCC_LOOKUP_TABLE))
            self._plt.addItem(items[-1])
        else:
            items[-1].setImage(image=occ_grid.data)
        items[-1].setRect(
            QtCore.QRectF(occ_grid.top_left_pose.x, occ_grid.top_left_pose.y,
                          occ_grid.data.shape[0] * occ_grid.resolution,
                          occ_grid.data.shape[1] * occ_grid.resolution))
        Visualiser._setLayer(items, layer)
        return items

    def _drawPath(self, path, layer=0, existing=[]):
        """Draws a path assuming the coordinate frame matches the plot"""
        items = existing
        if not items:
            items.append(self._plt.plot(path.xs, path.ys, pen=_PT_PEN))
        else:
            items[-1].setData(path.xs, path.ys)
        Visualiser._setLayer(items, layer)
        return items

    def _drawPose(self, pose, layer=0, existing=[]):
        """Draws a pose assuming the coordinate frame matches the plot"""
        items = existing
        if not items:
            items.extend([
                self._plt.plot([pose.x], [pose.y],
                               pxMode=False,
                               pen=None,
                               symbolSize=_ROBOT_RADIUS,
                               symbol='o',
                               symbolPen=_PO_PEN,
                               symbolBrush=_PO_BRUSH),
                self._plt.plot([pose.x], [pose.y],
                               pen=None,
                               symbol=Visualiser._triangleSymbol(pose.th),
                               symbolPen=_PO_PEN,
                               symbolBrush=_PO_BRUSH)
            ])
        else:
            items[-2].setData([pose.x], [pose.y])
            items[-1].setData([pose.x], [pose.y],
                              symbol=Visualiser._triangleSymbol(pose.th))

        Visualiser._setLayer(items, layer)
        return items

    def _drawSpatialLayout(self, layout, layer=0, existing=[]):
        """Draws a spatial layout"""
        # Get a starting list of existing items
        items = existing
        if not items:
            # 0 is series of lines for each constraint, 1 is a dict of mass
            # plots for each level, 2 is all labels
            items.extend([[], {}, {}])

        # Update the constraints (adding if we are missing one)
        it = iter(items[0])
        for c in layout._constraints:
            ps = np.concatenate([m.pos for m in c.masses()])
            constraint_plot = next(it, None)
            if constraint_plot is None:
                constraint_plot = self._plt.plot(ps[::2],
                                                 ps[1::2],
                                                 pen=_SL_LINES_PEN)
                items[0].append(constraint_plot)
            else:
                constraint_plot.setData(ps[::2], ps[1::2])

        # Update the masses by level (adding a level if needed...)
        label_parents = {}
        for level, g in itertools.groupby(
                sorted(layout._masses, key=lambda x: x._level),
                lambda x: x._level):
            ms = list(g)
            level_plot = items[1].get(level, None)

            # Draw all of the mass nodes
            ps = np.concatenate([m.pos for m in ms])
            if level_plot is None:
                s_size = 10 if level <= 1 else 10 * _SL_GROWTH_FACTOR * (
                    level - 1)
                s = 'o' if level > sl.MASS_LEVEL_LABEL else 's'
                s_pen = (_SL_NODES_PEN if level != sl.MASS_LEVEL_LABEL else
                         _SL_NODES_FIXED_PEN)
                s_brush = (_SL_NODES_BRUSH if level != sl.MASS_LEVEL_LABEL else
                           _SL_NODES_FIXED_BRUSH)
                level_plot = self._plt.plot(ps[::2],
                                            ps[1::2],
                                            pen=None,
                                            symbol=s,
                                            symbolSize=s_size,
                                            symbolPen=s_pen,
                                            symbolBrush=s_brush)
                items[1][level] = level_plot
            else:
                level_plot.setData(ps[::2], ps[1::2])

            # Save the data to help in labelling
            for i, m in enumerate(ms):
                label_parents[m.name] = (level_plot, i)

        # Update all of the mass labels
        for m in layout._masses:
            label_item = items[2].get(m.name, None)
            if label_item is None:
                label_item = pg.TextItem(text=m.name,
                                         color=_TEXT_COLOUR,
                                         anchor=(0.5, 0))
                label_item.setParentItem(pg.CurvePoint(*label_parents[m.name]))
                items[2][m.name] = label_item
            else:
                label_item.setParentItem(pg.CurvePoint(*label_parents[m.name]))

        # Add a title if appropriate
        if layout._energy_log is not None and layout._energy_log.t:
            self._plt.setTitle("t = %f" % (layout._energy_log.t[-1]))

        # Finish up
        Visualiser._setLayer(tools.flatten(items), layer)
        return items

    def _existingLayerItems(self, layer):
        """Attempts to get any existing layer items"""
        if layer is None:
            return None
        elif not isinstance(layer, int) or layer < 0:
            raise ValueError("Invalid layer number requested (%s)" % (layer))
        else:
            return self._layer_items.get(layer, [])

    @staticmethod
    def _setLayer(items, layer):
        """Set the z layer for a list of plot items"""
        for i in items:
            i.setZValue(layer)

    @staticmethod
    def _triangleSymbol(angle):
        """Returns a triangle symbol (QPainterPath) pointing at an angle"""
        s = QtGui.QPainterPath()
        s.moveTo(0.5 * np.cos(-angle), 0.5 * np.sin(-angle))
        s.lineTo(0.5 * np.cos(-angle + 0.75 * np.pi),
                 0.5 * np.sin(-angle + 0.75 * np.pi))
        s.lineTo(0.5 * np.cos(-angle - 0.75 * np.pi),
                 0.5 * np.sin(-angle - 0.75 * np.pi))
        s.closeSubpath()
        return s

    def clear(self):
        """Clears the entire window"""
        self._plt.clear()
        self._layer_items = {}

    def close(self):
        """Closes the visualiser"""
        with tools.HiddenPrints():
            self._win.close()

    def draw(self, obj, layer=0):
        """Draws the requested object if a drawing method can be found"""
        # self._clearLayer(layer)
        self._layer_items[layer] = self._drawFnFromType(obj)(
            obj, layer=layer, existing=self._layer_items.get(layer, []))

    def show(self):
        """Blunt tool to force showing of any updates"""
        QtGui.QGuiApplication.processEvents()

    def toggleOverlay(self, enable=True):
        """Toggles the overlay for the window"""
        if enable:
            self._overlay_items[0].show()
        else:
            self._overlay_items[0].hide()

    def visualise(self, obj):
        """Immediately visualises an object"""
        self.clear()
        self.draw(obj)
        self.show()


class _Updater(object):
    """Manages the update cycle of a visualiser object"""

    def __init__(self, update_method, rate):
        """Initialises an updater with a method and rate"""
        self._delay = 1.0 / rate
        self._last_time = 0
        self._update_method = update_method

    def update(self, obj):
        """Calls the update method with the obj only if it is time to"""
        if time.time() > self._last_time + self._delay:
            self._update_method(obj)
            self._last_time = time.time()


class _Controller(ABC):
    """Class to handle interfacing with an underlying visualiser"""

    def __init__(self):
        """Super constructor for setting up the basic controller mechanics"""
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

    @abc.abstractmethod
    def setRange(self, x_range, y_range):
        """Sets the x and y range for the attached visualiser"""
        pass

    def update(self, obj):
        """Method where the controller is asked to update the visualiser"""
        self._updater.update(obj)


class BasicController(_Controller):
    """Class for the basic, same thread, interface to a visualiser"""

    def __init__(self, visualiser, rate=10):
        """Constructor which attaches to a specified visualiser"""
        super(BasicController, self).__init__()
        self._visualiser = visualiser
        self._updater = _Updater(visualiser.visualise, rate)
        self._visualiser._win.keyPressEvent = (
            lambda ev: self.processKey(ev.key()))

    def close(self):
        """Closes the visualiser"""
        self._visualiser.close()

    @staticmethod
    def create(window_type=WindowType.DEFAULT, rate=10):
        """Function for creating a basic controller from basic info"""
        return BasicController(Visualiser(window_type=window_type))

    def setRange(self, x_range, y_range):
        """Sets the x and y range for the attached visualiser"""
        self._visualiser._plt.setRange(xRange=x_range, yRange=y_range)


class AsyncController(_Controller):
    """Class for interfacing with a visualiser on another thread"""

    def __init__(self, pipe, rate=10):
        """Constructor for attaching the controller to a specified pipe"""
        super(AsyncController, self).__init__()
        self._pipe = pipe
        self._pipe_thread = threading.Thread(target=self._pipe_monitor)
        self._updater = _Updater(self._pipe.send, rate)
        self._is_closed = False

        self._pipe_thread.start()

    def _pipe_monitor(self):
        """Monitors the pipe for any received data"""
        while self._pipe_thread.isAlive() and not self._is_closed:
            if self._pipe.poll(1):
                recv_obj = self._pipe.recv()
                self.processKey(recv_obj)

    @staticmethod
    def _target(pipe, window_type=WindowType.DEFAULT, rate=10):
        """The target of async requests, where an async visualiser is run"""
        # Create a visualiser and an updater to control the visualiser (we
        # want to be discarding updates that we don't have time to visualise)
        v = Visualiser(window_type=window_type)
        v._win.keyPressEvent = lambda ev: pipe.send(ev.key())

        updater = _Updater(v.visualise, rate)

        # Run the main loop, polling for new data over the pipe, and using the
        # updater to control updating of the visualiser
        quit = False
        latest_object = None
        while not quit:
            # Get any waiting data on the pipe
            if pipe.poll():
                recv_obj = pipe.recv()
                if recv_obj == "close":
                    quit = True
                elif (isinstance(recv_obj, basestring) and
                      recv_obj.startswith("range")):
                    vs = map(float, recv_obj.split(',')[1:])
                    v._plt.setRange(xRange=(vs[0], vs[1]),
                                    yRange=(vs[2], vs[3]))
                else:
                    latest_object = recv_obj

            # Give the updater the chance to update visualiser if ready
            if latest_object is not None:
                updater.update(latest_object)

        # We are here because we are quitting, close the visualiser
        v.close()

    def close(self):
        """Closes the visualiser"""
        self._pipe.send("close")
        self._is_closed = True

    @staticmethod
    def create(window_type=WindowType.DEFAULT, rate=10):
        """Function for creating an async controller from basic info"""
        pipe_target, pipe_controller = mp.Pipe(duplex=True)
        controller = AsyncController(pipe_controller)
        p = mp.Process(target=AsyncController._target,
                       args=(pipe_target, window_type, rate))
        p.start()
        return controller

    def setRange(self, x_range, y_range):
        """Sets the x and y range for the attached visualiser"""
        self._pipe.send("range,%f,%f,%f,%f" % (x_range + y_range))
