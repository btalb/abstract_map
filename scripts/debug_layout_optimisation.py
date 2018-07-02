import math
import numpy as np
import os
import pickle
import pudb
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui
import random
import sys
import string
import time

from abstract_map import abstract_map as am
from abstract_map import spatial_layout as sl
from abstract_map import tools
from abstract_map import visual

try:
    input = raw_input
except NameError:
    pass

parallelise = True

slow_mode = True

paused = False
done = False

log_win = None

if parallelise:
    con_layout = visual.AsyncController.create(
        window_type=visual.WindowType.IMMERSIVE)
    con_energy = visual.AsyncController.create()
else:
    con_layout = visual.BasicController.create(
        window_type=visual.WindowType.IMMERSIVE)
    con_energy = visual.BasicController.create()


def keyEvent(key):
    global paused, done
    if key == ord('P'):
        paused = not paused
    elif key == ord('Q'):
        done = True


def layoutTest(num):
    global slow_mode
    layout = sl.SpatialLayout()
    if num == 0:
        # Basic 3 mass system
        ms = [sl.Mass('A'), sl.Mass('B'), sl.Mass('C')]
        ms[1].pos[0] = 1
        ms[2].pos[0] = -1

        # Construct the spatial layout
        layout.addConstraints(
            [
                sl.ConstraintDistance(ms[0], ms[1], 2, 1),
                # sl.ConstraintAngleGlobal(ms[0], ms[1], math.pi / 4, 1),
                sl.ConstraintAngleLocal(ms[2], ms[1], ms[0], math.pi / 2, 1),
                # sl.ConstraintAngleGlobal(ms[2], ms[1], -math.pi / 4, 1),
                sl.ConstraintDistance(ms[2], ms[1], 2., 1)
            ],
            place=False)
        # layout.randomiseState(3)
    elif num == 1:
        # Full key world test
        ssi = [
            'here is near A, from here',
            'B is down Hall, from here',
            'D is down Hall, from here',
            'F is down Hall, from here',
            'C is right of B, from here',
            'E is right of D, from here',
            'D is before F, from here',
            'D is after B, from here',
            'F is beyond D, from here',
            'G is beyond F, from here',
            'G is towards H, from here',
            'G is left of F, from here',
            'K is right of F, from here',
            'J is right of F, from here',
            'J is between K, and I, from here',
            'H is beside I, from here',
            'I is after J, from here',
            'K is near F, from here',
        ]
        m = am.AbstractMap('I', 0, 0, 0, log=True)
        for s in ssi:
            m.addSymbolicSpatialInformation(s, (0, 0, 0))

        layout = m._spatial_layout
        layout.initialiseState()

        slow_mode = False
    elif num == 2:
        # Collision and bouncing test
        m_a = sl.Mass('A')
        m_b = sl.Mass('B')
        m_c = sl.Mass('C')
        m_d = sl.Mass('D')
        m_e = sl.Mass('E')
        m_a.pos = np.array([-1, 0])
        m_b.pos = np.array([0, 1 + sl.SAFE_DISTANCE])
        m_c.pos = np.array([1 + sl.SAFE_DISTANCE, 0])
        m_d.pos = np.array([0, -1 - sl.SAFE_DISTANCE])
        m_e.pos = np.array([-1 - sl.SAFE_DISTANCE, -1])
        m_a.vel = np.array([0.5, 0.5])
        m_e.vel = np.array([0, 0.125])
        layout.addMass(m_a, place=False)
        layout.addMass(m_b, place=False)
        layout.addMass(m_c, place=False)
        layout.addMass(m_d, place=False)
        layout.addMass(m_e, place=False)

        sl.FRICTION_COEFFICIENT = 0
    elif num == 3:
        # Test with fixed masses
        m_0 = sl.MassFixed('0', np.array([0, 0]))
        m_1 = sl.MassFixed('1', np.array([2, 0]))
        m_a = sl.Mass('A', np.array([1.5, 0]))
        m_b = sl.Mass('B', np.array([0, 1]))
        m_c = sl.Mass('C', np.array([2, -1]))
        layout.addConstraints(
            [
                sl.ConstraintDistance(m_0, m_a, 1, 1),
                sl.ConstraintAngleGlobal(m_b, m_0, math.pi / 3, 1),
                sl.ConstraintDistance(m_b, m_0, 1, 5),
                sl.ConstraintAngleLocal(m_c, m_1, m_a, math.pi / 3, 1),
                sl.ConstraintDistance(m_c, m_1, 1, 5)
            ],
            place=False)

        con_layout.setRange(x_range=(-1, 3), y_range=(-2, 2))
    elif num == 4:
        # Test of adding masses with default placement algorithm
        for c in string.ascii_uppercase:
            pauseBlock(layout)
            layout.addMass(sl.Mass(c))
    else:
        raise ValueError("A valid layout test must be selected")

    layout.resetEnergyLog()
    return layout


def pauseBlock(layout):
    global paused
    paused = True
    while paused:
        con_layout.update(layout)
        time.sleep(0.1)


def statePrint(layout):
    print("PRINTING STATE:")
    for m in layout._masses:
        print('Mass: %s' % (m.name))
        print('\tx:\t%f, %f' % (m.pos[0], m.pos[1]))
        print('\tdx:\t%f, %f' % (m.vel[0], m.vel[1]))
        print('\tddx:\t%f, %f' % (m.acc[0], m.acc[1]))


def stateVisual(layout):
    con_layout.update(layout)
    con_energy.update(layout._energy_log)


def timingLog(layout):
    global log_win

    # Perform the plotting aspect
    t = [x for x in layout._log['a']]
    integrate = [1000 * x for x in layout._log['b']]
    push = [1000 * x for x in layout._log['c']]
    mark = [1000 * x for x in layout._log['d']]
    log_win = pg.plot(title="Log")
    log_win.addItem(pg.PlotDataItem(t, integrate, pen='r'))
    log_win.addItem(pg.PlotDataItem(t, push, pen='b'))
    log_win.addItem(pg.PlotDataItem(t, mark, pen='g'))
    li = log_win.plotItem.addLegend()
    li.addItem(log_win.plotItem.items[0], 'Integrate')
    li.addItem(log_win.plotItem.items[1], 'Push State')
    li.addItem(log_win.plotItem.items[2], 'Mark Changed')
    log_win.keyPressEvent = lambda ev: keyEvent(ev.key())

    # Print some summary data
    integrate = sum(integrate)
    push = sum(push)
    mark = sum(mark)
    total = sum([integrate, push, mark])
    print("Timing distribution (ms):\tint: %.1f\tpush: %.1f\tmark: %.1f" %
          (integrate, push, mark))
    print("Timing distribution (%%):\tint: %.1f\tpush: %.1f\tmark: %.1f" %
          (100 * integrate / total, 100 * push / total, 100 * mark / total))


def main(test_num):
    # Configure the plots to control the program state
    con_layout.key_handler = keyEvent
    con_energy.key_handler = keyEvent

    # Create a layout for the selected test
    layout = layoutTest(test_num)
    layout.executeWaitingCalls()

    # Run through steps indefinitely...
    layout._post_state_change_fcn = stateVisual
    limit = 15
    a = time.time()
    while not done and layout._ode.t < limit:
        while paused and not done:
            layout._post_state_change_fcn(layout)
            time.sleep(0.1)

        layout.step()
        if slow_mode:
            time.sleep(0.1)
    print("Took %f seconds" % (time.time() - a))

    timingLog(layout)
    log_win.showButtons()
    while not done:
        layout._post_state_change_fcn(layout)
        QtGui.QGuiApplication.processEvents()
        time.sleep(0.1)

    with tools.HiddenPrints():
        con_layout.close()
        con_energy.close()
        log_win.close()


if __name__ == '__main__':
    main(int(sys.argv[1]))
