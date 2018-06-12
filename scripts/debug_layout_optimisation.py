import math
import numpy as np
import pdb
import pyqtgraph as pg
import random
import sys
import time

from abstract_map import abstract_map as am
from abstract_map import spatial_layout as sl
from abstract_map import visual

try:
    input = raw_input
except NameError:
    pass

vis_layout = visual.Visualiser(visual.WindowType.IMMERSIVE)
vis_energy = visual.Visualiser()

paused = True
quit = False

slow_mode = True


def keyControl(event):
    global paused, quit
    if event.key() == ord('P'):
        paused = not paused
        print("%s" % ("Paused" if paused else "Running"))
    elif event.key() == ord('Q'):
        quit = True
        paused = False
        print("Quitting")


def layoutTest(num):
    global slow_mode
    layout = sl.SpatialLayout()
    if num == 0:
        # Basic 3 mass system
        ms = [sl.Mass('A'), sl.Mass('B'), sl.Mass('C')]
        ms[1].pos[0] = 1
        ms[2].pos[0] = -1

        # Construct the spatial layout
        layout.addConstraints([
            sl.ConstraintDistance(ms[0], ms[1], 2, 1),
            # sl.ConstraintAngleGlobal(ms[0], ms[1], math.pi / 4, 1),
            sl.ConstraintAngleLocal(ms[2], ms[1], ms[0], math.pi / 2, 1),
            # sl.ConstraintAngleGlobal(ms[2], ms[1], -math.pi / 4, 1),
            sl.ConstraintDistance(ms[2], ms[1], 2., 1)
        ])
        # layout.randomiseState(3)
    elif num == 1:
        ssi = [
            'here is in Hall, from here',
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
            'J is between K,and I, from here',
            'H is beside I, from here',
            'I is after J, from here',
            'K is near F, from here',
        ]
        m = am.AbstractMap('I', 0, 0, 0)
        for s in ssi:
            m.addSymbolicSpatialInformation(s, (0, 0, 0))

        layout = m._spatial_layout
        layout.initialiseState()
        # layout.randomiseState(5)

        slow_mode = False
    elif num == 2:
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
        layout.addMass(m_a)
        layout.addMass(m_b)
        layout.addMass(m_c)
        layout.addMass(m_d)
        layout.addMass(m_e)
    elif num == 3:
        m_0 = sl.MassFixed('0', np.array([0, 0]))
        m_1 = sl.MassFixed('1', np.array([2, 0]))
        m_a = sl.Mass('A', np.array([1.5, 0]))
        m_b = sl.Mass('B', np.array([0, 1]))
        m_c = sl.Mass('C', np.array([2, -1]))
        layout.addConstraints([
            sl.ConstraintDistance(m_0, m_a, 1, 1),
            sl.ConstraintAngleGlobal(m_b, m_0, math.pi / 3, 1),
            sl.ConstraintDistance(m_b, m_0, 1, 5),
            sl.ConstraintAngleLocal(m_c, m_1, m_a, math.pi / 3, 1),
            sl.ConstraintDistance(m_c, m_1, 1, 5)
        ])

        vis_layout._plt.setRange(xRange=(-1, 3), yRange=(-2, 2))
    else:
        raise ValueError("A valid layout test must be selected")

    layout.resetEnergyLog()
    return layout


def statePrint(layout):
    for m in layout._masses:
        print('Mass: %s' % (m.name))
        print('\tx:\t%f, %f' % (m.pos[0], m.pos[1]))
        print('\tdx:\t%f, %f' % (m.vel[0], m.vel[1]))
        print('\tddx:\t%f, %f' % (m.acc[0], m.acc[1]))


def stateVisual(layout):
    vis_layout.visualise(layout)
    vis_energy.visualise(layout._energy_log)


def timingLog(layout):
    t = [x for x in layout._log['a']]
    integrate = [1000 * x for x in layout._log['b']]
    push = [1000 * x for x in layout._log['c']]
    mark = [1000 * x for x in layout._log['d']]
    pl = pg.plot(title="Log")
    pl.addItem(pg.PlotDataItem(t, integrate, pen='r'))
    pl.addItem(pg.PlotDataItem(t, push, pen='b'))
    pl.addItem(pg.PlotDataItem(t, mark, pen='g'))
    li = pl.plotItem.addLegend()
    li.addItem(pl.plotItem.items[0], 'Integrate')
    li.addItem(pl.plotItem.items[1], 'Push State')
    li.addItem(pl.plotItem.items[2], 'Mark Changed')


def main(test_num):
    # Create a layout for the selected test
    layout = layoutTest(test_num)

    # Configure the plots for interactivity
    vis_layout._win.keyPressEvent = keyControl
    vis_energy._win.keyPressEvent = keyControl

    # Run through steps indefinitely...
    layout._post_state_change_fcn = stateVisual
    limit = 15
    while not quit and layout._ode.t < limit:
        while paused and not quit:
            layout._post_state_change_fcn(layout)
            time.sleep(0.1)

        layout.step()
        if slow_mode:
            time.sleep(0.1)

    timingLog(layout)
    while not quit:
        layout._post_state_change_fcn(layout)
        time.sleep(0.1)


if __name__ == '__main__':
    main(int(sys.argv[1]))
