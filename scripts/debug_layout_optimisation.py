import math
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


def keyControl(event):
    global paused, quit
    if event.key() == ord('P'):
        paused = not paused
    elif event.key() == ord('Q'):
        quit = True
        paused = False


def layoutTest(num):
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


def main(test_num):
    # Create a layout for the selected test
    layout = layoutTest(test_num)

    # Configure the plots for interactivity
    vis_layout._win.keyPressEvent = keyControl
    vis_energy._win.keyPressEvent = keyControl

    # Run through steps indefinitely...
    layout._post_state_change_fcn = stateVisual
    ta = time.time()
    limit = 60
    while not quit and layout._ode.t < limit:
        while paused and not quit:
            time.sleep(0.1)
            layout._post_state_change_fcn(layout)

        layout.step()

    layout._post_state_change_fcn(layout)
    print("Time for %fs system time: %fs (%d steps)" %
          (layout._ode.t, time.time() - ta, len(layout._log)))

    push = [1000 * x[0] for x in layout._log]
    refresh = [1000 * x[1] for x in layout._log]
    concat = [1000 * x[2] for x in layout._log]
    pl = pg.plot(title="Log")
    pl.addItem(pg.PlotDataItem(push, pen='r'))
    pl.addItem(pg.PlotDataItem(refresh, pen='b'))
    pl.addItem(pg.PlotDataItem(concat, pen='g'))
    print("Mean: push=%fms refresh=%fms concat=%fms" %
          (sum(push) / len(push), sum(refresh) / len(refresh),
           sum(concat) / len(concat)))
    pdb.set_trace()


if __name__ == '__main__':
    main(int(sys.argv[1]))
