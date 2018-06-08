from PyQt5 import QtCore
import math
import sys
import time

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
    while not quit:
        while paused and not quit:
            time.sleep(0.1)
            layout._post_state_change_fcn(layout)

        layout.step()


if __name__ == '__main__':
    main(int(sys.argv[1]))
