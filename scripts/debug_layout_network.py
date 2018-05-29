import math
import pdb
import random

from abstract_map import spatial_layout as sl
from abstract_map import visual

try:
    input = raw_input
except NameError:
    pass

visualiser = visual.Visualiser()


def statePrint(layout):
    for m in layout._masses:
        print('Mass: %s' % (m.name))
        print('\tx:\t%f, %f' % (m.pos[0], m.pos[1]))
        print('\tdx:\t%f, %f' % (m.vel[0], m.vel[1]))
        print('\tddx:\t%f, %f' % (m.acc[0], m.acc[1]))


def stateVisual(layout):
    visualiser.visualise(layout)
    # statePrint(layout)
    # input(".")


def main():
    # Create a new spatial layout
    layout = sl.SpatialLayout()

    # Setup the visualisation
    layout._post_step_fcn = stateVisual

    # Construct and intialise the masses of the system
    ms = [sl.Mass('A'), sl.Mass('B'), sl.Mass('C')]
    for m in ms:
        m.pos[0] = random.random()
        m.pos[1] = random.random()

    # Add the constraints to the layout
    layout.addConstraints([
        sl.ConstraintDistance(ms[0], ms[1], 2.5, 1),
        # sl.ConstraintAngleGlobal(ms[0], ms[1], math.pi / 4, 1),
        sl.ConstraintAngleLocal(ms[2], ms[1], ms[0], math.pi / 2, 1),
        # sl.ConstraintAngleGlobal(ms[2], ms[1], -math.pi / 4, 1),
        sl.ConstraintDistance(ms[2], ms[1], 2., 1)
    ])

    # Run through steps indefinitely...
    stateVisual(layout)
    input("Press enter to start")
    while 1:
        # pdb.set_trace()
        layout.step()


if __name__ == '__main__':
    main()
