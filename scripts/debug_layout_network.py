import math

from abstract_map import spatial_layout as sl
from abstract_map import visual

try:
    input = raw_input
except NameError:
    pass


def statePrint(layout):
    for m in layout._masses:
        print('Mass: %s' % (m.name))
        print('\tx:\t%f, %f' % (m.pos[0], m.pos[1]))
        print('\tdx:\t%f, %f' % (m.vel[0], m.vel[1]))


def stateVisual(layout):
    visual.visualise(layout)


def main():
    # Create a new spatial layout
    layout = sl.SpatialLayout()

    # Setup the visualisation
    visualiser = visual.Visualiser()
    layout._post_step_fcn = visualiser.visualise

    # Construct the masses and constraints of the layout
    ma = sl.Mass('A')
    mb = sl.Mass('B')
    mc = sl.Mass('C')

    c1 = sl.ConstraintDistance(ma, mb, 2.5, 0.8)
    c2 = sl.ConstraintAngleLocal(mc, mb, ma, math.pi / 2, 1.5)

    # Add the elements to the layout
    layout.addConstraint(c1)
    # layout.addConstraint(c2)

    # Run through steps indefinitely...
    visualiser._fig.show()
    input("Press enter to start")
    while 1:
        layout.step()


if __name__ == '__main__':
    main()
