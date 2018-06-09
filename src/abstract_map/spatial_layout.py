import abc
import itertools
import numpy as np
import pdb
import random
import scipy.linalg as la
import scipy.integrate as ig
import sys
import time
import warnings

warnings.filterwarnings('ignore', '.*GUI is implemented')

# Abstract class compatibility across python 2 and python 3
ABC = abc.ABCMeta('ABC', (object,), {'__slots__': ()})

# Constants for the default behaviour of spatial layout
FRICTION_COEFFICIENT = 1
INTEGRATION_DT = 0.1

STIFF_XL = 5
STIFF_L = 1
STIFF_M = 0.5
STIFF_S = 0.01

DIST_UNIT = 1
DIR_ZERO = 0


class MyIntegrator(object):

    def __init__(self, f):
        self.f = f
        self.y = []
        self.t = 0

    def set_initial_value(self, y, t):
        self.y = y
        self.t = t

    def integrate(self, t_new):
        k1 = self.f(self.t, self.y)
        k2 = self.f(self.t, self.y + INTEGRATION_DT * 0.5 * k1)
        k3 = self.f(self.t, self.y + INTEGRATION_DT * 0.5 * k2)
        k4 = self.f(self.t, self.y + INTEGRATION_DT * k3)
        self.y += (1. / 6.) * (k1 + 2 * k2 + 2 * k3 + k4) * INTEGRATION_DT
        self.t = t_new
        return self.y


class SpatialLayout(object):
    """A set of springs and masses denoting abstract ideas about space"""

    def __init__(self, log_energy=True):
        """Constructs a new empty spatial layout"""
        self._constraints = []
        self._masses = []

        self._system_changed = False

        self._energy_log = EnergyLog() if log_energy else None
        self._post_state_change_fcn = None

        self._log = []

        # Initialise the ode solver
        self._ode = ig.ode(self._stateDerivative).set_integrator(
            'dopri5', atol=1e-6, rtol=1e-3)

    def _pullState(self):
        """Pulls the current state matrix of the system"""
        return np.concatenate(
            [np.concatenate((m.pos, m.vel)) for m in self._masses])

    def _pushState(self, y):
        """Pushes state matrix into system (obeying any safety conditions)"""
        # TODO safety conditions
        for i, m in enumerate(self._masses):
            m.pos = y[(i * 4):(i * 4 + 2)]
            m.vel = y[(i * 4 + 2):(i * 4 + 4)]

    def _refreshForces(self):
        """Refreshes the force value for each mass in the system"""
        for m in self._masses:
            m.acc[:] = 0
            m.applyFriction()

        for c in self._constraints:
            c.applyForce()

    def _stateDerivative(self, t, y):
        """Computes the derivative of the current state"""
        ta = time.time()
        self._pushState(y)
        tb = time.time()
        self._refreshForces()
        tc = time.time()
        y = np.concatenate(
            [np.concatenate((m.vel, m.acc)) for m in self._masses])
        td = time.time()
        self._log.append([tb - ta, tc - tb, td - tc])
        return y

    def addConstraints(self, cs):
        for c in cs:
            self.addConstraint(c)

    def addConstraint(self, c):
        """Adds a constraint (and any new masses to the layout)"""
        # Force only one mass in the system with a specified name
        for i, m in enumerate(c.masses()):
            m_found = self.getMass(m.name)
            if m_found is not None:
                if i == 0:
                    c._mass_a = m_found
                elif i == 1:
                    c._mass_b = m_found
                elif i == 2:
                    c._mass_c = m_found

        # Add in the new components
        for m in c.masses():
            self.addMass(m)

        self._constraints.append(c)

        # Mark that the system state has been changed
        self.markStateChanged()

    def addMass(self, m):
        """Adds a mass to the layout (only if it is new)"""
        if m not in self._masses:
            self._masses.append(m)

            # Mark that the system state has been changed
            self.markStateChanged()

    def getMass(self, name):
        """Returns a mass with the requested name if it exists"""
        return next((m for m in self._masses if m.name == name), None)

    def initialiseState(self):
        """Initialises the state to best match provided constraints"""
        # Sort all masses and constraints into the "best" order (best is
        # defined as iteratively placing the mass that will "complete" the most
        # remaining constraints on placement)
        cs = []
        ms = []
        while self._constraints or self._masses:
            # Recompute score for each mass (number of constraints that its
            # placement will complete)
            scores = [
                sum([
                    len(list(set(c.masses()).intersection(self._masses))) == 1
                    for c in self._constraints
                ])
                for m in self._masses
            ]

            # Place the "best" unplaced mass
            m_best = self._masses[scores.index(min(scores))]
            print("Mass placed: %s" % (m_best.name))
            ms.append(m_best)
            self._masses.remove(m_best)

            # Move all constraints with all masses placed to the placed list
            c_placed = [
                c for c in self._constraints
                if len(set(c.masses()).intersection(self._masses)) == 0
            ]
            for c in c_placed:
                print("\tConstraint placed: '%s'" % (c))
                cs.append(c)
                self._constraints.remove(c)

        # Now place all of the masses in order (using the constraints to inform
        # placement)
        print('\n\n')
        for m in ms:
            print('Mass retrieved: %s' % (m.name))
            # Get a list of placement suggestions from the added constraints
            cs_complete = [
                c for c in cs if set(c.masses()).issubset(self._masses + [m])
            ]
            for c in cs_complete:
                print("\tCompletes: '%s'" % (c))

            ps = [c.placementSuggestion(m) for c in cs_complete]
            for p in ps:
                print("\tSuggestion gained (ref: %s): %s" % (p['mass'].name, p))

            # Remove empties, and merge placement suggestions by mass that the
            # suggestion is relative to
            F_MASS = lambda x: x['mass'].name  # noqa: E731
            ps = [p for p in ps if p]
            ps_merged = []
            for m_key, g in itertools.groupby(sorted(ps, key=F_MASS), F_MASS):
                g = list(g)
                rs = np.array([list(p['r']) for p in g if 'r' in p])
                ths = np.array([list(p['th']) for p in g if 'th' in p])
                merged = {'mass': g[0]['mass']}
                if rs.size > 0:
                    merged['r'] = (np.sum(np.prod(rs, 1)) / np.sum(rs[:, 1]),
                                   np.sum(rs[:, 1]))
                if ths.size > 0:
                    mean_vector = np.sum(
                        np.array([np.cos(ths[:, 0]),
                                  np.sin(ths[:, 0])]) * ths[:, 1], 1)
                    merged['th'] = (np.arctan2(mean_vector[1], mean_vector[0]),
                                    np.sum(ths[:, 1]))
                ps_merged.append(merged)

            # Get an ideal location for placement, based on the merged
            # suggestions
            ps_merged = sorted(
                ps_merged, key=lambda x: ('r' in x and 'th' in x, 'th' in x))
            placement = np.zeros((2))
            weight = 0
            for p in ps_merged:
                print("\tMerged suggestion (ref: %s): %s" % (p['mass'].name, p))
                # Get a suggested position
                if 'r' in p and 'th' in p:
                    # Suggested is simply r,th from reference position
                    suggested = p['mass'].pos + np.array([
                        p['r'][0] * np.cos(p['th'][0]),
                        p['r'][0] * np.sin(p['th'][0])
                    ])
                    w = p['r'][1] + p['th'][1]  # Not sure if should div 2...
                elif 'th' in p:
                    # Suggested is on line at angle theta from reference, with
                    # distance along line always guaranteed to be greater than
                    # 1 (suggesting close to reference is bad for system
                    # stability, & 1 is also fallback if no current placement)
                    uv = np.array([np.cos(p['th'][0]), np.sin(p['th'][0])])
                    r = np.dot(placement - p['mass'].pos, uv)
                    suggested = p['mass'].pos + (1 if r < 1 or weight == 0 else
                                                 r) * uv
                    w = p['th'][1]
                elif 'r' in p:
                    # Suggested is a distance r from the reference, in the
                    # direction of the suggested placement (direction falls
                    # back to 0 if no existing placement)
                    uv = np.array([
                        1, 0
                    ]) if weight == 0 else ((placement - p['mass'].pos) /
                                            la.norm(placement - p['mass'].pos))
                    suggested = p['mass'].pos + p['r'][0] * uv
                    w = p['r'][1]

                # Incorporate the placement suggestion, and update the weight
                print("\tPlacement suggestion (ref: %s): (%f, %f)" %
                      (p['mass'].name, suggested[0], suggested[1]))
                placement = (placement * weight + suggested * w) / (weight + w)
                weight += w

            # FINALLY, place the mass and add the constraints in
            m.pos = placement
            print("Suggesting to place %s @ (%f, %f)" % (m.name, m.pos[0],
                                                         m.pos[1]))
            self._masses.append(m)
            cs = [c for c in cs if c not in cs_complete]
            self._constraints.extend(cs_complete)

    def logEnergy(self):
        """Writes the current system energy to the enrgy log if available"""
        if self._energy_log is not None:
            self._energy_log.logEnergy(self)

    def markStateChanged(self, system_changed=True, reset=False):
        """Explicit declaration of a change of system state"""
        self._system_changed = system_changed

        if reset:
            self.resetEnergyLog()

        self.logEnergy()

        if self._post_state_change_fcn is not None:
            self._post_state_change_fcn(self)

    def step(self):
        """Performs a single iteration of the spatial layout optimisation"""
        # Handle system changes if present
        if self._system_changed:
            self._ode.set_initial_value(self._pullState(), self._ode.t)
            self._system_changed = False

        # Perform a step with the ODE integrator
        state_next = self._ode.integrate(self._ode.t + INTEGRATION_DT)

        # Safely apply the suggested new state
        self._pushState(state_next)

        # Mark that the system state has been changed
        self.markStateChanged(False)

    def resetEnergyLog(self):
        """Resets the energy log"""
        if self._energy_log is not None:
            self._energy_log.reset()

    def randomiseState(self, window_size=5):
        """Randomises the initial state within a given window size"""
        for m in self._masses:
            m.pos[0] = (random.random() - 0.5) * window_size
            m.pos[1] = (random.random() - 0.5) * window_size
            m.vel = np.zeros_like(m.vel)
            m.acc = np.zeros_like(m.acc)

        # Mark that the system state has been changed
        self.markStateChanged(reset=True)

    def updateConstraints(self, cs):
        """Update existing constraints from a tag id (instead of adding)"""
        assert cs[0]._tag_id >= 0, "To update, tag_ids must be >= 0"
        assert all(
            c._tag_id == cs[0]._tag_id for c in cs
        ), "All constraints that are being updated must have the same tag ID"
        tag_id = cs[0]._tag_id
        self._constraints = [
            c for c in self._constraints if c._tag_id != tag_id
        ]
        self._constraints.extend(cs)

        # Mark that the system state has been changed
        self.markStateChanged()


class EnergyLog(object):
    """Log of the energy within a layout system"""

    def __init__(self):
        """Initialise the empty logs"""
        self.reset()

    def logEnergy(self, layout):
        """Logs the current energy in the spatial layout object"""
        self.t.append(layout._ode.t)
        self.kinetic.append(sum([m.totalEnergy() for m in layout._masses]))
        self.potential.append(
            sum([c.totalEnergy() for c in layout._constraints]))

    def reset(self):
        """Resets the log"""
        self.t = []
        self.kinetic = []
        self.potential = []


class _Energised(ABC):
    """Abstraction for an inhereting class to denote it contains energy"""

    @abc.abstractmethod
    def totalEnergy(self):
        pass


class Mass(_Energised):
    """A point-mass, representing a toponym's location in a spatial layout"""

    def __init__(self, name):
        """Constructs a new point mass, with a given name"""
        _Energised.__init__(self)

        self.name = name
        self._mass = 1
        self.pos = np.zeros((2))
        self.vel = np.zeros((2))
        self.acc = np.zeros((2))

    def applyFriction(self):
        """Applies the friction force to the mass"""
        self.acc += -FRICTION_COEFFICIENT * self.vel

    def totalEnergy(self):
        """Returns the kinetic energy in the moving mass"""
        return 0.5 * self._mass * np.sum(np.square(self.vel))


class Constraint(_Energised, ABC):
    """A spring like constraint guide for relative position of point-masses"""

    def __init__(self, tag_id=-1):
        """Constructor which gives a tag_id to link the constraint to"""
        self._tag_id = tag_id

    @abc.abstractmethod
    def __str__(self):
        """Force every subclass to implement a verbose string representation"""
        pass

    def totalEnergy(self):
        """Returns the potential energy held by the constraint"""
        return 0.5 * self._stiffness * np.square(self.displacement())

    @abc.abstractmethod
    def applyForce(self):
        """Applies the current constraint force to each attached point-mass"""
        pass

    @abc.abstractmethod
    def displacement(self):
        """Distance the spring is displaced from its natural length"""
        pass

    @abc.abstractmethod
    def length(self):
        """Returns length of the constraint (same units as natural length)"""
        pass

    @abc.abstractmethod
    def masses(self):
        """Returns a list of masses in the constraint"""
        pass

    @abc.abstractmethod
    def placementSuggestion(self, mass):
        """Returns a placement tuple suggesting where to place the mass"""
        pass


class ConstraintDistance(Constraint):
    """A constraint on the distance between two point-masses"""

    def __init__(self, mass_a, mass_b, natural_length, stiffness, tag_id=-1):
        """Constructs the specified constraint between masses"""
        super(ConstraintDistance, self).__init__(tag_id)

        self._mass_a = mass_a
        self._mass_b = mass_b
        self._natural_length = natural_length
        self._stiffness = stiffness

    def __str__(self):
        return "Constrain distance between %s & %s to %f (%f)" % (
            self._mass_a.name, self._mass_b.name, self._natural_length,
            self._stiffness)

    def applyForce(self):
        """Applies the constraint force to masses a and b"""
        force_vector = -self._stiffness * self.displacement() * _uv(
            self._mass_a, self._mass_b)
        self._mass_a.acc += force_vector / self._mass_a._mass
        self._mass_b.acc += -force_vector / self._mass_b._mass

    def displacement(self):
        """Distance the spring is displaced from its natural length"""
        return self.length() - self._natural_length

    def masses(self):
        """Returns the list of masses in the distance constraint"""
        return [self._mass_a, self._mass_b]

    def length(self):
        """Returns distance between position of mass a and b"""
        return _distance(self._mass_a, self._mass_b)

    def placementSuggestion(self, mass):
        """Returns a placement tuple suggesting where to place the mass"""
        if mass == self._mass_a:
            return {
                'mass': self._mass_b,
                'r': (self._natural_length, self._stiffness)
            }
        elif mass == self._mass_b:
            return {
                'mass': self._mass_a,
                'r': (self._natural_length, self._stiffness)
            }
        else:
            return {}


class ConstraintAngleGlobal(Constraint):
    """A constraint on the angle between two point-masses, in the global frame"""

    def __init__(self, mass_a, mass_b, natural_length, stiffness, tag_id=-1):
        """Constructs the specified constraint between masses"""
        super(ConstraintAngleGlobal, self).__init__(tag_id)

        self._mass_a = mass_a
        self._mass_b = mass_b
        self._natural_length = natural_length
        self._stiffness = stiffness

    def __str__(self):
        return "Constrain angle to %s from %s to %f (%f)" % (
            self._mass_a.name, self._mass_b.name, self._natural_length,
            self._stiffness)

    def applyForce(self):
        """Applies the constraint force to masses a and b"""
        force_vector = -self._stiffness * self.displacement() / _distance(
            self._mass_a, self._mass_b) * _orthog(
                _uv(self._mass_a, self._mass_b))

        self._mass_a.acc += force_vector / self._mass_a._mass
        self._mass_b.acc += -force_vector / self._mass_b._mass

    def displacement(self):
        """Distance the spring is displaced from its natural length"""
        return _angleWrap(self.length() - self._natural_length)

    def masses(self):
        """Returns the list of masses in the global angular constraint"""
        return [self._mass_a, self._mass_b]

    def length(self):
        """Returns angle of of mass a relative to mass b, in global frame"""
        return _angle(self._mass_a, self._mass_b)

    def placementSuggestion(self, mass):
        """Returns a placement tuple suggesting where to place the mass"""
        if mass == self._mass_a:
            return {
                'mass': self._mass_b,
                'th': (self._natural_length, self._stiffness)
            }
        elif mass == self._mass_b:
            return {
                'mass': self._mass_a,
                'th': (_angleWrap(self._natural_length + np.pi),
                       self._stiffness)
            }
        else:
            return {}


class ConstraintAngleLocal(Constraint):
    """A constraint on the angle formed by three point-masses"""

    def __init__(self,
                 mass_a,
                 mass_b,
                 mass_c,
                 natural_length,
                 stiffness,
                 tag_id=-1):
        """Constructs the specified constraint between masses"""
        super(ConstraintAngleLocal, self).__init__(tag_id)

        self._mass_a = mass_a
        self._mass_b = mass_b
        self._mass_c = mass_c
        self._natural_length = natural_length
        self._stiffness = stiffness

    def __str__(self):
        return "Constrain angle to %s from %s (relative to %s) to %f (%f)" % (
            self._mass_a.name, self._mass_b.name, self._mass_c.name,
            self._natural_length, self._stiffness)

    def applyForce(self):
        """Applies the constraint force to masses a, b, and c"""
        force_vector_a = -self._stiffness * self.displacement() / _distance(
            self._mass_a, self._mass_b) * _orthog(
                _uv(self._mass_a, self._mass_b))
        force_vector_c = -self._stiffness * self.displacement() / _distance(
            self._mass_c, self._mass_b) * _orthog(
                _uv(self._mass_c, self._mass_b))

        acc_a = force_vector_a / self._mass_a._mass
        acc_c = force_vector_c / self._mass_c._mass
        self._mass_a.acc += acc_a
        self._mass_b.acc += -acc_a + -acc_c
        self._mass_c.acc += acc_c

    def displacement(self):
        """Distance the spring is displaced from its natural length"""
        return _angleWrap(self.length() - self._natural_length)

    def masses(self):
        """Returns the list of masses in the local angular constraint"""
        return [self._mass_a, self._mass_b, self._mass_c]

    def length(self):
        """Returns angle of mass a, relative to vector from mass b to c"""
        return _angle(self._mass_a, self._mass_b, self._mass_c)

    def placementSuggestion(self, mass):
        """Returns a placement tuple suggesting where to place the mass"""
        if mass == self._mass_a:
            return {
                'mass':
                self._mass_b,
                'th': (_angleWrap(
                    _angle(self._mass_c, self._mass_b) + self._natural_length),
                       self._stiffness)
            }
        elif mass == self._mass_b:
            # There is no easy way to do this (the path of possible placements
            # of B follows a complex arc, which is discontinous because it is
            # present on both sides - i.e. constraint can be on left or right
            # side of |AC|). The whole optimisation process is needed for
            # overcoming problems like these. So for now, take the easy option
            # and simply place a suggestion (relative to C) for B to be at the
            # midpoint of |AC|
            return {
                'mass':
                self._mass_c,
                'r': (_distance(self._mass_a, self._mass_c) / 2,
                      self._stiffness / 2),
                'th': (_angle(self._mass_a, self._mass_c), self._stiffness / 2)
            }
        elif mass == self._mass_c:
            return {
                'mass':
                self._mass_b,
                'th': (_angleWrap(
                    _angle(self._mass_a, self._mass_b) - self._natural_length),
                       self._stiffness)
            }
        else:
            return {}


def _angle(mass_a, mass_b, mass_c=None):
    """Compute the angle formed by mass a, relative to b (and optionally c) """
    v_ab = mass_a.pos - mass_b.pos
    ret = np.arctan2(v_ab[1], v_ab[0])
    if mass_c is not None:
        v_ac = mass_c.pos - mass_b.pos
        ret -= np.arctan2(v_ac[0], v_ac[1])

    return _angleWrap(ret)


def _angleWrap(angle):
    """Returns the angle, in the range of [-PI,+PI)"""
    ret = (angle + np.pi) % (2 * np.pi)
    if ret < 0:
        ret += 2 * np.pi

    return ret - np.pi


def _distance(mass_a, mass_b):
    """Computes the distance between two masses"""
    return la.norm(mass_a.pos - mass_b.pos)


def _uv(mass_a, mass_b):
    """Returns the unit vector pointing to mass a, from mass b"""
    return np.array([1, 0]).T if np.array_equal(mass_a.pos, mass_b.pos) else (
        mass_a.pos - mass_b.pos) / la.norm(mass_a.pos - mass_b.pos)


def _orthog(vector):
    return np.array([-vector[1], vector[0]])