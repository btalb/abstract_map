import abc
import numpy as np
import pdb
import random
import scipy.linalg as la
import scipy.integrate as ig

# Abstract class compatibility across python 2 and python 3
ABC = abc.ABCMeta('ABC', (object,), {'__slots__': ()})

# Constants for the default behaviour of spatial layout
FRICTION_COEFFICIENT = 0
INTEGRATION_DT = 0.01

STIFF_XL = 5
STIFF_L = 1
STIFF_M = 0.5
STIFF_S = 0.01

DIST_UNIT = 1
DIR_ZERO = 0


class SpatialLayout(object):
    """A set of springs and masses denoting abstract ideas about space"""

    def __init__(self, log_energy=True):
        """Constructs a new empty spatial layout"""
        self._constraints = []
        self._masses = []

        self._system_changed = False

        self._energy_log = EnergyLog() if log_energy else None
        self._post_state_change_fcn = None

        # Initialise the ode solver
        self._ode = ig.ode(self._stateDerivative).set_integrator('dopri5')

    def _pullState(self):
        """Pulls the current state matrix of the system"""
        y = np.zeros([len(self._masses) * 4, 1])
        for i, m in enumerate(self._masses):
            y[(i * 4):(i * 4 + 2)] = m.pos
            y[(i * 4 + 2):(i * 4 + 4)] = m.vel

        return y

    def _pushState(self, y):
        """Pushes state matrix into system (obeying any safety conditions)"""
        # TODO safety conditions
        for i, m in enumerate(self._masses):
            m.pos = y[(i * 4):(i * 4 + 2)]
            m.vel = y[(i * 4 + 2):(i * 4 + 4)]

    def _refreshForces(self):
        """Refreshes the force value for each mass in the system"""
        for m in self._masses:
            m.acc = np.zeros_like(m.acc)
            m.applyFriction()

        for c in self._constraints:
            c.applyForce()

    def _stateDerivative(self, t, y):
        """Computes the derivative of the current state"""
        # Push the supplied state, and update all forces
        self._pushState(y.reshape(-1, 1))
        self._refreshForces()

        # Extract and return the state derivative
        dy = np.empty_like(np.atleast_2d(y).T)
        for i, m in enumerate(self._masses):
            dy[(i * 4):(i * 4 + 2)] = m.vel
            dy[(i * 4 + 2):(i * 4 + 4)] = m.acc

        return dy

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
        self.pos = np.zeros((2, 1))
        self.vel = np.zeros((2, 1))
        self.acc = np.zeros((2, 1))

    def applyFriction(self):
        """Applies the friction force to the mass"""
        self.acc += -FRICTION_COEFFICIENT * self.vel

    def totalEnergy(self):
        """Returns the kinetic energy in the moving mass"""
        return 0.5 * self._mass * np.sum(np.square(self.vel))


class Constraint(_Energised, ABC):
    """A spring like constraint guiding the relative position of point-masses"""

    def __init__(self, tag_id=-1):
        """Constructor which gives a tag_id to link the constraint to"""
        self._tag_id = tag_id

    def totalEnergy(self):
        """Returns the potential energy held by the constraint"""
        return 0.5 * self._stiffness * np.square(self.length() -
                                                 self._natural_length)

    @abc.abstractmethod
    def applyForce(self):
        """Applies the constraint's current force to each attached point-mass"""
        pass

    @abc.abstractmethod
    def masses(self):
        """Returns a list of masses in the constraint"""
        pass

    @abc.abstractmethod
    def length(self):
        """Returns length of the constraint (same units as natural length)"""
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

    def applyForce(self):
        """Applies the constraint force to masses a and b"""
        force_vector = -self._stiffness * (self.length() - self._natural_length
                                          ) * _uv(self._mass_a, self._mass_b)
        self._mass_a.acc += force_vector / self._mass_a._mass
        self._mass_b.acc += -force_vector / self._mass_b._mass

    def masses(self):
        """Returns the list of masses in the distance constraint"""
        return [self._mass_a, self._mass_b]

    def length(self):
        """Returns distance between position of mass a and b"""
        return _distance(self._mass_a, self._mass_b)


class ConstraintAngleGlobal(Constraint):
    """A constraint on the angle between two point-masses, in the global frame"""

    def __init__(self, mass_a, mass_b, natural_length, stiffness, tag_id=-1):
        """Constructs the specified constraint between masses"""
        super(ConstraintAngleGlobal, self).__init__(tag_id)

        self._mass_a = mass_a
        self._mass_b = mass_b
        self._natural_length = natural_length
        self._stiffness = stiffness

    def applyForce(self):
        """Applies the constraint force to masses a and b"""
        force_vector = -self._stiffness * _angleWrap(
            self.length() - self._natural_length) / _distance(
                self._mass_a, self._mass_b) * _orthog(
                    _uv(self._mass_a, self._mass_b))

        print("Length: %f, Natural Length: %f, Deviation: %f" %
              (self.length(), self._natural_length,
               self.length() - self._natural_length))

        self._mass_a.acc += force_vector / self._mass_a._mass
        self._mass_b.acc += -force_vector / self._mass_b._mass

    def masses(self):
        """Returns the list of masses in the global angular constraint"""
        return [self._mass_a, self._mass_b]

    def length(self):
        """Returns angle of of mass a relative to mass b, in global frame"""
        return _angle(self._mass_a, self._mass_b)


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

    def applyForce(self):
        """Applies the constraint force to masses a, b, and c"""
        force_vector_a = -self._stiffness * (
            self.length() - self._natural_length) / _distance(
                self._mass_a, self._mass_b) * _orthog(
                    _uv(self._mass_a, self._mass_b))
        force_vector_c = -self._stiffness * (
            self.length() - self._natural_length) / _distance(
                self._mass_c, self._mass_b) * _orthog(
                    _uv(self._mass_c, self._mass_b))

        acc_a = force_vector_a / self._mass_a._mass
        acc_c = force_vector_c / self._mass_a._mass
        self._mass_a.acc += acc_a
        self._mass_b.acc += -acc_a + -acc_c
        self._mass_c.acc += acc_c

    def masses(self):
        """Returns the list of masses in the local angular constraint"""
        return [self._mass_a, self._mass_b, self._mass_c]

    def length(self):
        """Returns angle of mass a, relative to vector from mass b to c"""
        return _angle(self._mass_a, self._mass_b, self._mass_c)


def _angle(mass_a, mass_b, mass_c=None):
    """Computes the angle formed by the positions of masses"""
    v_ab = mass_a.pos - mass_b.pos
    ret = np.arctan2(v_ab[1], v_ab[0])
    if mass_c != None:
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
    return np.array([[1, 0]]).T if np.array_equal(mass_a.pos, mass_b.pos) else (
        mass_a.pos - mass_b.pos) / la.norm(mass_a.pos - mass_b.pos)


def _orthog(vector):
    return np.array([-vector[1], vector[0]])
