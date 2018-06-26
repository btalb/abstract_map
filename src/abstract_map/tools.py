import numpy as np
import os
import sys
import tf_conversions


class abstractstatic(staticmethod):
    """Allows the abstractstatic decorator in Python 2 (not needed in 3.3+)"""
    __slots__ = ()

    def __init__(self, function):
        super(abstractstatic, self).__init__(function)
        function.__isabstractmethod__ = True

    __isabstractmethod__ = True


class HiddenPrints:
    """Hacky solution to stop library calls from printing junk..."""

    def __enter__(self):
        self._original_stdout = sys.stdout
        sys.stdout = None

    def __exit__(self, exc_type, exc_val, exc_tb):
        sys.stdout = self._original_stdout


def quaternionMsgToTuple(msg):
    """Helper function for converting a quaternion msg to a tuple"""
    return (msg.x, msg.y, msg.z, msg.w)


def quaternionMsgToYaw(msg):
    """Helper function for getting the yaw angle from a quaternion msg"""
    r, p, y = tf_conversions.transformations.euler_from_quaternion(
        quaternionMsgToTuple(msg))
    return y


def uv(vector):
    """Returns the unit vector of a 2D vector"""
    return (np.array([1, 0]) if not vector.any() else
            vector / (vector[0]**2 + vector[1]**2)**0.5)
