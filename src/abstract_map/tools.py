import tf_conversions


def quaternionMsgToTuple(msg):
    """Helper function for converting a quaternion msg to a tuple"""
    return (msg.x, msg.y, msg.z, msg.w)


def quaternionMsgToYaw(msg):
    """Helper function for getting the yaw angle from a quaternion msg"""
    r, p, y = tf_conversions.transformations.euler_from_quaternion(
        quaternionMsgToTuple(msg))
    return y
