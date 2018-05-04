import math

import numpy as np
import rospy
import tf

import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs

import human_cues_tag_reader_msgs.msg as human_cues_tag_reader_msgs


class AbstractMapNode:
    """ROS node for integrating an abstract map into a navigating robot"""

    def __init__(self):
        """Construct a new node, attaching to all required topics"""
        rospy.init_node('abstract_map')

        self._ssi_store = _SsiCache()

        self._pub_goal = rospy.Publisher(
            'goal', geometry_msgs.PoseStamped, queue_size=1)
        self._sub_ssi = rospy.Subscriber(
            'symbolic_spatial_info',
            human_cues_tag_reader_msgs.SymbolicSpatialInformation,
            self.callbackSsi)

        self._pub_debug = rospy.Publisher(
            'debug', geometry_msgs.PoseArray, queue_size=1)

    def callbackSsi(self, msg):
        """Callback to process any new symbolic spatial information received"""
        assert isinstance(msg,
                          human_cues_tag_reader_msgs.SymbolicSpatialInformation)
        self._ssi_store.addSymbolicSpatialInformation(msg)
        rospy.logwarn("Observed tag %d:" % (msg.tag_id))
        rospy.logwarn("\tSymbolic spatial information: %s" % (msg.ssi))
        mp = self._ssi_store._store[msg.tag_id].meanPose()
        rospy.logwarn(
            "\tMean observed pose: %s" % (', '.join(str(x) for x in mp)))

        # Debugging to check pose stability...
        d = geometry_msgs.PoseArray()
        d.header = msg.header
        d.header.frame_id = "map"
        for v in self._ssi_store._store.itervalues():
            mp = v.meanPose()
            p = geometry_msgs.Pose()
            p.position = geometry_msgs.Point(*(mp[0], mp[1], 2.0))
            p.orientation = geometry_msgs.Quaternion(
                *tf.transformations.quaternion_from_euler(0, 0, mp[2]))
            d.poses.append(p)

        rospy.logwarn("%d poses in msg, %d values in cache" % (len(
            d.poses), len(self._ssi_store._store.keys())))
        self._pub_debug.publish(d)


class _SsiCache(object):
    """Organised cache for a collection of symbolic spatial information"""

    def __init__(self):
        """Construct a new empty store """
        self._store = {}

    def addSymbolicSpatialInformation(self, ssi):
        """Adds a new piece of symbolic spatial information to the store"""
        assert isinstance(ssi,
                          human_cues_tag_reader_msgs.SymbolicSpatialInformation)
        if ssi.tag_id in self._store:
            self._store[ssi.tag_id].addRosPose(ssi.location)
        else:
            self._store[ssi.tag_id] = _SsiCache._SsiCacheItem(
                ssi.tag_id, ssi.ssi, ssi.location)

    class _SsiCacheItem(object):
        """Item representing a distinct piece of symbolic spatial information"""
        _ROTATION = tf.transformations.quaternion_from_euler(
            math.pi, -math.pi / 2, 0)

        def __init__(self, tag_id, ssi, pose):
            """Construct a cache item from identifying data (i.e. no pose)"""
            assert isinstance(pose, geometry_msgs.Pose)
            self.tag_id = tag_id
            self.ssi = ssi

            self.poses = []
            self.addRosPose(pose)

        def addRosPose(self, ros_pose):
            """Adds a pose from a ROS message"""
            assert isinstance(ros_pose, geometry_msgs.Pose)
            world_quartenion = tf.transformations.quaternion_multiply([
                ros_pose.orientation.x, ros_pose.orientation.y,
                ros_pose.orientation.z, ros_pose.orientation.w
            ], self._ROTATION)
            (r, p,
             y) = tf.transformations.euler_from_quaternion(world_quartenion)
            self.poses.append([
                ros_pose.position.x, ros_pose.position.y,
                math.cos(y),
                math.sin(y)
            ])

        def meanPose(self):
            """Calculates the mean pose observed for a requested tag"""
            return [
                np.mean([x[0] for x in self.poses]),
                np.mean([x[1] for x in self.poses]),
                math.atan2(
                    np.mean([x[3] for x in self.poses]),
                    np.mean([x[2] for x in self.poses]))
            ]
