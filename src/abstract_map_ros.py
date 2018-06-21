from __future__ import absolute_import
import math
import numpy as np
import pudb
import rospy
import tf

import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs
import nav_msgs.msg as nav_msgs

import human_cues_tag_reader_msgs.msg as human_cues_tag_reader_msgs
import abstract_map.abstract_map as am
import abstract_map.tools as tools


class AbstractMapNode(object):
    """ROS node for integrating an abstract map into a navigating robot"""

    def __init__(self, visualiser=None):
        """Configure the node, attaching to all required topics"""
        # Initialise an Abstract Map with information that already exists
        start_pose = rospy.wait_for_message("/odom",
                                            nav_msgs.Odometry).pose.pose
        x = start_pose.position.x
        y = start_pose.position.y
        th = tools.quaternionMsgToYaw(start_pose.orientation)
        self._goal = rospy.get_param("goal", None)
        self._abstract_map = am.AbstractMap(self._goal, x, y, th)
        rospy.loginfo(
            "Starting Abstract Map @ (%f, %f) facing %f deg, with the goal: %s"
            % (x, y, th * 180. / math.pi, "None"
               if self._goal is None else self._goal))
        self._ssi_store = _SsiCache()
        self.visualiser = visualiser

        # Configure the ROS side
        self._sub_ssi = rospy.Subscriber(
            'symbolic_spatial_info',
            human_cues_tag_reader_msgs.SymbolicSpatialInformation,
            self.cbSymbolicSpatialInformation)
        if self._goal is not None:
            self._pub_goal = rospy.Publisher(
                'goal', geometry_msgs.PoseStamped, queue_size=1)
        else:
            rospy.logwarn(
                "No goal received; Abstract Map will run in observe mode.")

        self._pub_debug = rospy.Publisher(
            'debug', geometry_msgs.PoseArray, queue_size=1)

    def spin(self):
        """Blocking function where the Abstract Map operates"""
        while not rospy.is_shutdown():
            self._abstract_map._spatial_layout.step()

    @property
    def visualiser(self):
        """ Gets the attached visualiser"""
        return self._visualiser

    @visualiser.setter
    def visualiser(self, value):
        self._visualiser = value
        if self._visualiser is not None:
            self._abstract_map._spatial_layout._post_state_change_fcn = (
                self._visualiser.visualise)

    def cbSymbolicSpatialInformation(self, msg):
        """Callback to process any new symbolic spatial information received"""
        assert isinstance(
            msg, human_cues_tag_reader_msgs.SymbolicSpatialInformation)
        fn = (self._abstract_map.addSymbolicSpatialInformation
              if self._ssi_store.addSymbolicSpatialInformation(msg) else
              self._abstract_map.updateSymbolicSpatialInformation)
        fn(msg.ssi, self._ssi_store._store[msg.tag_id].meanPose(), msg.tag_id)
        rospy.loginfo("%s symoblic spatial information: %s (tag_id=%d)" % (
            ("Added" if fn == self._abstract_map.addSymbolicSpatialInformation
             else "Updated"), msg.ssi, msg.tag_id))


class _SsiCache(object):
    """Organised cache for a collection of symbolic spatial information"""

    def __init__(self):
        """Construct a new empty store """
        self._store = {}

    def addSymbolicSpatialInformation(self, ssi):
        """Adds symbolic spatial information to store, returns if new or not"""
        assert isinstance(
            ssi, human_cues_tag_reader_msgs.SymbolicSpatialInformation)
        if ssi.tag_id >= 0 and ssi.tag_id in self._store:
            self._store[ssi.tag_id].addRosPose(ssi.location)
            return False
        else:
            self._store[ssi.tag_id] = _SsiCache._SsiCacheItem(
                ssi.tag_id, ssi.ssi, ssi.location)
            return True

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
