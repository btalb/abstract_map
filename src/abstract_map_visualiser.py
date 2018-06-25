import rospy

import nav_msgs.msg as nav_msgs
import std_msgs.msg as std_msgs

import abstract_map.visual as visual


class AbstractMapVisualiserNode:
    """ROS node for integrating ROS topics with Abstract Map visualisation"""

    def __init__(self):
        """Initialise the node by setting up subscriptions and callbacks"""
        # Set up the visualiser, preparing it for the incoming messages
        self._visualiser = visual.Visualiser(
            window_type=visual.WindowType.IMMERSIVE)
        self._rate = rospy.get_param("rate", 10)

        # Configure all of the necessary ROS subscriptions
        self._sub_abstract_map = rospy.Subscriber(
            'abstract_map', std_msgs.ByteMultiArray, self.cbAbstractMap)
        self._sub_map = rospy.Subscriber('map', nav_msgs.OccupancyGrid,
                                         self.cbMap)
        self._sub_plan = rospy.Subscriber('todo', nav_msgs.Path, self.cbPlan)
        self._sub_pose = rospy.Subscriber('odom', nav_msgs.Odometry,
                                          self.cbPose)

    def cbAbstractMap(self, msg):
        """Callback to handle visualising Abstract Map updates"""
        pass

    def cbMap(self, msg):
        """Callback to handle visualising occapancy grid map updates"""
        pass

    def cbPlan(self, msg):
        """Callback to handle visualising plan updates"""
        pass

    def cbPose(self, msg):
        """Callback to handle visualising pose updates"""
        pass

    def spin(self):
        """Blocking function to spin the visualiser at the configured rate"""
        r = rospy.Rate(self._rate)
        while not rospy.is_shutdown():
            self._visualiser.visualise(None)
            r.sleep()
