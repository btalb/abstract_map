import cPickle as pickle
import rospy

import nav_msgs.msg as nav_msgs
import std_msgs.msg as std_msgs

import abstract_map.visual as visual


class VisualiserNode:
    """ROS node for integrating ROS topics with Abstract Map visualisation"""
    _ZERO_DURATION = rospy.Duration(0)

    def __init__(self):
        """Initialise the node by setting up subscriptions and callbacks"""
        # Set up the visualiser, preparing it for the incoming messages
        self._visualiser = visual.Visualiser(
            window_type=visual.WindowType.IMMERSIVE)
        self._rate = rospy.get_param("rate", 10)

        # Declare all msg data objects
        self._abstract_map = None
        self._map = None
        self._plan = None
        self._pose = None

        self._is_abstract_map_new = None
        self._is_map_new = None
        self._is_plan_new = None
        self._is_pose_new = None

        # Configure all of the necessary ROS subscriptions
        self._sub_abstract_map = rospy.Subscriber(
            'abstract_map', std_msgs.String, self.cbAbstractMap)
        self._sub_map = rospy.Subscriber('map', nav_msgs.OccupancyGrid,
                                         self.cbMap)
        self._sub_plan = rospy.Subscriber('todo', nav_msgs.Path, self.cbPlan)
        self._sub_pose = rospy.Subscriber('odom', nav_msgs.Odometry,
                                          self.cbPose)

    def cbAbstractMap(self, msg):
        """Callback to handle visualising Abstract Map updates"""
        self._abstract_map = pickle.loads(msg.data)
        self._is_abstract_map_new = True

    def cbMap(self, msg):
        """Callback to handle visualising occapancy grid map updates"""
        self._map = msg
        self._is_map_new = False

    def cbPlan(self, msg):
        """Callback to handle visualising plan updates"""
        self._plan = msg
        self._is_plan_new = False

    def cbPose(self, msg):
        """Callback to handle visualising pose updates"""
        self._pose = msg
        self._is_pose_new = False

    def spin(self):
        """Blocking function to spin the visualiser at the configured rate"""
        r = rospy.Rate(self._rate)
        while not rospy.is_shutdown():
            # Perform all drawing
            if self._is_plan_new:
                # self._visualiser.draw(self._plan, 0)
                self._is_plan_new = False
            if self._is_abstract_map_new:
                self._visualiser.draw(self._abstract_map._spatial_layout, 0)
                self._is_abstract_map_new = False

            # Process all events for rest of loop
            while r.remaining() > VisualiserNode._ZERO_DURATION:
                self._visualiser.show()
            r.sleep()
