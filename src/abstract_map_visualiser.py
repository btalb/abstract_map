import cPickle as pickle
import numpy as np
import rospy
import tf2_geometry_msgs
import tf2_ros

import geometry_msgs.msg as geometry_msgs
import nav_msgs.msg as nav_msgs
import std_msgs.msg as std_msgs

import abstract_map.visual as visual
import abstract_map.tools as tools


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
        self._goal = None
        self._map = None
        self._plan = None
        self._pose = None

        self._is_abstract_map_new = None
        self._is_goal_new = None
        self._is_map_new = None
        self._is_plan_new = None
        self._is_pose_new = None

        # Configure all of the necessary ROS subscriptions
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._sub_abstract_map = rospy.Subscriber(
            'abstract_map',
            std_msgs.String,
            self.cbAbstractMap,
            queue_size=100)
        self._sub_goal = rospy.Subscriber(
            '/move_base_simple/goal',
            geometry_msgs.PoseStamped,
            self.cbGoal,
            queue_size=1)
        self._sub_map = rospy.Subscriber(
            'map', nav_msgs.OccupancyGrid, self.cbMap, queue_size=1)
        self._sub_plan = rospy.Subscriber(
            '/move_base/GlobalPlanner/plan',
            nav_msgs.Path,
            self.cbPlan,
            queue_size=1)
        self._sub_pose = rospy.Subscriber(
            'odom', nav_msgs.Odometry, self.cbPose, queue_size=1)

    def cbAbstractMap(self, msg):
        """Callback to handle visualising Abstract Map updates"""
        self._abstract_map = pickle.loads(msg.data)
        self._is_abstract_map_new = True

    def cbGoal(self, msg):
        self._goal = visual.GoalPrimitive(
            *(tools.poseMsgToXYTh(msg.pose)[0:2]))
        self._is_goal_new = True

    def cbMap(self, msg):
        """Callback to handle visualising occapancy grid map updates"""
        self._map = visual.MapPrimitive(
            np.asarray(msg.data).reshape((msg.info.width, msg.info.height)),
            msg.info.resolution,
            visual.PosePrimitive(*tools.poseMsgToXYTh(msg.info.origin)))
        self._is_map_new = True

    def cbPlan(self, msg):
        """Callback to handle visualising plan updates"""
        self._plan = visual.PathPrimitive(
            [p.pose.position.x for p in msg.poses],
            [p.pose.position.y for p in msg.poses])
        self._is_plan_new = True

    def cbPose(self, msg):
        """Callback to handle visualising pose updates"""
        try:
            # Needed because:
            # - stage with multiple robots stupidly prepends frames with slash
            # - tf2 is too moronic to handle frames starting with a slash...
            msg.header.frame_id = msg.header.frame_id.strip("/")

            transformed_pose = self._tf_buffer.transform(
                geometry_msgs.PoseStamped(
                    header=msg.header, pose=msg.pose.pose), "map",
                rospy.Duration(1.0))
            self._pose = visual.PosePrimitive(
                *tools.poseMsgToXYTh(transformed_pose.pose))
            self._is_pose_new = True
        except Exception as e:
            return

    def spin(self):
        """Blocking function to spin the visualiser at the configured rate"""
        r = rospy.Rate(self._rate)
        while not rospy.is_shutdown():  # TODO close if window is closed
            # Process all events for rest of loop
            while r.remaining() > VisualiserNode._ZERO_DURATION:
                self._visualiser.show()
            r.sleep()

            # Perform all drawing
            if self._is_abstract_map_new:
                # t = time.time()
                self._visualiser.draw(self._abstract_map._spatial_layout, 3)
                self._visualiser.toggleOverlay(
                    enable=not self._abstract_map._spatial_layout.isSettled())
                # rospy.loginfo(
                #     "Draw Abstract Map took: %fs" % (time.time() - t))
                self._is_abstract_map_new = False
            if self._is_goal_new:
                self._visualiser.draw(self._goal, 4)
                self._is_goal_new = False
            if self._is_map_new:
                # t = time.time()
                self._visualiser.draw(self._map, 0)
                # rospy.loginfo(
                #     "Draw Occupancy Grid took: %fs" % (time.time() - t))
                self._is_map_new = False
            if self._is_plan_new:
                # t = time.time()
                self._visualiser.draw(self._plan, 1)
                # rospy.loginfo("Draw Plan took: %fs" % (time.time() - t))
                self._is_plan_new = False
            if self._is_pose_new:
                # t = time.time()
                self._visualiser.draw(self._pose, 2)
                # rospy.loginfo("Draw Robot Pose took: %fs" % (time.time() - t))
                self._is_pose_new = False
