from __future__ import absolute_import
import cPickle as pickle
import cv2
import math
import numpy as np
import rospy
import time
import tf

import actionlib_msgs.msg as actionlib_msgs
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs
import nav_msgs.msg as nav_msgs

import human_cues_tag_reader_msgs.msg as human_cues_tag_reader_msgs
import abstract_map.abstract_map as am
import abstract_map.tools as tools
import abstract_map.spatial_layout as sl


class AbstractMapNode(object):
    """ROS node for integrating an abstract map into a navigating robot"""
    _ZERO_DURATION = rospy.Duration(0)

    def __init__(self):
        """Configure the node, attaching to all required topics"""
        # DEBUG TODO DELETE
        self._debug_lock = False
        self._debug_publish = True
        self._debug_coem = rospy.Publisher('coem',
                                           geometry_msgs.PoseStamped,
                                           queue_size=1)

        # Get parameters and initialisation messages from ROS
        self._publish_abstract_map = rospy.get_param("~publish_abstract_map",
                                                     True)
        # self._publish_rate = rospy.Rate(rospy.get_param("~publish_rate", 10))
        self._goal = rospy.get_param("~goal", "")
        self._goal_complete = False
        self._last_goal_status = None
        start_pose = rospy.wait_for_message("/odom",
                                            nav_msgs.Odometry).pose.pose
        x = start_pose.position.x
        y = start_pose.position.y
        th = tools.quaternionMsgToYaw(start_pose.orientation)

        # Used to ensure we only publish on a change in settled state
        self._last_settled = None

        # Initialise an Abstract Map with information that already exists
        self._abstract_map = am.AbstractMap(self._goal, x, y, th, log=False)
        rospy.loginfo(
            "Starting Abstract Map @ (%f, %f) facing %f deg, with the goal: %s"
            % (x, y, th * 180. / math.pi,
               "None" if not self._goal else self._goal))
        self._ssi_store = _SsiCache()

        # Configure the ROS side
        self._sub_vel = rospy.Subscriber('cmd_vel_suggested',
                                         geometry_msgs.Twist, self.cbVelocity)
        self._pub_vel = rospy.Publisher('cmd_vel',
                                        geometry_msgs.Twist,
                                        queue_size=10)

        self._sub_ssi = rospy.Subscriber(
            'symbolic_spatial_info',
            human_cues_tag_reader_msgs.SymbolicSpatialInformation,
            self.cbSymbolicSpatialInformation)
        self._pub_goal = (rospy.Publisher(
            '/move_base_simple/goal', geometry_msgs.PoseStamped, queue_size=10)
                          if self._goal else None)
        self._sub_goal_status = (rospy.Subscriber(
            '/move_base/status', actionlib_msgs.GoalStatusArray,
            self.cbGoalStatus) if self._goal else None)
        if self._pub_goal is None:
            rospy.logwarn(
                "No goal received; Abstract Map will run in observe mode.")

        self._pub_am = (rospy.Publisher(
            'abstract_map', std_msgs.String, latch=True, queue_size=1)
                        if self._publish_abstract_map else None)

        # Configure the spatial layout to publish itself if necessary
        if self._pub_am is not None:
            self._abstract_map._spatial_layout._post_state_change_fcn = (
                self.publish)

        # Pull in a hierarchy if one is found
        self.pullInHierarchy()

    def _update_coem(self):
        """Extracts and stores a new explored center of mass in the map"""
        # Get the latest occupancy grid map
        latest_map = rospy.wait_for_message("/map", nav_msgs.OccupancyGrid)

        # Build an opencv binary image, with 1 corresponding to space that has
        # been marked as free
        map_data = np.array(latest_map.data,
                            dtype=np.int8).reshape(latest_map.info.width,
                                                   latest_map.info.height)
        free_space_map = ((map_data >= 0) & (map_data <= 50)).astype(np.uint8)

        # Use "image moments" to compute the centre of mass
        # TODO should probably incorporate orientation in coordinate calc...
        ms = cv2.moments(free_space_map, True)
        centre = np.array([ms['m10'], ms['m01']]) / ms['m00']
        centre_coordinates = np.array([
            latest_map.info.origin.position.x,
            latest_map.info.origin.position.y
        ]) + latest_map.info.resolution * centre

        # Update "centre of explored mass" in the abstract map
        # TODO remove debug
        self._abstract_map._spatial_layout._coem = centre_coordinates
        self._debug_coem.publish(
            geometry_msgs.PoseStamped(
                header=std_msgs.Header(stamp=rospy.Time.now(), frame_id='map'),
                pose=tools.xythToPoseMsg(centre_coordinates[0],
                                         centre_coordinates[1], 0)))

    def cbGoalStatus(self, msg):
        # TODO make this much less brittle...
        current_status = (msg.status_list[0].status
                          if msg.status_list else None)
        if (self._last_goal_status == actionlib_msgs.GoalStatus.ACTIVE and
                current_status >= actionlib_msgs.GoalStatus.SUCCEEDED):
            # Bump the exploration factor
            print("Goal was not found at the expected location. "
                  "Increasing exploration factor...")
            self._abstract_map._spatial_layout._scale_manager.bumpExploration()
            self._abstract_map._spatial_layout._paused = False

        self._last_goal_status = current_status

    def cbSymbolicSpatialInformation(self, msg):
        """Callback to process any new symbolic spatial information received"""
        assert isinstance(
            msg, human_cues_tag_reader_msgs.SymbolicSpatialInformation)
        # Discard SSI if it is empty
        if not msg.ssi:
            return

        # Add SSI to the SSI store, and call the appropriate function based on
        # whether it registers as new or an update
        fn = (self._abstract_map.addSymbolicSpatialInformation
              if self._ssi_store.addSymbolicSpatialInformation(msg) else
              self._abstract_map.updateSymbolicSpatialInformation)
        for i, s in enumerate(msg.ssi.split("\\n")):
            fn(s, self._ssi_store._store[msg.tag_id].meanPose(),
               (msg.tag_id, i))
            # Only unpause if the SSI is new TODO do this smarter...
            if fn == self._abstract_map.addSymbolicSpatialInformation:
                self._update_coem()
                sl = self._abstract_map._spatial_layout
                sl._scale_manager.resetExploration()
                sl._paused = False
                rospy.loginfo("Added SSI: \"%s\" (tag_id=%d,line#=%d)" %
                              (s, msg.tag_id, i))

    def cbVelocity(self, msg):
        """Callback to only push velocity to robot if layout is settled"""
        if (self._abstract_map._spatial_layout.isSettled() and
                not self._goal_complete and not self._debug_lock):
            self._pub_vel.publish(msg)

    def publish(self, *_):
        """Publishes to any required topics, only if conditions are met"""
        del _
        # DEBUG TODO DELETE
        if self._debug_publish:
            self._pub_am.publish(
                std_msgs.String(data=pickle.dumps(self._abstract_map)))

        # Only proceed publishing if the network has recently changed from
        # being settled to unsettled (or vice versa)
        settled = self._abstract_map._spatial_layout.isSettled()
        if settled == self._last_settled:
            return

        # Update the paused status
        if settled:
            self._abstract_map._spatial_layout._paused = True

        # Publish the abstract map as a pickled byte stream
        self._pub_am.publish(
            std_msgs.String(data=pickle.dumps(self._abstract_map)))

        # Publish an updated goal if the running in goal mode
        if (self._abstract_map._spatial_layout.isObserved(self._goal) and
                settled):
            self._goal_complete = True
            rospy.loginfo("MISSION ACCOMPLISHED! %s was found." % (self._goal))
        elif settled and self._pub_goal is not None:
            # Get the suggested pose for the goal from the abstract map
            goal_pos = self._abstract_map.getToponymLocation(self._goal)

            # Send the message (only if we found a suggested pose for the goal)
            if goal_pos is not None:
                self._pub_goal.publish(
                    geometry_msgs.PoseStamped(
                        header=std_msgs.Header(stamp=rospy.Time.now(),
                                               frame_id='map'),
                        pose=geometry_msgs.Pose(
                            position=geometry_msgs.Vector3(
                                goal_pos[0], goal_pos[1], 0),
                            orientation=tools.yawToQuaternionMsg(0))))

        # Update the last_settled state
        self._last_settled = settled

        if settled:
            self._debug_lock = True
            self._debug_lock = False

    def pullInHierarchy(self):
        """Attempts to pull a published hierarchy into the Abstract Map"""
        hierarchy_topic = rospy.get_param('hierarchy_topic', '/hierarchy')
        if next((t for t in rospy.get_published_topics()
                 if t[0] == hierarchy_topic), None) is not None:
            # Reconstruct the hierarchy
            hierarchy = pickle.loads(
                rospy.wait_for_message('/hierarchy',
                                       std_msgs.String,
                                       timeout=1.0).data)

            # Add the associated symbolic spatial information strings
            for h in hierarchy:
                if h[1] is not None:
                    ssi = "%s is in %s" % (h[0], h[1])
                    self._abstract_map.addSymbolicSpatialInformation(
                        ssi, None, immediate=True)
                    rospy.loginfo("Added symbolic spatial information: %s" %
                                  (ssi))

            # Because we pulled in an entire hierarchy, we should finish by
            # initialising the state of the entire network (this allows us to
            # layout the network WITH correct mass levels, whereas the adding
            # done above is not able to guarantee this...)
            self._abstract_map._spatial_layout.initialiseState()
            self._update_coem()
        else:
            rospy.logwarn("Hierarchy not available; continuing without")

    def spin(self):
        """Blocking function where the Abstract Map operates"""
        while not rospy.is_shutdown():
            self._abstract_map._spatial_layout.step()
        rospy.logerr("Exiting spin...")


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
            x, y, th = tools.poseMsgToXYTh(ros_pose)
            self.poses.append([x, y, math.cos(th), math.sin(th)])

        def meanPose(self):
            """Calculates the mean pose observed for a requested tag"""
            return (np.mean([x[0] for x in self.poses]),
                    np.mean([x[1] for x in self.poses]),
                    math.atan2(np.mean([x[3] for x in self.poses]),
                               np.mean([x[2] for x in self.poses])))
