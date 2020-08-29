#!/usr/bin/env python

import cPickle as pickle
import graphviz as gv
import os
import rospy
import sys

import std_msgs.msg as std_msgs

import human_cues_tag_reader as hctr


def main(fn):
    # Get the hierarchy either by pulling from a ROS topic, or directly from
    # a file
    if fn is None:
        rospy.init_node('hierarchy_viewer')
        topic = rospy.get_param('hierarchy_topic',
                                hctr.DEFAULT_HIERARCHY_TOPIC)
        hierarchy = pickle.loads(
            rospy.wait_for_message(topic, std_msgs.String).data)
    else:
        hierarchy = hctr.loadHierarchy(fn)

    # Visualise the hierarchy
    hctr.viewHierarchy(hierarchy)


if __name__ == '__main__':
    if len(sys.argv) == 2:
        filename = os.path.abspath(sys.argv[1])
    elif len(sys.argv) == 1:
        filename = None
    else:
        raise ValueError('Received too many arguments (max 1 accepted)')
    main(filename)
