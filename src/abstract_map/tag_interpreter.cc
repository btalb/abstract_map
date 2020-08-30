#include "abstract_map/tag_interpreter.h"

#include <ros/console.h>

#include <abstract_map/SymbolicSpatialInformation.h>

namespace humancues {

TagInterpreter::TagInterpreter() : nh_(), nh_private("~"), tf_listener_() {
  // Read in all parameters
  nh_private.param("xml_filename", xml_name_, std::string(""));

  // Attempt to load the requested experiment from the XML file
  // (it is ok to fail, but let the user know)
  if (!humancues::SymbolMapping::parseListFromXML(xml_name_.c_str(),
                                                  &mappings_))
    ROS_ERROR("Loading mappings failed (attempted to parse file: %s)",
              xml_name_.c_str());
  ROS_INFO("Succesfully loaded %ld mappings from file: %s", mappings_.size(),
           xml_name_.c_str());

  // Set up the publishers and subscribers
  sub_detections_ = nh_.subscribe("/tag_detections", 10,
                                  &TagInterpreter::callbackDetection, this);
  pub_ssi_ = nh_.advertise<abstract_map::SymbolicSpatialInformation>(
      "/symbolic_spatial_info", 10);
  pub_dbg_ = nh_.advertise<geometry_msgs::PoseArray>("/debug_reading", 10);
}

const SymbolMapping *findMapping(int id, const std::list<SymbolMapping> &m) {
  for (std::list<SymbolMapping>::const_iterator it = m.begin(); it != m.end();
       ++it) {
    if (it->tag_id_ == id) return &*it;
  }
  return NULL;
}

void TagInterpreter::callbackDetection(
    const apriltag_ros::AprilTagDetectionArray &message) {
  // Exit if there are no detections
  if (message.detections.empty()) return;

  // Loop through each of the detections, attempting to interpret them
  apriltag_ros::AprilTagDetectionArray::_detections_type::const_iterator it;
  for (it = message.detections.begin(); it != message.detections.end(); ++it) {
    const SymbolMapping *sm = findMapping(it->id[0], mappings_);
    if (sm) {
      // Abondon the message if the transform lookup fails
      try {
        geometry_msgs::PoseStamped mapPose, inPose;
        inPose.header = it->pose.header;
        inPose.pose = it->pose.pose.pose;
        tf_listener_.transformPose("map", inPose, mapPose);

        dbg_.header.stamp = ros::Time::now();
        dbg_.header.frame_id = "map";
        dbg_.poses.push_back(mapPose.pose);
        pub_dbg_.publish(dbg_);

        abstract_map::SymbolicSpatialInformation msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";
        msg.ssi = sm->text_;
        msg.tag_id = it->id[0];
        msg.location = mapPose.pose;
        pub_ssi_.publish(msg);
      } catch (const std::exception &e) {
        ROS_INFO("Message read failed with transform exception: %s", e.what());
      }
    }
  }
}

}  // namespace humancues
