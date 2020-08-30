#include <list>
#include <string>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/PoseArray.h>

#include "abstract_map/symbol_mapping.h"

namespace humancues {

class TagInterpreter {
 public:
  TagInterpreter();

 private:
  ros::NodeHandle nh_, nh_private;
  ros::Subscriber sub_detections_;
  ros::Publisher pub_ssi_, pub_dbg_;
  tf::TransformListener tf_listener_;

  geometry_msgs::PoseArray dbg_;

  std::string xml_name_;
  std::list<SymbolMapping> mappings_;

  void callbackDetection(const apriltag_ros::AprilTagDetectionArray &message);
};

}  // namespace humancues
