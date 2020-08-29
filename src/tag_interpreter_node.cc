#include <ros/ros.h>

#include "abstract_map/tag_interpreter.h"

int main(int argc, char **argv) {
  // Initialise the node
  ros::init(argc, argv, "experiment_publisher");

  humancues::TagInterpreter ti;

  ros::spin();

  return 0;
}
