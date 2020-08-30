cmake_minimum_required(VERSION 2.8.3)
project(abstract_map)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Find all Catkin dependencies
find_package(catkin REQUIRED COMPONENTS
  apriltag_ros
  cmake_modules
  geometry_msgs
  message_generation
  rospy
  std_msgs
  )

find_package(TinyXML REQUIRED)

# Uncomment this if the package has a setup.py. This macro ensures
# modules and global scripts declared therein get installed
# See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

# Generate messages in the 'msg' folder
add_message_files(FILES SymbolicSpatialInformation.msg)

# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

# Declare the Catkin package
catkin_package(
  DEPENDS TinyXML
  )

# Compile source code
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
  )

add_executable(tag_interpreter_node
  ./src/tag_interpreter_node.cc
  ./src/abstract_map/symbol_mapping.cc
  ./src/abstract_map/tag_interpreter.cc
  )

target_link_libraries(tag_interpreter_node
  ${catkin_LIBRARIES}
  ${TinyXML_LIBRARIES}
  )

# Mark executable scripts (Python etc.) for installation
# in contrast to setup.py, you can choose the destination
install(PROGRAMS
  ./nodes/abstract_map_node
  ./nodes/ssi_commentator
  ./nodes/visualiser
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
