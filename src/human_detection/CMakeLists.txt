cmake_minimum_required(VERSION 3.0.2)
project(human_detection)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  tf
  roscpp
  sensor_msgs
  visualization_msgs
  tf_conversions
)

find_package(pcl_conversions REQUIRED)


catkin_package(
  LIBRARIES human_detection
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(clasters_node src/example_planarsegmentation.cpp)
target_link_libraries(clasters_node ${catkin_LIBRARIES}  ${PCL_LIBRARIES} ${pcl_conversions_INCLUDE_DIRS})
