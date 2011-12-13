cmake_minimum_required(VERSION 2.8)
project(ros_tutorials)

find_package(catkin)
find_package(ROS REQUIRED COMPONENTS roscpp_serialization std_msgs sensor_msgs roscpp rosconsole rostime cpp_common roscpp_traits)

add_subdirectory(roscpp_tutorials)

catkin_package(roscpp_tutorials)
