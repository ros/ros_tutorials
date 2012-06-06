cmake_minimum_required(VERSION 2.8)
project(ros_tutorials)

find_package(catkin REQUIRED)
find_package(ROS REQUIRED COMPONENTS
  roscpp_serialization
  std_msgs
  roscpp
  rosconsole
  rostime
  cpp_common
  roscpp_traits
  xmlrpcpp
  genmsg
  rostest
  )
catkin_stack()

foreach(subdir
    rospy_tutorials
    roscpp_tutorials
    turtlesim
    )
  add_subdirectory(${subdir})
endforeach()

# install all manifests
install_matching_to_share(manifest.xml)

install(FILES stack.xml
        DESTINATION share/ros_tutorials)
