cmake_minimum_required(VERSION 2.8)
project(turtlesim)
find_package(ROS COMPONENTS catkin roscpp roslib rosconsole std_srvs rostime roscpp_serialization)
catkin_project(${PROJECT_NAME})

find_package(Qt4 4.6 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
find_package(Boost COMPONENTS thread)

include_directories(include ${ROS_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${QT_INCLUDE_DIRS})

add_message_files(DIRECTORY msg FILES 
                  Color.msg Pose.msg Velocity.msg)
add_service_files(DIRECTORY srv FILES
                  Kill.srv
                  SetPen.srv
                  Spawn.srv
                  TeleportAbsolute.srv
                  TeleportRelative.srv)
generate_messages(DEPENDENCIES std_msgs std_srvs)

set(turtlesim_node_SRCS
  src/turtlesim.cpp
  src/turtle.cpp
  src/turtle_frame.cpp
)
set(turtlesim_node_HDRS
  include/turtlesim/turtle_frame.h
)

qt4_wrap_cpp(turtlesim_node_MOCS ${turtlesim_node_HDRS})


add_executable(turtlesim_node ${turtlesim_node_SRCS} ${turtlesim_node_MOCS})
target_link_libraries(turtlesim_node ${QT_LIBRARIES} ${ROS_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(turtlesim_node turtlesim_gencpp)

add_executable(turtle_teleop_key tutorials/teleop_turtle_key.cpp)
target_link_libraries(turtle_teleop_key ${ROS_LIBRARIES})
add_dependencies(turtle_teleop_key turtlesim_gencpp)
add_executable(draw_square tutorials/draw_square.cpp)
target_link_libraries(draw_square ${ROS_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(draw_square turtlesim_gencpp)
add_executable(mimic tutorials/mimic.cpp)
target_link_libraries(mimic ${ROS_LIBRARIES})
add_dependencies(mimic turtlesim_gencpp)

install(TARGETS turtlesim_node turtle_teleop_key draw_square mimic
        RUNTIME DESTINATION share/${PROJECT_NAME})
install(DIRECTORY images/
  DESTINATION share/turtlesim/images/
  FILES_MATCHING PATTERN "*.png"
  PATTERN ".svn" EXCLUDE)
