cmake_minimum_required(VERSION 2.8)
project(turtlesim)
find_package(ROS COMPONENTS catkin roscpp roslib rosconsole std_srvs rostime roscpp_serialization)
catkin_project(${PROJECT_NAME})

find_package(wxWidgets REQUIRED)
include(${wxWidgets_USE_FILE})
find_package(Boost COMPONENTS thread)

include_directories(include ${ROS_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${wxWidgets_INCLUDE_DIRS})

add_message_files(DIRECTORY msg FILES 
                  Color.msg Pose.msg Velocity.msg)
add_service_files(DIRECTORY srv FILES
                  Kill.srv
                  SetPen.srv
                  Spawn.srv
                  TeleportAbsolute.srv
                  TeleportRelative.srv)
generate_messages(DEPENDENCIES std_msgs std_srvs)


add_executable(turtlesim_node src/turtlesim.cpp src/turtle.cpp src/turtle_frame.cpp)
target_link_libraries(turtlesim_node ${wxWidgets_LIBRARIES} ${ROS_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(turtlesim_node turtlesim_gencpp)

add_executable(turtle_teleop_key tutorials/teleop_turtle_key.cpp)
target_link_libraries(turtle_teleop_key ${wxWidgets_LIBRARIES} ${ROS_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(turtle_teleop_key turtlesim_gencpp)
add_executable(draw_square tutorials/draw_square.cpp)
target_link_libraries(draw_square ${wxWidgets_LIBRARIES} ${ROS_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(draw_square turtlesim_gencpp)
add_executable(mimic tutorials/mimic.cpp)
target_link_libraries(mimic ${wxWidgets_LIBRARIES} ${ROS_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(mimic turtlesim_gencpp)

install(TARGETS turtlesim_node turtle_teleop_key draw_square mimic
        RUNTIME DESTINATION share/${PROJECT_NAME})
