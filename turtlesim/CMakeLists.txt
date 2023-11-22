cmake_minimum_required(VERSION 3.5)
project(turtlesim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include ${Qt5Widgets_INCLUDE_DIRS})

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/RotateAbsolute.action"
  "msg/Color.msg"
  "msg/Pose.msg"
  "srv/Kill.srv"
  "srv/SetPen.srv"
  "srv/Spawn.srv"
  "srv/TeleportAbsolute.srv"
  "srv/TeleportRelative.srv")

qt5_wrap_cpp(turtlesim_node_MOCS include/turtlesim/turtle_frame.hpp)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

add_executable(turtlesim_node
  src/turtlesim.cpp
  src/turtle.cpp
  src/turtle_frame.cpp
  ${turtlesim_node_MOCS}
)
target_link_libraries(turtlesim_node PRIVATE
  ament_index_cpp::ament_index_cpp
  ${cpp_typesupport_target}
  ${geometry_msgs_TARGETS}
  Qt5::Widgets
  ${rcl_interfaces_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${std_srvs_TARGETS}
)

add_executable(turtle_teleop_key tutorials/teleop_turtle_key.cpp)
target_link_libraries(turtle_teleop_key PRIVATE
  "${cpp_typesupport_target}"
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
)

add_executable(draw_square tutorials/draw_square.cpp)
target_link_libraries(draw_square PRIVATE
  "${cpp_typesupport_target}"
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
  ${std_srvs_TARGETS}
)

add_executable(mimic tutorials/mimic.cpp)
target_link_libraries(mimic PRIVATE
  "${cpp_typesupport_target}"
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS turtlesim_node turtle_teleop_key draw_square mimic
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY images
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.png" PATTERN "*.svg")

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
