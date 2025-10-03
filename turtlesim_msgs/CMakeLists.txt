cmake_minimum_required(VERSION 3.20)

project(turtlesim_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  action/RotateAbsolute.action
)

set(action_files
  msg/Color.msg
  msg/Pose.msg
)

set(srv_files
  srv/Kill.srv
  srv/SetPen.srv
  srv/Spawn.srv
  srv/TeleportAbsolute.srv
  srv/TeleportRelative.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
