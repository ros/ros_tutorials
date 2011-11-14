cmake_minimum_required(VERSION 2.8)
project(quux_user)

# set(CMAKE_FIND_ROOT_PATH /tmp/sandbox)

find_package(rostime)
include_directories(${rostime_INCLUDE_DIRS})

find_package(rosconsole)
include_directories(${rosconsole_INCLUDE_DIRS})

find_package(XmlRpc)
include_directories(${XmlRpc_INCLUDE_DIRS})

find_package(cpp_common)
include_directories(${cpp_common_INCLUDE_DIRS})

find_package(roscpp_traits)
include_directories(${roscpp_traits_INCLUDE_DIRS})

find_package(roscpp_serialization)
include_directories(${roscpp_serialization_INCLUDE_DIRS})

find_package(std_msgs)
include_directories(${std_msgs_INCLUDE_DIRS})

find_package(roscpp)
include_directories(${roscpp_INCLUDE_DIRS})

add_executable(talker-exec
  talker.cpp
  )

target_link_libraries(talker-exec ${roscpp_LIBRARIES}) 

