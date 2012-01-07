cmake_minimum_required(VERSION 2.8)
project(roscpp_tutorials)
find_package(Boost COMPONENTS date_time thread)
find_package(ROS COMPONENTS catkin genmsg roscpp)
include_directories(${ROS_INCLUDE_DIRS})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)

add_service_files(DIRECTORY srv FILES TwoInts.srv)
generate_messages(DEPENDENCIES std_msgs)


macro(rostutorial T)
  add_executable(${T} ${T}/${T}.cpp)
  find_package(ROS COMPONENTS catkin genmsg rostime roscpp rosconsole roscpp_serialization)
  target_link_libraries(${T} ${ROS_LIBRARIES} ${Boost_LIBRARIES})
  add_dependencies(${T} roscpp_tutorials_gencpp)
  install(TARGETS ${T} RUNTIME DESTINATION share/roscpp_tutorials/bin)
endmacro()

foreach(dir
    listener
    notify_connect
    talker
    babbler
    add_two_ints_client
    add_two_ints_server
    add_two_ints_server_class
    anonymous_listener
    listener_with_userdata
    listener_multiple
    listener_threaded_spin
    listener_async_spin
    # listener_single_message
    listener_with_tracked_object
    listener_unreliable
    listener_class
    node_handle_namespaces
    custom_callback_processing
    timers
    parameters
    )
  rostutorial(${dir})
endforeach()

add_executable(time_api_sleep time_api/sleep/sleep.cpp)
target_link_libraries(time_api_sleep ${ROS_LIBRARIES})
install(TARGETS time_api_sleep RUNTIME DESTINATION share/roscpp_tutorials/bin)

catkin_project(roscpp_tutorials)
