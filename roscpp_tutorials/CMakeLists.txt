cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_catkinize()
rosbuild_init()
rosbuild_gensrv()
include_directories(${PROJECT_SOURCE_DIR}/srv/cpp)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

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
    time_api
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
    parameters)
  add_subdirectory(${dir})
endforeach()
