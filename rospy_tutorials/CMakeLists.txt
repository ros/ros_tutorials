cmake_minimum_required(VERSION 2.8.3)
project(rospy_tutorials)

find_package(catkin REQUIRED COMPONENTS message_generation rostest std_msgs)

add_message_files(DIRECTORY msg FILES Floats.msg HeaderString.msg)
add_service_files(DIRECTORY srv FILES AddTwoInts.srv BadTwoInts.srv)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

foreach(T
    test/test-add-two-ints.launch
    test/test-peer-subscribe-notify.launch
    test/test-add-two-ints-with-roscpp-server.launch
    test/test-talker-listener.launch
    test/test-talker-listener-with-roscpp.launch
    test/test-on-shutdown.launch
    test/test-connection-header.launch)
  add_rostest(${T})
endforeach()

# do not wildcard install files since the root folder of the package will contain a debian folder for releasing
foreach(T
    001_talker_listener
    002_headers
    003_listener_with_user_data
    004_listener_subscribe_notify
    005_add_two_ints
    006_parameters
    007_connection_header
    008_on_shutdown
    009_advanced_publish)
  install(DIRECTORY ${T}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS)
endforeach()
