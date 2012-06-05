project(rospy_tutorials)
find_package(catkin REQUIRED)
find_package(ROS REQUIRED COMPONENTS genmsg)
catkin_project(${PROJECT_NAME})

add_message_files(DIRECTORY msg FILES Floats.msg HeaderString.msg)
add_service_files(DIRECTORY srv FILES AddTwoInts.srv BadTwoInts.srv)

generate_messages(DEPENDENCIES std_msgs)

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

install(DIRECTORY .
  DESTINATION share/rospy_tutorials
  USE_SOURCE_PERMISSIONS
  PATTERN CMakeLists.txt EXCLUDE
  PATTERN catkin.cmake EXCLUDE
  PATTERN .svn EXCLUDE
  )
