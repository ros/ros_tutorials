cmake_minimum_required(VERSION 2.8.3)
project(rospy_tutorials)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(DIRECTORY msg FILES Floats.msg HeaderString.msg)
add_service_files(DIRECTORY srv FILES AddTwoInts.srv BadTwoInts.srv)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

# do not wildcard install files since the root folder of the package will contain a debian folder for releasing
foreach(T
    001_talker_listener/listener
    001_talker_listener/listener.py
    001_talker_listener/talker
    001_talker_listener/talker.py)
  catkin_install_python(PROGRAMS ${T}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/001_talker_listener)
endforeach()

foreach(T
    002_headers/listener_header.py
    002_headers/talker_header.py)
  catkin_install_python(PROGRAMS ${T}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/002_headers)
endforeach()

catkin_install_python(PROGRAMS 003_listener_with_user_data/listener_with_user_data.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/003_listener_with_user_data)

catkin_install_python(PROGRAMS 004_listener_subscribe_notify/listener_subscribe_notify.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/004_listener_subscribe_notify)

foreach(T
    005_add_two_ints/add_two_ints_client
    005_add_two_ints/add_two_ints_server)
  catkin_install_python(PROGRAMS ${T}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/005_add_two_ints)
endforeach()

catkin_install_python(PROGRAMS 006_parameters/param_talker.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/006_parameters)

foreach(T
    007_connection_header/client_connection_header.py
    007_connection_header/listener_connection_header.py
    007_connection_header/server_connection_header.py
    007_connection_header/talker_connection_header.py)
  catkin_install_python(PROGRAMS ${T}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/007_connection_header)
endforeach()

catkin_install_python(PROGRAMS 008_on_shutdown/publish_on_shutdown.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/008_on_shutdown)

catkin_install_python(PROGRAMS 009_advanced_publish/advanced_publish.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/009_advanced_publish)

install(DIRECTORY ./
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch" PATTERN "README")

if(CATKIN_ENABLE_TESTING)
  find_package(rostest)
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
endif()
