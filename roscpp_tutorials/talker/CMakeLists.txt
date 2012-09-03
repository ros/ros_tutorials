include_directories(${catkin_INCLUDE_DIRS})
add_executable(talker talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
install(TARGETS talker
  RUNTIME DESTINATION ${PROJECT_LIBEXEC_DESTINATION})
