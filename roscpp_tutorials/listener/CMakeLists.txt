include_directories(${catkin_INCLUDE_DIRS})
add_executable(listener listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
install(TARGETS listener
  RUNTIME DESTINATION ${PROJECT_LIBEXEC_DESTINATION})
