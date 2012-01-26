include_directories(${ROS_INCLUDE_DIRS})
add_executable(listener listener.cpp )
target_link_libraries(listener ${ROS_LIBRARIES}) 
install(TARGETS listener RUNTIME DESTINATION share/roscpp_tutorials/bin)

