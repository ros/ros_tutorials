if(CATKIN)
  include(catkin.cmake)
  return()
endif()

rosbuild_add_executable(listener listener.cpp)
