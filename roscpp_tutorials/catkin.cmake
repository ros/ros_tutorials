cmake_minimum_required(VERSION 2.8)
project(roscpp_tutorials)
find_package(catkin)

foreach(subdir
    talker
#    listener
    )
  add_subdirectory(${subdir})
endforeach()

catkin_package(roscpp_tutorials)
