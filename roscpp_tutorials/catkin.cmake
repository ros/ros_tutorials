cmake_minimum_required(VERSION 2.8)
project(roscpp_tutorials)
find_package(catkin)

add_subdirectory(talker)
add_subdirectory(listener)

install_cmake_infrastructure(roscpp_tutorials)

