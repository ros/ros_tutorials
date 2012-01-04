cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()
rosbuild_gensrv()

find_package(wxWidgets REQUIRED)
include(${wxWidgets_USE_FILE})
include_directories( ${wxWidgets_INCLUDE_DIRS} )

rosbuild_add_boost_directories()
rosbuild_add_executable(turtlesim_node src/turtlesim.cpp src/turtle.cpp src/turtle_frame.cpp)
rosbuild_link_boost(turtlesim_node thread)
target_link_libraries(turtlesim_node ${wxWidgets_LIBRARIES})

rosbuild_add_executable(turtle_teleop_key tutorials/teleop_turtle_key.cpp)
rosbuild_add_executable(draw_square tutorials/draw_square.cpp)
rosbuild_add_executable(mimic tutorials/mimic.cpp)
