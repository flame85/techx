
cmake_minimum_required(VERSION 2.4.6)
SET(CMAKE_CXX_FLAGS "-msse4 -O3")
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#SET(CMAKE_C_COMPILER "/usr/bin/colorgcc")
#SET(CMAKE_CXX_COMPILER "/usr/bin/colorgcc")

set(ROS_BUILD_TYPE RelWithDebInfo)
# Turn on higher warnings, because I'm hard-core
#set(ROS_COMPILE_FLAGS "-W -Wall -g -O3")
# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#find_package(PCL 1.5 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})
rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
include_directories(${CMAKE_CURRENT_BINARY_DIR} ${QT_QTOPENGL_INCLUDE_DIR})
LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})
SET(${CMAKE_LIBRARY_PATH} ${LIBRARY_OUTPUT_PATH})

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_executable(stereo_process src/stereo_processing.cpp)
#target_link_libraries(bumblebee_grabber dc1394 mypgrlibdcstereo)
#target_link_libraries(myBumblebee dc1394 raw1394 pgrlibdcstereo)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
