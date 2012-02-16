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

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_library(${PROJECT_NAME} src/segment_lib.cpp)
rosbuild_add_library(SegmentVisualizer src/SegmentVisualizer.cpp)
rosbuild_add_executable(segment_test src/segment_test.cpp)

target_link_libraries(${PROJECT_NAME} SegmentVisualizer)

target_link_libraries(segment_test ${PROJECT_NAME})
target_link_libraries(segment_test SegmentVisualizer)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#include(CheckIncludeFiles)
#CHECK_INCLUDE_FILES(segment_lib/segment_lib.h HAVE_SEGMENT_LIB_H)
#if(NOT HAVE_SEGMENT_LIB_H)
#  message(FATAL_ERROR "Can't proceed without <segment_lib/segment_lib.h>")
#endif(NOT HAVE_SEGMENT_LIB_H)
