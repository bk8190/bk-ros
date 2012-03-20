cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE DEBUG)

rosbuild_init()
rosbuild_add_boost_directories()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(path_checker    src/path_checker.cpp)
rosbuild_add_library(feeder_thread   src/feeder_thread.cpp)
rosbuild_add_library(planning_thread src/planning_thread.cpp)

rosbuild_add_executable(bk_planner src/bk_planner.cpp)
rosbuild_add_executable(covariance_faker src/covariance_faker.cpp)

target_link_libraries(planning_thread SegmentVisualizer)
target_link_libraries(feeder_thread   SegmentVisualizer)

target_link_libraries(bk_planner path_checker)
target_link_libraries(bk_planner segment_lib)
target_link_libraries(bk_planner SegmentVisualizer)
target_link_libraries(bk_planner planning_thread)
target_link_libraries(bk_planner feeder_thread)

rosbuild_link_boost  (bk_planner thread signals system)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
