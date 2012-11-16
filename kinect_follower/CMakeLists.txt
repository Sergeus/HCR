cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


#Get sources
file(GLOB TF_SOURCES
    "src/tf_broadcaster/*.h" "src/tf_broadcaster/*.hxx" "src/tf_broadcaster/*.cpp" )
file(GLOB FOLLOWER_SOURCES
    "src/kinect_follower/*.h" "src/kinect_follower/*.hxx" "src/kinect_follower/*.cpp" )

#Build everything
rosbuild_add_executable("tf_broadcaster" ${TF_SOURCES} )
rosbuild_add_executable(${PROJECT_NAME} ${FOLLOWER_SOURCES})

