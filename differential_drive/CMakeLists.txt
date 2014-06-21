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
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

option(BUILD_SDL_STUFF "Build SDL stuff" ON)
if (BUILD_SDL_STUFF)
	rosbuild_add_executable(KeyboardEvent src/KeyboardEvent.cpp)
	target_link_libraries(KeyboardEvent SDL)
endif (BUILD_SDL_STUFF)


rosbuild_add_executable(bezier_traj 
				src/bezier_traj.cpp
				src/Bezier.cpp)

rosbuild_add_executable(mode1 src/mode1.cpp)
rosbuild_add_executable(mode2 src/mode2.cpp)

rosbuild_add_executable(KeyboardControl src/KeyboardControl.cpp)
# rosbuild_add_executable(HumanInterface src/HumanInterface2.cpp)

rosbuild_add_executable(goto src/goto.cpp)

rosbuild_add_executable(openLoop src/openLoop.cpp)




