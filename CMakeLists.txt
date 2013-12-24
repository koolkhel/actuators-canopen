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

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

include_directories("/usr/local/include/canfestival")

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(actuators canlib pthread canfestival)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(actuators 
	src/actuators_canopen/actuators.c
	src/actuators_canopen/main-actuators.cpp
	src/actuators_canopen/actuators-ros.cpp
	src/actuators_canopen/canopen-util.cpp
	src/actuators_canopen/matlab-connector.cpp
	)
#rosbuild_add_executable(emulator 
#	src/actuators_canopen/main-emulator.cpp
#	src/actuators_canopen/canopen-util.cpp
#	src/actuators_canopen/emulator.c)
set_target_properties(actuators PROPERTIES COMPILE_FLAGS " -falign-functions=8 -fsplit-stack -fstack-check --param ssp-buffer-size=8192")
#set_target_properties(actuators PROPERTIES COMPILE_FLAGS " -falign-functions=8 -fsplit-stack -fconserve-stack")
target_link_libraries(actuators canlib rt canfestival canfestival_unix dl rt)
#arget_link_libraries(emulator canlib rt canfestival canfestival_unix dl rt)
#target_link_libraries(emulator canlib rt)
