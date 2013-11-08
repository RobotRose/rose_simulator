cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()


         
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(~/rosepc1/rosbee_control_wireless/msg_gen/cpp/include/)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})


add_library(LiftController SHARED LiftController.cpp)
target_link_libraries(LiftController ${GAZEBO_libraries})

set_target_properties(LiftController PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/../)

