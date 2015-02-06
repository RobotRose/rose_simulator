cmake_minimum_required (VERSION 2.4.6)
project(rose20_gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS 
  gazebo_ros
  roscomm 
  rose_common
  rose_converions
  rose20_platform
  server_multiple_client
  shared_variables
)

set(CMAKE_CXX_FLAGS "-std=c++0x -g") # TODO OH: -g is DEBUG FLAG

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
    INCLUDE_DIRS 
      include
    LIBRARIES 
      LiftController 
      NeckController 
      WheelUnitController
    CATKIN_DEPENDS 
      roscomm 
      rose_common 
      rose_converions
      rose20_platform 
      server_multiple_client
      shared_variables
    DEPENDS 
      gazebo_ros
)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${gazebo_LIBS})

add_library(LiftController SHARED src/LiftController.cpp) 
target_link_libraries(LiftController ${gazebo_LIBS} ${catkin_LIBRARIES})

add_library(NeckController SHARED src/NeckController.cpp)
target_link_libraries(NeckController ${gazebo_LIBS} ${catkin_LIBRARIES})

add_library(WheelUnitController SHARED src/WheelUnitController.cpp)
target_link_libraries(WheelUnitController ${gazebo_LIBS} ${catkin_LIBRARIES})

add_dependencies(LiftController 			roscomm rose_common)
add_dependencies(NeckController 			roscomm rose_common)
add_dependencies(WheelUnitController 		roscomm rose_common)



add_executable(SimWheelController 		${PROJECT_SOURCE_DIR}/src/sim_joint.cpp
										${PROJECT_SOURCE_DIR}/src/sim_wheel_unit_controller.cpp
										${PROJECT_SOURCE_DIR}/src/sim_wheel_controller.cpp
										${PROJECT_SOURCE_DIR}/src/sim_wheel_controller_node.cpp
										)

target_link_libraries(SimWheelController ${gazebo_LIBS} ${catkin_LIBRARIES})
