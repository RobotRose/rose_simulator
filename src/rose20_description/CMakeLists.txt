cmake_minimum_required (VERSION 2.4.6)
project(rose20_description)

find_package(catkin REQUIRED COMPONENTS 
  gazebo_ros
  xacro 
)


add_custom_command(
	  OUTPUT ${PROJECT_SOURCE_DIR}/rose20/rose20.urdf
      COMMAND rosrun xacro xacro.py ${PROJECT_SOURCE_DIR}/rose20/rose20.xacro > ${PROJECT_SOURCE_DIR}/rose20/rose20.urdf)

add_custom_command(
	  OUTPUT ${PROJECT_SOURCE_DIR}/rose20/rose20.sdf
      COMMAND gzsdf print ${PROJECT_SOURCE_DIR}/rose20/rose20.urdf > ${PROJECT_SOURCE_DIR}/rose20/rose20.sdf
  	  DEPENDS ${PROJECT_SOURCE_DIR}/rose20/rose20.urdf)

add_custom_target(generate_urdf ALL DEPENDS ${PROJECT_SOURCE_DIR}/rose20/rose20.urdf )
add_custom_target(generate_sdf ALL DEPENDS ${PROJECT_SOURCE_DIR}/rose20/rose20.sdf)
