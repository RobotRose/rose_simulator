cmake_minimum_required (VERSION 2.4.6)
project(rose20_description)

find_package(catkin REQUIRED COMPONENTS 
  gazebo_ros
  xacro 
)


# cyton_gamma_1500
add_custom_command(
	  OUTPUT ${PROJECT_SOURCE_DIR}/cyton_gamma_1500/new/urdf/cyton_gamma_1500.urdf
      COMMAND rosrun xacro xacro.py ${PROJECT_SOURCE_DIR}/cyton_gamma_1500/new/urdf/cyton_gamma/cyton_gamma.urdf.xacro > ${PROJECT_SOURCE_DIR}/cyton_gamma_1500/new/urdf/cyton_gamma_1500.urdf)

# add_custom_command(
# 	  OUTPUT ${PROJECT_SOURCE_DIR}/cyton_gamma_1500/new/sdf/cyton_gamma_1500.sdf
#       COMMAND gzsdf print ${PROJECT_SOURCE_DIR}/cyton_gamma_1500/new/urdf/cyton_gamma_1500.urdf > ${PROJECT_SOURCE_DIR}/cyton_gamma_1500/new/sdf/cyton_gamma_1500.sdf
#   	  DEPENDS ${PROJECT_SOURCE_DIR}/cyton_gamma_1500/new/urdf/cyton_gamma_1500.urdf)

add_custom_target(generate_arm_urdf ALL DEPENDS ${PROJECT_SOURCE_DIR}/cyton_gamma_1500/new/urdf/cyton_gamma_1500.urdf)
#add_custom_target(generate_arm_sdf ALL DEPENDS ${PROJECT_SOURCE_DIR}/cyton_gamma_1500/new/sdf/cyton_gamma_1500.sdf)

# rose
add_custom_command(
	  OUTPUT ${PROJECT_SOURCE_DIR}/rose20/rose20.urdf
      COMMAND rosrun xacro xacro.py ${PROJECT_SOURCE_DIR}/rose20/rose20.xacro > ${PROJECT_SOURCE_DIR}/rose20/rose20.urdf)

add_custom_command(
	  OUTPUT ${PROJECT_SOURCE_DIR}/rose20/rose20.sdf
      COMMAND gzsdf print ${PROJECT_SOURCE_DIR}/rose20/rose20.urdf > ${PROJECT_SOURCE_DIR}/rose20/rose20.sdf
  	  DEPENDS ${PROJECT_SOURCE_DIR}/rose20/rose20.urdf)

add_custom_target(generate_rose20_urdf ALL DEPENDS ${PROJECT_SOURCE_DIR}/rose20/rose20.urdf )
add_custom_target(generate_rose20_sdf ALL DEPENDS ${PROJECT_SOURCE_DIR}/rose20/rose20.sdf)
