#ifndef SIM_WHEEL_CONTROLLER__NODE_HPP
#define SIM_WHEEL_CONTROLLER__NODE_HPP

#include <iostream>
#include <stdio.h>

#include <map>

#include "rose_common/common.hpp"
#include "rose_converions/converions.hpp"
#include "ros_name/ros_name.hpp"
#include "opteq_wheelunits_01/wheel_unit.hpp"
#include "rose20_platform/wheelunit_states.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>


#include "sim_wheel_controller.hpp"

void CB_initialpose(const geometry_msgs::PoseWithCovarianceStamped& initial_pose);

#endif // SIM_WHEEL_CONTROLLER__NODE_HPP
