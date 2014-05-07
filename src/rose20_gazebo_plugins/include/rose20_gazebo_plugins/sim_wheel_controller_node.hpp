#ifndef SIM_WHEEL_CONTROLLER__NODE_HPP
#define SIM_WHEEL_CONTROLLER__NODE_HPP

#include <iostream>
#include <stdio.h>

#include <map>

#include "rose20_common/common.hpp"
#include "rose20_common/ros_name.hpp"
#include "rose20_common/wheel_unit.hpp"
#include "roscomm/wheelunit_states.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


#include "sim_wheel_controller.hpp"

void CB_initialpose(const geometry_msgs::PoseWithCovarianceStamped& initial_pose);

#endif // SIM_WHEEL_CONTROLLER__NODE_HPP
