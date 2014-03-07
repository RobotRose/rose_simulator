/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*  Author: Okke Hendriks
*  Date  : 2014/03/07
*     - File created.
*
* Description:
*  Contains simulation controller components for a wheelunit
* 
***********************************************************************************/
#ifndef WHEELUNITCONTROLLER_H
#define WHEELUNITCONTROLLER_H

#include <stdio.h>
#include <math.h>

#include <boost/bind.hpp>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include "rose20_common/common.hpp"
#include "rose20_common/PID/PID.hpp"

#include "wheel_controller/wheel_unit.hpp"

#include "rose20_gazebo_plugins/sim_joint.hpp"

using namespace std;
using namespace rose20_common;

class SimWheelUnitController
{
public: 
  SimWheelUnitController();
  SimWheelUnitController(ros::NodeHandle n, string wheel_unit_name, int wheel_direction);
  ~SimWheelUnitController();
  void update();

  std::string               wheel_unit_name_;
  

private:  
  void CB_SetRequestedCasterPos(const std_msgs::Float64::ConstPtr& msg);     
  void CB_SetRequestedWheelVel(const std_msgs::Float64::ConstPtr& msg);

  ros::NodeHandle n_;
  
  // Publishers
  ros::Publisher pub_enc_caster_pos_;
  ros::Publisher pub_enc_caster_vel_;
  ros::Publisher pub_enc_wheel_pos_;
  ros::Publisher pub_enc_wheel_vel_;
  
  // Subscribers
  ros::Subscriber sub_caster_pos_;
  ros::Subscriber sub_wheel_vel_;  

  // Control
  
  std::string               caster_joint_string_;
  std::string               wheel_joint_string_;

  SimJoint*                 caster_joint_;    
  SimJoint*                 wheel_joint_; 

  ros::Time                 cur_time_;
  ros::Time                 prev_time_;    
  rose20_common::PID        PID_caster_;
  rose20_common::PID        PID_wheel_;
  
  // State
  float   cur_caster_pos_;
  float   cur_caster_vel_;
  float   cur_wheel_pos_;
  float   cur_wheel_vel_;
  float   req_caster_pos_;
  float   req_wheel_vel_;
  float   wheel_direction_;
  
  std::string   caster_namespace_;
  std::string   wheel_namespace_;
  
};

#endif // WHEELUNITCONTROLLER_H

