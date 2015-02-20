/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*  Author: Okke Hendriks
*  Date  : 2013/12/02
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
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <std_msgs/Float64.h>

#include "rose_common/common.hpp"
#include "rose_conversions/conversions.hpp"
#include "opteq_wheelunits_01/wheel_unit.hpp"

namespace gazebo
{   
class WheelUnitController : public ModelPlugin
{
public: 
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
  void OnUpdate(const common::UpdateInfo & /*info*/);
  

private:  
  void CB_SetRequestedCasterPos(const std_msgs::Float64::ConstPtr& msg);     
  void CB_SetRequestedWheelVel(const std_msgs::Float64::ConstPtr& msg);
  
  // Publishers
  ros::Publisher pub_enc_caster_pos_;
  ros::Publisher pub_enc_caster_vel_;
  ros::Publisher pub_enc_wheel_pos_;
  ros::Publisher pub_enc_wheel_vel_;
  
  // Subscribers
  ros::Subscriber sub_caster_pos_;
  ros::Subscriber sub_wheel_vel_;
  
  // Gazebo     
  physics::ModelPtr         model_;
  sdf::ElementPtr           sdf_;
  gazebo::physics::JointPtr caster_joint_;    
  gazebo::physics::JointPtr wheel_joint_;          
  event::ConnectionPtr      updateConnection_;
  
  // Control
  std::string               controller_name_;
  std::string               caster_joint_string_;
  std::string               wheel_joint_string_;
  common::Time              curr_time_;
  common::Time              prev_time_;    
  common::PID               PID_caster_;
  common::PID               PID_wheel_;
  
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
}

#endif // WHEELUNITCONTROLLER_H

