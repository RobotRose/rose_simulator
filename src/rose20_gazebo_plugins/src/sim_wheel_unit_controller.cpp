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

#include "rose20_gazebo_plugins/sim_wheel_unit_controller.hpp"


SimWheelUnitController::SimWheelUnitController()
{

}

SimWheelUnitController::SimWheelUnitController(ros::NodeHandle n, string wheel_unit_name, int wheel_direction) 
{
  n_                    = n;
  wheel_unit_name_      = wheel_unit_name;    
  caster_namespace_     = wheel_unit_name_ + "_caster";
  wheel_namespace_      = wheel_unit_name_ + "_wheel";
  wheel_direction_      = wheel_direction;

  caster_joint_         = new SimJoint(2.0, 10.0, 0.0, 250.0);
  wheel_joint_          = new SimJoint(4.0, 10.0, 0.0, 250.0);

  // Initialize caster & wheel PID controllers
  PID_caster_.initialize(200.0, 50.0, 0.0, -200.0, 200.0, -150.0, 150.0);
  PID_wheel_.initialize(200.0, 50.0, 0.0, -200.0, 200.0, -250.0, 250.0);

  // Publishers
  pub_enc_caster_pos_    = n_.advertise<std_msgs::Int32>("/wheel_unit/" + caster_namespace_ + "/enc_pos", 5);
  pub_enc_caster_vel_    = n_.advertise<std_msgs::Int32>("/wheel_unit/" + caster_namespace_ + "/enc_vel", 5);
  pub_enc_wheel_pos_     = n_.advertise<std_msgs::Int32>("/wheel_unit/" + wheel_namespace_ + "/enc_pos", 5);
  pub_enc_wheel_vel_     = n_.advertise<std_msgs::Int32>("/wheel_unit/" + wheel_namespace_ + "/enc_vel", 5);
  
  // Subscribers
  sub_caster_pos_   = n_.subscribe("sim_wheel_controller/" + caster_namespace_ + "/req_pos", 1, &SimWheelUnitController::CB_SetRequestedCasterPos, this);
  sub_wheel_vel_    = n_.subscribe("sim_wheel_controller/" + wheel_namespace_  + "/req_vel", 1, &SimWheelUnitController::CB_SetRequestedWheelVel, this);
      
  cur_caster_vel_ = 0.0;
  cur_caster_pos_ = 0.0;
  cur_wheel_vel_  = 0.0;
  req_caster_pos_ = 0.0; 
  req_wheel_vel_  = 0.0;

  caster_joint_->pos_   = cur_caster_pos_;
  caster_joint_->vel_   = cur_caster_vel_; 
  wheel_joint_->pos_    = cur_wheel_pos_;
  wheel_joint_->vel_    = cur_wheel_vel_;

}

SimWheelUnitController::~SimWheelUnitController()
{
  
}


void SimWheelUnitController::update(string name)
{
  // Get current sim time 
  cur_time_ = ros::Time::now(); 
  
  // Update current joint states
  cur_caster_pos_   = caster_joint_->pos_;
  cur_caster_vel_   = caster_joint_->vel_;
  cur_wheel_pos_    = wheel_joint_->pos_;
  cur_wheel_vel_    = wheel_joint_->vel_;
   
  // Publish current joint states
  std_msgs::Int32 msg;
  
  WheelUnit wheel_unit;
  msg.data   = wheel_unit.toLowLevelSteer(cur_caster_pos_);
  pub_enc_caster_pos_.publish(msg);
  
  msg.data   = wheel_unit.toLowLevelSteer(cur_caster_vel_);
  pub_enc_caster_vel_.publish(msg);
  
  msg.data   = wheel_unit.toLowLevelDrive(cur_wheel_pos_);
  pub_enc_wheel_pos_.publish(msg);
  
  msg.data   = wheel_unit.toLowLevelDrive(cur_wheel_vel_ );
  pub_enc_wheel_vel_.publish(msg);

  // Calculate errors
  float caster_error  = req_caster_pos_ - cur_caster_pos_;
  float wheel_error   = req_wheel_vel_  - cur_wheel_vel_;
   
  // Calculate required PID effort
  float caster_effort = PID_caster_.update(caster_error);
  float wheel_effort  = PID_wheel_.update(wheel_error);
  
  // Save finish sim time
  prev_time_ = ros::Time::now();

  ROS_DEBUG_NAMED(ROS_NAME, "%s CasterPID [pos = %.3frad/s -> %.3frad/s, error = %.3f, effort = %.3f]", name.c_str(), cur_caster_pos_, req_caster_pos_, caster_error, caster_effort);
  ROS_DEBUG_NAMED(ROS_NAME, "%s WheelPID  [vel = %.3frad/s -> %.3frad/s, error = %.3f, effort = %.3f]", name.c_str(), cur_wheel_vel_, req_wheel_vel_, wheel_error, wheel_effort);
  
  // Apply control effort
  caster_joint_->update(caster_effort);
  wheel_joint_->update(wheel_effort);
}


void SimWheelUnitController::CB_SetRequestedCasterPos(const std_msgs::Int32::ConstPtr& msg)
{
  // TODO Limits 
  WheelUnit wheel_unit("Only For Functions", -1);
  req_caster_pos_  = wheel_unit.toAngleRad((float)msg->data);  
  ROS_DEBUG_NAMED(ROS_NAME, "Request caster pos: %.3frad, current: %.3frad", req_caster_pos_, cur_caster_pos_);
}
  
void SimWheelUnitController::CB_SetRequestedWheelVel(const std_msgs::Int32::ConstPtr& msg)
{
  // TODO Limits via param server
  WheelUnit wheel_unit("Only For Functions", -1);
  req_wheel_vel_ = wheel_unit.toVelocityRadPerSec((float)msg->data);  
  ROS_DEBUG_NAMED(ROS_NAME, "Request wheel vel: %.3frad/s %dpulses/s, current: %.3frad/s", req_wheel_vel_, wheel_unit.toLowLevelDrive(req_wheel_vel_), cur_wheel_vel_);
}
