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

  caster_joint_         = new SimJoint(1.0, 50.0, 0.0, 20.0);
  wheel_joint_          = new SimJoint(1.0, 100.0, 0.0, 5.0);

  // Initialize caster & wheel PID controllers
  PID_caster_.initialize(15.0, 0.1, 0.0, -0.5, 0.5, -7.5, 7.5);
  PID_wheel_.initialize(5.0, 0.1, 0.0, -0.5, 0.5, -5.0, 5.0);

  // Publishers
  pub_enc_caster_pos_    = n_.advertise<std_msgs::Float64>("/wheel_unit/" + caster_namespace_ + "/enc_pos", 5);
  pub_enc_caster_vel_    = n_.advertise<std_msgs::Float64>("/wheel_unit/" + caster_namespace_ + "/enc_vel", 5);
  pub_enc_wheel_pos_     = n_.advertise<std_msgs::Float64>("/wheel_unit/" + wheel_namespace_ + "/enc_pos", 5);
  pub_enc_wheel_vel_     = n_.advertise<std_msgs::Float64>("/wheel_unit/" + wheel_namespace_ + "/enc_vel", 5);
  
  // Subscribers
  sub_caster_pos_   = n_.subscribe("sim_wheel_controller/" + caster_namespace_ + "/req_pos", 1, &SimWheelUnitController::CB_SetRequestedCasterPos, this);
  sub_wheel_vel_    = n_.subscribe("sim_wheel_controller/" + wheel_namespace_  + "/req_vel", 1, &SimWheelUnitController::CB_SetRequestedWheelVel, this);
      
  // Initialize variables
  cur_caster_pos_ = 0.0;
  cur_caster_vel_ = 0.0;
  cur_wheel_pos_  = 0.0;
  cur_wheel_vel_  = 0.0;
  req_caster_pos_ = 0.0;
  req_wheel_vel_  = 0.0 * wheel_direction_;

  stopstart_error_ = WHEELUNIT_STOP_MOVE_ANGLE_ERR_VAL;
}

SimWheelUnitController::~SimWheelUnitController()
{
  
}

void SimWheelUnitController::update()
{
  // Get current sim time 
  cur_time_ = ros::Time::now(); 
  
  // Update current joint states
  cur_caster_pos_   = caster_joint_->pos_;
  cur_caster_vel_   = caster_joint_->vel_;
  cur_wheel_pos_    = wheel_joint_->pos_;
  cur_wheel_vel_    = wheel_joint_->vel_;
   
  // Publish current joint states
  std_msgs::Float64 msg;
  
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

  // Set speed to zero if a wheelunit has to rotate more thatn a certain amount (with hysteresis)
  if (fabs(caster_error) > stopstart_error_)
  {
      req_wheel_vel_ = 0.0;
      wheel_error = req_wheel_vel_ - cur_wheel_vel_;
      // Wait for totally rotated
      stopstart_error_ = WHEELUNIT_START_MOVE_ANGLE_ERR_VAL;
      ROS_WARN_NAMED("SimWheelUnitController", "Setting drive speed to zero to turn wheels, caster_error: %.4f", caster_error);

      // Keep current orientation of the wheels if the speed error is above a certain threshold
      float stopstart_speed_error_ = 0.1; // [m/s]
      if (fabs(wheel_error) > stopstart_speed_error_)
      {
          req_caster_pos_ = cur_caster_pos_;
          caster_error  = req_caster_pos_ - cur_caster_pos_;
          // Wait for speed error small enough
      //    stopstart_speed_error_ = WHEELUNIT_START_MOVE_ANGLE_ERR_VAL;
          ROS_WARN_NAMED("SimWheelUnitController", "Keeping orientation to slow down or speedup, wheel_error: %.4f", wheel_error);
      }
      //else
      //    stopstart_speed_error_ = WHEELUNIT_STOP_MOVE_ANGLE_ERR_VAL;
      
  }
  else
      stopstart_error_ = WHEELUNIT_STOP_MOVE_ANGLE_ERR_VAL;


  
   
  // Calculate required PID effort
  float dt = cur_time_.toSec() - prev_time_.toSec();
  float caster_effort = PID_caster_.update(caster_error, dt);
  float wheel_effort  = PID_wheel_.update(wheel_error, dt);

  ROS_INFO_NAMED("SimWheelUnitController", "Caster PID [pos = %.3f/%.3f, error = %.3f, effort = %.3f]", cur_caster_pos_, req_caster_pos_, caster_error, caster_effort);
  ROS_INFO_NAMED("SimWheelUnitController", "Wheel_PID  [vel = %.3f/%.3f, error = %.3f, effort = %.3f]", cur_wheel_vel_, req_wheel_vel_, wheel_error, wheel_effort);
  
  // Apply control effort
  caster_joint_->update(caster_effort);
  wheel_joint_->update(wheel_effort);
  
  // Save finish sim time
  prev_time_ = ros::Time::now();
}


void SimWheelUnitController::CB_SetRequestedCasterPos(const std_msgs::Float64::ConstPtr& msg)
{
  // TODO Limits 
  WheelUnit wheel_unit("Only For Functions", -1);
  req_caster_pos_  = -wheel_unit.toAngleRad((float)msg->data);  
  ROS_INFO_NAMED("SimWheelUnitController", "Request caster pos: %.3frad, current: %.3frad", req_caster_pos_, cur_caster_pos_);
}
  
void SimWheelUnitController::CB_SetRequestedWheelVel(const std_msgs::Float64::ConstPtr& msg)
{
  // TODO Limits via param server
  WheelUnit wheel_unit("Only For Functions", -1);
  req_wheel_vel_ = wheel_unit.toVelocityRadPerSec((float)msg->data)*wheel_direction_;  
  ROS_INFO_NAMED("SimWheelUnitController", "Request wheel vel: %.3frad/s, current: %.3frad/s", req_wheel_vel_, cur_wheel_vel_);
}
