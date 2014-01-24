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

#include "rose20_gazebo_plugins/WheelUnitController.hpp"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(WheelUnitController)

void WheelUnitController::Load(physics::ModelPtr model, sdf::ElementPtr sdf) 
{
  sdf_    = sdf;
  model_  = model;
  
  // Update event
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&WheelUnitController::OnUpdate, this, _1));
  
  controller_name_      = "WheelUnitController";    
  // Get caster and wheel joints  
  if (sdf_->HasElement("caster"))
  {
    caster_joint_string_  = sdf_->Get<std::string>("caster");
    caster_joint_         = model_->GetJoint(caster_joint_string_);
    caster_namespace_     = ReplaceString(caster_joint_string_, "::", "/"); 

    if(caster_joint_->GetChild()->GetWorldPose().rot.z >= 0)
      wheel_direction_ = 1.0;
    else
      wheel_direction_ = -1.0;
  }
  if (sdf_->HasElement("wheel"))
  {
    wheel_joint_string_ = sdf_->Get<std::string>("wheel");
    wheel_joint_        = model_->GetJoint(wheel_joint_string_);
    wheel_namespace_    = ReplaceString(wheel_joint_string_, "::", "/");  
  }
    
  if(caster_joint_)
    ROS_INFO("(%s): Caster joint found: %s", controller_name_.c_str(), caster_joint_string_.c_str());
  else
    ROS_ERROR("(%s): Caster joint not found: %s, correctly set <caster>insert_caster_joint_name_here</caster>", controller_name_.c_str() , caster_joint_string_.c_str());   
        
  if(wheel_joint_)
    ROS_INFO("(%s): Wheel joint found: %s", controller_name_.c_str(), wheel_joint_string_.c_str());
  else
    ROS_ERROR("(%s): Wheel joint not found: %s, correctly set <wheel>insert_wheel_joint_name_here</wheel>", controller_name_.c_str(), wheel_joint_string_.c_str());   
    
  // Initialize caster & wheel PID controllers
  PID_caster_.Init(200.0, 1.0, 0.0, 50.0, -50.0, 1500.0, -1500.0);
  PID_wheel_.Init(200.0, 1.0, 0.0, 50.0, -50.0, 1500.0, -1500.0);
  
  // ROS connection
  int argc = 0;
  char** argv = NULL;
  ros::init(argc,argv,"Wheelunit_Controller_Gazebo_plugin");
  ros::NodeHandle n_;
  
  // Publishers
  pub_enc_caster_pos_    = n_.advertise<std_msgs::Float64>("wheel_unit/" + caster_namespace_ + "/enc_pos", 5);
  pub_enc_caster_vel_    = n_.advertise<std_msgs::Float64>("wheel_unit/" + caster_namespace_ + "/enc_vel", 5);
  pub_enc_wheel_pos_     = n_.advertise<std_msgs::Float64>("wheel_unit/" + wheel_namespace_ + "/enc_pos", 5);
  pub_enc_wheel_vel_     = n_.advertise<std_msgs::Float64>("wheel_unit/" + wheel_namespace_ + "/enc_vel", 5);
  
  // Subscribers
  sub_caster_pos_   = n_.subscribe("sim_wheel_controller/" + caster_namespace_ + "/req_pos", 1, &WheelUnitController::CB_SetRequestedCasterPos, this);
  sub_wheel_vel_    = n_.subscribe("sim_wheel_controller/" + wheel_namespace_  + "/req_vel", 1, &WheelUnitController::CB_SetRequestedWheelVel, this);
      
  // Initialize variables
  cur_caster_pos_ = caster_joint_->GetAngle(1).Radian();
  cur_caster_vel_ = caster_joint_->GetVelocity(1);
  cur_wheel_pos_  = wheel_joint_->GetAngle(1).Radian();
  cur_wheel_vel_  = wheel_joint_->GetVelocity(1);
  req_caster_pos_ = 0.0;
  req_wheel_vel_  = 0.0 * wheel_direction_;
}

void WheelUnitController::OnUpdate(const common::UpdateInfo & /*info*/)
{
  // Get current sim time 
  curr_time_ = model_->GetWorld()->GetSimTime(); 
  
  // Update current joint states
  cur_caster_pos_   = caster_joint_->GetAngle(1).Radian();
  cur_caster_vel_ = caster_joint_->GetVelocity(1);
  cur_wheel_pos_    = wheel_joint_->GetAngle(1).Radian();
  cur_wheel_vel_  = wheel_joint_->GetVelocity(1);
   
  // Publish current joint states
  std_msgs::Float64 msg;
  
  WheelUnit wheel_unit;
  msg.data   = wheel_unit.toLowLevelSteer(cur_caster_pos_);
  pub_enc_caster_pos_.publish(msg);
  
  msg.data   = wheel_unit.toLowLevelSteer(cur_caster_vel_);
  pub_enc_caster_vel_.publish(msg);
  
  msg.data   = wheel_unit.toLowLevelDrive(cur_wheel_pos_);
  pub_enc_wheel_pos_.publish(msg);
  
  msg.data   = wheel_unit.toLowLevelDrive(cur_wheel_vel_);
  pub_enc_wheel_vel_.publish(msg);

  // Calculate position error
  float caster_error  = cur_caster_pos_ - req_caster_pos_;
  float wheel_error   = cur_wheel_vel_  - req_wheel_vel_;
   
  // Calculate required PID effort
  float caster_effort = PID_caster_.Update(caster_error, curr_time_- prev_time_);
  float wheel_effort  = PID_wheel_.Update(wheel_error, curr_time_- prev_time_);
  
  // Apply control effort
  caster_joint_->SetForce(0, caster_effort);
  wheel_joint_->SetForce(0, wheel_effort);
  
  // Save finsih sim time
  prev_time_ = model_->GetWorld()->GetSimTime();   
}


void WheelUnitController::CB_SetRequestedCasterPos(const std_msgs::Float64::ConstPtr& msg)
{
  // TODO Limits 
  WheelUnit wheel_unit;
  req_caster_pos_  = -wheel_unit.toAngleRad((float)msg->data);  
  ROS_INFO("Request caster pos: %.3frad", req_caster_pos_);
}
  
void WheelUnitController::CB_SetRequestedWheelVel(const std_msgs::Float64::ConstPtr& msg)
{
  // TODO Limits via param server
  WheelUnit wheel_unit("Only For Functions", -1);
  req_wheel_vel_ = wheel_unit.toVelocityRadPerSec((float)msg->data)*wheel_direction_;  
  ROS_INFO("Request wheel vel: %.3frad/s", req_wheel_vel_);
}
