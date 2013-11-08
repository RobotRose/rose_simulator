/** neck_controller.cpp
 *
 * @class NeckController
 *
 * Contains simulation controller components for the neck actuation system 
 *
 * \author Okke Hendriks
 * \date 10-10-2013
 * \version 1.0
 */

#include "NeckController.hpp"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(NeckController)

void NeckController::Load(physics::ModelPtr parent, sdf::ElementPtr /*sdf*/) 
{
  // Store the pointer to the model
  model_ = parent;
  
  // Get the yaw & pitch neck joints
  pan_joint_   = model_->GetJoint("neck_pan_joint"); 
  tilt_joint_  = model_->GetJoint("neck_tilt_joint");          

  // Update event
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&NeckController::OnUpdate, this, _1));
  
  // Initialize pan & tilt PID controllers
  PID_pan_.Init(50.0, 10.0, 0.0, 50.0, -50.0, 100.0, -100.0);
  PID_tilt_.Init(50.0, 10.0, 0.0, 50.0, -50.0, 100.0, -100.0);
  
  // ROS connection
  int argc = 0;
  char** argv = NULL;
  ros::init(argc,argv,"NeckController_Gazebo_plugin");
  ros::NodeHandle n_;
 
  // Publishers and subscribers
  pub_pan_      = n_.advertise<std_msgs::Float64>("head_pan_controller/state", 5);
  pub_tilt_     = n_.advertise<std_msgs::Float64>("head_tilt_controller/state", 5);
  sub_panCom_   = n_.subscribe("neck_Control/pan", 5, &NeckController::CB_panCommand, this);
  sub_tiltCom_  = n_.subscribe("neck_Control/tilt", 5, &NeckController::CB_tiltCommand, this);
  
  // Initialize looptime
  prev_time_    = model_->GetWorld()->GetSimTime();
  
  // Initialize variables
  req_pan_      = 0.0;
  req_tilt_     = 0.0;
  
  ROS_INFO("Loaded NeckController");
}

void NeckController::OnUpdate(const common::UpdateInfo & /*info*/)
{
  // Get current sim time 
  curr_time_ = model_->GetWorld()->GetSimTime(); 
  
  // Update current joint states
  cur_pan_  = pan_joint_->GetAngle(2).Radian();
  cur_tilt_ = tilt_joint_->GetAngle(1).Radian();  
   
  // Publish current joint states
  std_msgs::Float64 pan_msg, tilt_msg;
  pan_msg.data   = cur_pan_;
  tilt_msg.data  = cur_tilt_;
  pub_pan_.publish(pan_msg);
  pub_tilt_.publish(tilt_msg);
  
  // Calculate position error
  float pan_error   = cur_pan_  - req_pan_;
  float tilt_error  = cur_tilt_ - req_tilt_;
   
  // Calculate required PID effort
  float pan_effort  = PID_pan_.Update(pan_error, curr_time_- prev_time_);
  float tilt_effort = PID_tilt_.Update(tilt_error, curr_time_- prev_time_);
  
  // Apply control effort
  pan_joint_->SetForce(0, pan_effort);
  tilt_joint_->SetForce(0, tilt_effort);
  
  // Save finsih sim time
  prev_time_ = model_->GetWorld()->GetSimTime();          
}
  
void NeckController::CB_panCommand(const std_msgs::Float64::ConstPtr& msg)
{
  // TODO Limits via param server
  req_pan_  = msg->data;  
  ROS_DEBUG("Request neck state pan: %.2frad & tilt: %.2frad.", req_pan_, req_tilt_);
}
  
void NeckController::CB_tiltCommand(const std_msgs::Float64::ConstPtr& msg)
{
  // TODO Limits via param server
  req_tilt_ = msg->data;  
  ROS_DEBUG("Request neck state pan: %.2frad & tilt: %.2frad.", req_pan_, req_tilt_);
}



