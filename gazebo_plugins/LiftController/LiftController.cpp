/** lift_controller.cpp
 *
 * @class LiftController
 *
 * Contains simulation controller components for the lift actuation system 
 *
 * \author Okke Hendriks
 * \date 10-10-2013
 * \version 1.0
 */

#include "LiftController.hpp"
  
using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(LiftController)

void LiftController::Load(physics::ModelPtr parent, sdf::ElementPtr /*sdf*/) 
{  
  // Store the pointer to the model
  model_ = parent;
  
  // Update event
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&LiftController::OnUpdate, this, _1));
  
  // Get the joint
  lift_prism_joint_ = model_->GetJoint("base_lift_prismatic");       

  // ROS connection
  int argc = 0;
  char** argv = NULL;
  ros::init(argc,argv,"LiftController_Gazebo_plugin");
  ros::NodeHandle n_;
  lift_pose_request_sub_  = n_.subscribe("lift_pose_request", 50, &LiftController::CB_SetLiftStatus, this);
  
  // Force to a start position
  lift_target_pose_ = 1;
  joint_target_pos_ = 0.00;
  
  ROS_INFO("Loaded LiftController");      
}

void LiftController::OnUpdate(const common::UpdateInfo & /*info*/)
{
  // Get current sim time and check if previous animation has finished
  curr_time_ = model_->GetWorld()->GetSimTime();     
  if(animation_finished_time_.Double() - curr_time_.Double() < 0.0)
  {
    // Create looping animation to keep lift at current position
    createAndAttachAnimation(10.0, true);
  }    
}
   
void LiftController::CB_SetLiftStatus(const rosbee_control_wireless::lift::ConstPtr& lift_message)
{
  // Store requested lift pose
  this->lift_target_pose_ = lift_message->pose;
  
  switch(lift_target_pose_)
  {
    case 0:
      joint_target_pos_ = 0.02;
      break;
    case 1:
      joint_target_pos_ = 0.00;
      break;
    case 2:
      joint_target_pos_ = -0.02;                 
      break;
    default:
      ROS_WARN("Invalid lift pose requested");
      break;           
  }

  // Create a new animation from the current position to the requested position
  // Calculate the duration of the animation
  cur_lift_position_  = lift_prism_joint_->GetAngle(0).Radian();
  double duration     = (fabs(cur_lift_position_ - joint_target_pos_))/LIFT_PRISMATIC_SPEED;
  createAndAttachAnimation(duration, false);
}

void LiftController::createAndAttachAnimation(double duration, bool repeat)
{
  curr_time_          = model_->GetWorld()->GetSimTime();
  cur_lift_position_  = lift_prism_joint_->GetAngle(0).Radian();
   
  anim_["base_lift_prismatic"].reset(new common::NumericAnimation("lift_animation", duration, repeat));

  // Create a starting keyframe
  common::NumericKeyFrame *key = anim_["base_lift_prismatic"]->CreateKeyFrame(0.0);
  key->SetValue(cur_lift_position_);

  // Create the end keyframe 
  key = anim_["base_lift_prismatic"]->CreateKeyFrame(duration);
  key->SetValue(joint_target_pos_);
 
  // Attach the animation to the model
  model_->SetJointAnimation(anim_);
  
  animation_finished_time_ = curr_time_ + common::Time(duration);
}


