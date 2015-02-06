/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*  Author: Okke Hendriks
*  Date  : 2013/12/02
*     - File created.
*
* Description:
*  Contains simulation controller components for the lift actuation system 
* 
***********************************************************************************/

#include "rose20_gazebo_plugins/LiftController.hpp"
  
using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(LiftController)

void LiftController::Load(physics::ModelPtr parent, sdf::ElementPtr sdf) 
{  
  // Store the pointer to the model
  model_ = parent;
  sdf_   = sdf;

  controller_name_ = "WheelUnitController";
  
  // Update event
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&LiftController::OnUpdate, this, _1));
  
   // Get the liftjoints  
  if (sdf_->HasElement("bottom_lift_joint"))
  {
    bottom_joint_string_  = sdf_->Get<std::string>("bottom_lift_joint");
    bottom_joint_         = model_->GetJoint(bottom_joint_string_);
    top_namespace_        = rose_conversions::replaceString(bottom_joint_string_, "::", "/"); 
  }
  if (sdf_->HasElement("top_lift_joint"))
  {
    top_joint_string_ = sdf_->Get<std::string>("top_lift_joint");
    top_joint_        = model_->GetJoint(top_joint_string_);
    top_namespace_    = rose_conversions::replaceString(top_joint_string_, "::", "/");  
  }

  // Check if we sucessfully got the joints from the model
  if(bottom_joint_)
    ROS_INFO("(%s): Bottom lift joint found: %s", controller_name_.c_str(), bottom_joint_string_.c_str());
  else
    ROS_ERROR("(%s): Bottom lift not found: %s, correctly set <bottom_lift_joint>insert_joint_name_here</bottom_lift_joint>", controller_name_.c_str() , bottom_joint_string_.c_str());   
        
  if(top_joint_)
    ROS_INFO("(%s): Top lift joint found: %s", controller_name_.c_str(), top_joint_string_.c_str());
  else
    ROS_ERROR("(%s): Top lift joint not found: %s, correctly set <top_lift_joint>insert_joint_name_here</top_lift_joint>", controller_name_.c_str(), top_joint_string_.c_str());   
        

  // ROS connection
  int argc = 0;
  char** argv = NULL;
  ros::init(argc,argv,"LiftController_Gazebo_plugin");
  ros::NodeHandle n_;
  lift_pose_request_sub_  = n_.subscribe("lift_pose_request", 50, &LiftController::CB_SetLiftStatus, this);
  
  // Force to a start position
  lift_target_pose_ = 1;
  setLiftMid();

  // Create a new animation from the current position to the requested position
  createAndAttachAnimation(calcAnimationTime(), false);
  
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
   
void LiftController::CB_SetLiftStatus(const rose20_platform::lift::ConstPtr& lift_message)
{
  // Store requested lift pose
  this->lift_target_pose_ = lift_message->pose;
  ROS_INFO("Lift pose requested: %d", this->lift_target_pose_);
  switch(lift_target_pose_)
  {
    case 0:
      setLiftLow();
      break;
    case 1:
      setLiftMid();
      break;
    case 2:
      setLiftHigh();
      break;
    default:
      ROS_WARN("Invalid lift pose requested");
      break;           
  }

}

double LiftController::calcAnimationTime()
{
  // Calculate the duration of the animation
  cur_lift_bottom_pos_  = bottom_joint_->GetAngle(0).Radian();
  cur_lift_top_pos_     = top_joint_->GetAngle(0).Radian();
  double duration       = (fabs(cur_lift_bottom_pos_ - bottom_joint_target_pos_))/LIFT_PRISMATIC_SPEED;
  return duration;
}

void LiftController::setLiftLow()
{
  bottom_joint_target_pos_  =  LIFT_BOTTOM_MIN_ANGLE; 
  top_joint_target_pos_     = -LIFT_BOTTOM_MIN_ANGLE; 
  // Create a new animation from the current position to the requested position
  createAndAttachAnimation(calcAnimationTime(), false);   
}

void LiftController::setLiftMid()
{
  bottom_joint_target_pos_  =  (LIFT_BOTTOM_MIN_ANGLE + LIFT_BOTTOM_MAX_ANGLE)/2; 
  top_joint_target_pos_     = -(LIFT_BOTTOM_MIN_ANGLE + LIFT_BOTTOM_MAX_ANGLE)/2;   
  // Create a new animation from the current position to the requested position
  createAndAttachAnimation(calcAnimationTime(), false);
}

void LiftController::setLiftHigh()
{    
  bottom_joint_target_pos_  =  LIFT_BOTTOM_MAX_ANGLE; 
  top_joint_target_pos_     = -LIFT_BOTTOM_MAX_ANGLE; 
  // Create a new animation from the current position to the requested position
  createAndAttachAnimation(calcAnimationTime(), false);         
}

void LiftController::createAndAttachAnimation(double duration, bool repeat)
{
  curr_time_            = model_->GetWorld()->GetSimTime();
  cur_lift_bottom_pos_  = bottom_joint_->GetAngle(0).Radian();
  cur_lift_top_pos_     = top_joint_->GetAngle(0).Radian();
  ROS_DEBUG("cur_lift_bottom_pos_: %.3f", cur_lift_bottom_pos_);
  ROS_DEBUG("cur_lift_top_pos_: %.3f", cur_lift_top_pos_);
  ROS_DEBUG("bottom_joint_target_pos_: %.3f", bottom_joint_target_pos_);
  ROS_DEBUG("top_joint_target_pos_: %.3f", top_joint_target_pos_);
  ROS_DEBUG("duration: %.3f", duration);

  // Create the annimations 
  anim_[bottom_joint_string_].reset(new common::NumericAnimation("lift_bottom_animation", duration, repeat));
  anim_[top_joint_string_].reset(new common::NumericAnimation("lift_top_animation", duration, repeat));

  // Create a starting keyframes
  common::NumericKeyFrame *key_bottom = anim_[bottom_joint_string_]->CreateKeyFrame(0.0);
  common::NumericKeyFrame *key_top    = anim_[top_joint_string_]->CreateKeyFrame(0.0);
  key_bottom->SetValue(cur_lift_bottom_pos_);
  key_top->SetValue(cur_lift_top_pos_);

  // Create the end keyframes 
  key_bottom  = anim_[bottom_joint_string_]->CreateKeyFrame(duration);
  key_top     = anim_[top_joint_string_]->CreateKeyFrame(duration);
  key_bottom->SetValue(bottom_joint_target_pos_);
  key_top->SetValue(top_joint_target_pos_);

  // Attach the animations to the model
  model_->SetJointAnimation(anim_);

  animation_finished_time_ = curr_time_ + common::Time(duration);
}


