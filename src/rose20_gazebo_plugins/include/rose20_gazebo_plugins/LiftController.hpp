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

#ifndef LIFTCONTROLLER_H
#define LIFTCONTROLLER_H

#include <stdio.h>
#include <math.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include "rose20_common/common.hpp"
#include "roscomm/lift.h"
#include "roscomm/lift_control.h"

#define LIFT_PRISMATIC_SPEED  0.02  // [m/s]

namespace gazebo
{   
class LiftController : public ModelPlugin
{
public: 
  void Load(physics::ModelPtr parent, sdf::ElementPtr /*sdf*/);
  void OnUpdate(const common::UpdateInfo & /*info*/);
  
private:     
  void CB_SetLiftStatus(const roscomm::lift::ConstPtr& lift_message);
  void createAndAttachAnimation(double duration, bool repeat);
       
  double                    cur_lift_bottom_pos_;
  double                    cur_lift_top_pos_;
  double                    bottom_joint_target_pos_;
  double                    top_joint_target_pos_;
  int                       lift_target_pose_;
  std::string               controller_name_;
  sdf::ElementPtr           sdf_;
  
  ros::Subscriber           lift_pose_request_sub_;
      
  physics::ModelPtr         model_;
  std::string               bottom_joint_string_;
  std::string               top_joint_string_;
  std::string               bottom_namespace_;
  std::string               top_namespace_;
  gazebo::physics::JointPtr bottom_joint_;  
  gazebo::physics::JointPtr top_joint_;           
  event::ConnectionPtr      updateConnection_;
  
  common::Time              curr_time_;
  common::Time              animation_finished_time_;
  std::map<std::string, common::NumericAnimationPtr> bottom_anim_;
  std::map<std::string, common::NumericAnimationPtr> top_anim_;
};
}

#endif // LIFTCONTROLLER_H

