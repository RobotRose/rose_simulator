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
#include "rose20_platform/lift.h"
#include "rose20_platform/lift_control.h"

#define LIFT_PRISMATIC_SPEED  (M_PI/12.0)  // [rad/s]
#define LIFT_BOTTOM_MAX_ANGLE (M_PI/2.0 - M_PI/8.0) 
#define LIFT_BOTTOM_MIN_ANGLE (     0.0 + M_PI/8.0)  

using namespace rose20_common;

namespace gazebo
{   
class LiftController : public ModelPlugin
{
public: 
  void    Load(physics::ModelPtr parent, sdf::ElementPtr /*sdf*/);
  void    OnUpdate(const common::UpdateInfo & /*info*/);
  
private:     
  void    CB_SetLiftStatus(const rose20_platform::lift::ConstPtr& lift_message);
  double  calcAnimationTime();
  void    createAndAttachAnimation(double duration, bool repeat);
  void    setLiftLow();
  void    setLiftMid();
  void    setLiftHigh();
       
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
  std::map<std::string, common::NumericAnimationPtr> anim_;
};
}

#endif // LIFTCONTROLLER_H

