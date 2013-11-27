/** lift_controller.hpp
 *
 * @class LiftController
 *
 * Contains simulation controller components for the lift actuation system 
 *
 * \author Okke Hendriks
 * \date 10-10-2013
 * \version 1.0
 */
#ifndef LIFTCONTROLLER_H
#define LIFTCONTROLLER_H

#include <ros/ros.h>
#include <ros/console.h>

#include "roscomm/lift.h"
#include "roscomm/lift_control.h"

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <math.h>

#define LIFT_PRISMATIC_SPEED  0.02  // [m/s]

namespace gazebo
{   
class LiftController : public ModelPlugin
{
public: 
  void Load(physics::ModelPtr parent, sdf::ElementPtr /*sdf*/);
  void OnUpdate(const common::UpdateInfo & /*info*/);
  
private:     
  void CB_SetLiftStatus(const rosbee_control_wireless::lift::ConstPtr& lift_message);
  void createAndAttachAnimation(double duration, bool repeat);
       
  double                    cur_lift_position_;
  double                    joint_target_pos_;
  int                       lift_target_pose_;
  
  ros::Subscriber           lift_pose_request_sub_;
      
  physics::ModelPtr         model_;
  gazebo::physics::JointPtr lift_prism_joint_;            
  event::ConnectionPtr      updateConnection_;
  
  common::Time              curr_time_;
  common::Time              animation_finished_time_;
  std::map<std::string, common::NumericAnimationPtr> anim_;
};
}

#endif // LIFTCONTROLLER_H

