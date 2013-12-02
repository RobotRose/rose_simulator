/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*  Author: Okke Hendriks
*  Date  : 2013/12/02
*     - File created.
*
* Description:
*  Contains simulation controller components for the neck actuation system 
* 
***********************************************************************************/
#ifndef NECKCONTROLLER_H
#define NECKCONTROLLER_H

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <std_msgs/Float64.h>

#include <boost/bind.hpp>
#include <stdio.h>
#include <math.h>

namespace gazebo
{   
class NeckController : public ModelPlugin
{
public: 
  void Load(physics::ModelPtr parent, sdf::ElementPtr /*sdf*/);
  void OnUpdate(const common::UpdateInfo & /*info*/);
  
private:
  void CB_panCommand(const std_msgs::Float64::ConstPtr& msg);     
  void CB_tiltCommand(const std_msgs::Float64::ConstPtr& msg);
   
  // Publishers
  ros::Publisher pub_pan_;
  ros::Publisher pub_tilt_;
  
  // Subscribers
  ros::Subscriber sub_panCom_;
  ros::Subscriber sub_tiltCom_;
     
  // Gazebo    
  physics::ModelPtr         model_;
  gazebo::physics::JointPtr pan_joint_;    
  gazebo::physics::JointPtr tilt_joint_;          
  event::ConnectionPtr      updateConnection_;
  
  // Control
  common::Time              curr_time_;
  common::Time              prev_time_;    
  common::PID               PID_pan_;
  common::PID               PID_tilt_;
  
  float                     req_pan_;
  float                     req_tilt_;
  float                     cur_pan_;
  float                     cur_tilt_;

};
}

#endif // NECKCONTROLLER_H
  
