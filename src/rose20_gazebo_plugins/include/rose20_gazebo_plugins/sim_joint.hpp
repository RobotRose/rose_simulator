

#ifndef SIM_JOINT_HPP
#define SIM_JOINT_HPP

#include <iostream>
#include <stdio.h>
#include <math.h>

#include <ros/ros.h>


class SimJoint
{
  public:
  	SimJoint(float mass, float damping, float friction);

  	void update(float force);

  	float pos_;
  	float vel_;

  private:
  	float mass_;
  	float damping_;
  	float friction_;
  	ros::Time prev_time_;

};


#endif // SIM_JOINT_HPP
