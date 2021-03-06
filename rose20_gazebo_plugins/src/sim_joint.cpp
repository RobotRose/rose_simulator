#include "rose20_gazebo_plugins/sim_joint.hpp"

SimJoint::SimJoint(float mass, float damping, float friction, float max_force)
	: mass_(mass)
	, damping_(damping)
	, friction_(friction)
	, pos_(0.0)
	, vel_(0.0)
	, prev_time_(ros::Time::now())
	, max_force_(max_force)
{}


void SimJoint::update(float force)
{
	ros::Time cur_time = ros::Time::now();

	float dt = cur_time.toSec() - prev_time_.toSec();

	if(dt <= 0.0)
	{
		prev_time_ = ros::Time::now();
		return;
	}


	float damping_force = -damping_*vel_;

	// Calculate friction
	float new_friction = 0.0;
	if(force > 0)
		new_friction = -fmin(fabs(force), friction_);
	else if(force < 0)
		new_friction = fmin(fabs(force), friction_);
	else
		new_friction = 0.0;

	float new_force  	= fmin(max_force_, fmax(-max_force_, force + damping_force + new_friction));

	float new_acc = new_force/mass_;
	float new_vel = vel_ + new_acc*dt;
	float new_pos = pos_ + new_vel*dt;

	vel_ = new_vel;
	pos_ = new_pos;

	ROS_DEBUG_NAMED(ROS_NAME, "Update joint dt=%.3f  [force  = %.3f, damping_force  = %.3f, friction  = %.3f, applied_force  = %.3f, acc  = %.3f, vel  = %.3f, pos  = %.3f]", dt, force, damping_force, new_friction, new_force, new_acc, vel_, pos_);
  


	prev_time_ = ros::Time::now();
}
