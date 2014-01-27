/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*	Author: Okke Hendriks
*	Date  : 2014/01/24
* 		- File created.
*
* Description:
*	description
* 
***********************************************************************************/

#ifndef SIM_WHEEL_CONTROLLER_HPP
#define SIM_WHEEL_CONTROLLER_HPP

#include <iostream>
#include <stdio.h>

#include <map>

#include "rose20_common/common.hpp"
#include "roscomm/wheelunit_states.h"
#include <std_msgs/Float64.h>

#include "wheel_controller/wheel_unit.hpp"

class SimWheelController
{
  public:
  	SimWheelController(string name, ros::NodeHandle n);
  	~SimWheelController();

  	bool 	enable();
  	bool 	disable();
  	bool 	isEnabled();
  	map<string, WheelUnit>* getWheelUnits();
  	bool 	checkWatchdog();
  	bool 	writeWheelStates();
  	bool 	UpdateWheelUnitStates();
	bool 	PublishWheelUnitStates();	

	void 	CB_WheelUnitStatesRequest(const roscomm::wheelunit_states::ConstPtr& wheelunit_states);
	void 	CB_FR_enc_pos(const std_msgs::Float64::ConstPtr& msg);
	void 	CB_FR_enc_vel(const std_msgs::Float64::ConstPtr& msg);
	void 	CB_FL_enc_pos(const std_msgs::Float64::ConstPtr& msg);
	void 	CB_FL_enc_vel(const std_msgs::Float64::ConstPtr& msg);
	void 	CB_BR_enc_pos(const std_msgs::Float64::ConstPtr& msg);
	void 	CB_BR_enc_vel(const std_msgs::Float64::ConstPtr& msg);
	void 	CB_BL_enc_pos(const std_msgs::Float64::ConstPtr& msg);
	void 	CB_BL_enc_vel(const std_msgs::Float64::ConstPtr& msg);
	
  private:
  	string 	                name_;
    ros::NodeHandle         n_;
    
  	ros::Publisher			wheelunit_states_pub_;
  	ros::Publisher			FR_caster_pub_;
    ros::Publisher			FR_wheel_pub_;
    ros::Publisher			FL_caster_pub_;
    ros::Publisher			FL_wheel_pub_;
    ros::Publisher			BR_caster_pub_;
    ros::Publisher			BR_wheel_pub_;
    ros::Publisher			BL_caster_pub_;
    ros::Publisher			BL_wheel_pub_;
    
  	ros::Subscriber			wheelunit_states_sub_;
  	ros::Subscriber			FR_caster_sub_;
    ros::Subscriber			FR_wheel_sub_;
    ros::Subscriber			FL_caster_sub_;
    ros::Subscriber			FL_wheel_sub_;
    ros::Subscriber			BR_caster_sub_;
    ros::Subscriber			BR_wheel_sub_;
    ros::Subscriber			BL_caster_sub_;
    ros::Subscriber			BL_wheel_sub_;

  	int 					enabled_;
  	map<string, WheelUnit>	wheelunits_;
};

#endif // SIM_WHEEL_CONTROLLER_HPP
