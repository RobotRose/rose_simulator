/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
* Author: Okke Hendriks
* Date  : 2014/01/24
*     - File created.
*
* Description:
* description
* 
***********************************************************************************/

#ifndef SIM_WHEEL_CONTROLLER_HPP
#define SIM_WHEEL_CONTROLLER_HPP

#include <iostream>
#include <stdio.h>

#include <map>
#include <boost/foreach.hpp>

#include "rose20_common/common.hpp"
#include "rose20_common/ros_name.hpp"
#include "rose20_common/TF_helper.hpp"
#include "rose20_common/wheel_unit.hpp"
#include "action_result_message.hpp"
#include "rose20_platform/wheelunit_states.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

#include "rose20_common/server_multiple_client/server_multiple_client.hpp"

#include "rose20_platform/wheelunit_states.h"
#include "rose20_platform/wheelunit_statesAction.h"
#include "rose20_platform/wheelunit_statesActionGoal.h"
#include "rose20_platform/wheelunit_statesActionResult.h"
#include "rose20_platform/wheelunit_statesActionFeedback.h"

#include "rose20_gazebo_plugins/sim_wheel_unit_controller.hpp"

class SimWheelController
{
  protected:
    typedef ServerMultipleClient<rose20_platform::wheelunit_statesAction> SMC;

  public:
    SimWheelController();
    SimWheelController(string name, ros::NodeHandle n);
    ~SimWheelController();

    bool  enable();
    bool  disable();
    bool  isEnabled();
    map<string, WheelUnit>* getWheelUnits();
    map<string, SimWheelUnitController>* getWheelUnitControllers();
    bool  checkWatchdog();
    bool  writeWheelStates();
    bool  UpdateWheelUnitControllers();
    bool  PublishWheelUnitStates(); 

    //void  CB_WheelUnitStatesRequest(const rose20_platform::wheelunit_states::ConstPtr& wheelunit_states);
    void  CB_FR_enc_pos(const std_msgs::Int32::ConstPtr& msg);
    void  CB_FR_enc_vel(const std_msgs::Int32::ConstPtr& msg);
    void  CB_FL_enc_pos(const std_msgs::Int32::ConstPtr& msg);
    void  CB_FL_enc_vel(const std_msgs::Int32::ConstPtr& msg);
    void  CB_BR_enc_pos(const std_msgs::Int32::ConstPtr& msg);
    void  CB_BR_enc_vel(const std_msgs::Int32::ConstPtr& msg);
    void  CB_BL_enc_pos(const std_msgs::Int32::ConstPtr& msg);
    void  CB_BL_enc_vel(const std_msgs::Int32::ConstPtr& msg);

    void  CB_WheelUnitStatesRequest(const rose20_platform::wheelunit_statesGoalConstPtr& goal, SMC* smc);
    
  private:
    string                  name_;
    ros::NodeHandle         n_;
    int                     enabled_;
    SMC*                    smc_;
    
    ros::Publisher      wheelunit_states_pub_;
    ros::Publisher      FR_caster_pub_;
    ros::Publisher      FR_wheel_pub_;
    ros::Publisher      FL_caster_pub_;
    ros::Publisher      FL_wheel_pub_;
    ros::Publisher      BR_caster_pub_;
    ros::Publisher      BR_wheel_pub_;
    ros::Publisher      BL_caster_pub_;
    ros::Publisher      BL_wheel_pub_;
    
    //ros::Subscriber     wheelunit_states_sub_;
    ros::Subscriber     FR_caster_sub_;
    ros::Subscriber     FR_wheel_sub_;
    ros::Subscriber     FL_caster_sub_;
    ros::Subscriber     FL_wheel_sub_;
    ros::Subscriber     BR_caster_sub_;
    ros::Subscriber     BR_wheel_sub_;
    ros::Subscriber     BL_caster_sub_;
    ros::Subscriber     BL_wheel_sub_;

    TFHelper*           FR_transform_;
    TFHelper*           FL_transform_;
    TFHelper*           BR_transform_;
    TFHelper*           BL_transform_;

    ros::Time           FR_prev_T_;
    ros::Time           FL_prev_T_;
    ros::Time           BR_prev_T_;
    ros::Time           BL_prev_T_;

    float               stopstart_angle_treshold_;
    int                 stopstart_speed_treshold_;

    typedef std::map<string, WheelUnit>     wheelunit_map;
    wheelunit_map                           wheelunits_;
    
    typedef std::map<string, SimWheelUnitController*>   wheelunit_controller_map;
    wheelunit_controller_map                            wheelunit_controllers_;
};

#endif // SIM_WHEEL_CONTROLLER_HPP
