/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*    Author: Okke Hendriks
*    Date  : 2014/01/24
*         - File created.
*
* Description:
*    Controller of the platform in the imsulator, listens to wheelunit_states_request
*    and publishes to wheelunit_states.
* 
***********************************************************************************/
#include "rose20_gazebo_plugins/sim_wheel_controller.hpp"

SimWheelController::SimWheelController(string name, ros::NodeHandle n)
    : sh_platform_controller_alarm_(SharedVariable<bool>("platform_controller_alarm"))
    , sh_platform_controller_reset_(SharedVariable<bool>("platform_controller_reset"))
{
    n_           	= n;
    name_       	= name;
    enabled_ 	 	= false;
    stopstart_angle_treshold_ = WHEELUNIT_STOP_MOVE_ANGLE_ERR_VAL;
    stopstart_speed_treshold_ = 15;                                         //! @todo: OH Make viam define and settable in lowlevel

    // Host read-only, published alarm state, max publish rate is 10 hz
    sh_platform_controller_alarm_.host(false, true, ros::Rate(10.0));
    sh_platform_controller_alarm_ = false;

    // Host mutable, non-published reset boolean
    sh_platform_controller_reset_.host(false, false);
    sh_platform_controller_reset_ = false;

    FR_prev_T_      = ros::Time::now(); 
    FL_prev_T_      = ros::Time::now(); 
    BR_prev_T_      = ros::Time::now(); 
    BL_prev_T_      = ros::Time::now(); 

    smc_ = new SMC(n_, "platform_controller", boost::bind(&SimWheelController::CB_WheelUnitStatesRequest, this, _1, _2));
    smc_->startServer();

    // Publishers
    wheelunit_states_pub_   = n.advertise<rose_base_msgs::wheelunit_states>("/platform_controller/wheelunit_states", 1);
    FR_caster_pub_          = n.advertise<std_msgs::Int32>("/sim_platform_controller/FR_caster/req_pos", 1);
    FR_wheel_pub_           = n.advertise<std_msgs::Int32>("/sim_platform_controller/FR_wheel/req_vel", 1);
    FL_caster_pub_          = n.advertise<std_msgs::Int32>("/sim_platform_controller/FL_caster/req_pos", 1);
    FL_wheel_pub_           = n.advertise<std_msgs::Int32>("/sim_platform_controller/FL_wheel/req_vel", 1);
    BR_caster_pub_          = n.advertise<std_msgs::Int32>("/sim_platform_controller/BR_caster/req_pos", 1);
    BR_wheel_pub_           = n.advertise<std_msgs::Int32>("/sim_platform_controller/BR_wheel/req_vel", 1);
    BL_caster_pub_          = n.advertise<std_msgs::Int32>("/sim_platform_controller/BL_caster/req_pos", 1);
    BL_wheel_pub_           = n.advertise<std_msgs::Int32>("/sim_platform_controller/BL_wheel/req_vel", 1);

    // Subscribers
    // wheelunit_states_sub_   = n_.subscribe("/drive_controller/wheelunit_states_request", 1, &SimWheelController::CB_WheelUnitStatesRequest, this);
    FR_caster_sub_          = n_.subscribe("/wheel_unit/FR_caster/enc_pos", 1, &SimWheelController::CB_FR_enc_pos, this);
    FR_wheel_sub_           = n_.subscribe("/wheel_unit/FR_wheel/enc_vel", 1 , &SimWheelController::CB_FR_enc_vel, this);
    FL_caster_sub_          = n_.subscribe("/wheel_unit/FL_caster/enc_pos", 1, &SimWheelController::CB_FL_enc_pos, this);
    FL_wheel_sub_           = n_.subscribe("/wheel_unit/FL_wheel/enc_vel", 1 , &SimWheelController::CB_FL_enc_vel, this);
    BR_caster_sub_          = n_.subscribe("/wheel_unit/BR_caster/enc_pos", 1, &SimWheelController::CB_BR_enc_pos, this);
    BR_wheel_sub_           = n_.subscribe("/wheel_unit/BR_wheel/enc_vel", 1 , &SimWheelController::CB_BR_enc_vel, this);
    BL_caster_sub_          = n_.subscribe("/wheel_unit/BL_caster/enc_pos", 1, &SimWheelController::CB_BL_enc_pos, this);
    BL_wheel_sub_           = n_.subscribe("/wheel_unit/BL_wheel/enc_vel", 1 , &SimWheelController::CB_BL_enc_vel, this);

    // Transforms
    FR_transform_           = new TFHelper("FR", n_, "/base_link", "FR");
    FL_transform_           = new TFHelper("FL", n_, "/base_link", "FL");
    BR_transform_           = new TFHelper("BR", n_, "/base_link", "BR");
    BL_transform_           = new TFHelper("BL", n_, "/base_link", "BL");

    // Add Wheelunits to the map
    WheelUnit* wheelunit;
    wheelunit = new WheelUnit("FR", 0);
    wheelunits_.insert( std::pair<string, WheelUnit>(wheelunit->name_, *wheelunit));
    
    wheelunit = new WheelUnit("FL", 2);
    wheelunits_.insert( std::pair<string, WheelUnit>(wheelunit->name_, *wheelunit));
    
    wheelunit = new WheelUnit("BR", 4);
    wheelunits_.insert( std::pair<string, WheelUnit>(wheelunit->name_, *wheelunit));
    
    wheelunit = new WheelUnit("BL", 6);
    wheelunits_.insert( std::pair<string, WheelUnit>(wheelunit->name_, *wheelunit));
    delete wheelunit;

    // Add wheelunit controllers to the map
    wheelunit_controllers_.insert( std::pair<string, SimWheelUnitController*>("FR", new SimWheelUnitController(n_, "FR", 1)));
    wheelunit_controllers_.insert( std::pair<string, SimWheelUnitController*>("FL", new SimWheelUnitController(n_, "FL", -1)));
    wheelunit_controllers_.insert( std::pair<string, SimWheelUnitController*>("BR", new SimWheelUnitController(n_, "BR", 1)));
    wheelunit_controllers_.insert( std::pair<string, SimWheelUnitController*>("BL", new SimWheelUnitController(n_, "BL", -1)));

    ROS_INFO_NAMED(ROS_NAME, "Started %s", name_.c_str()); 

    enable();
}

SimWheelController::~SimWheelController()
{
    disable();
    ROS_INFO_NAMED(ROS_NAME, "Stopped %s", name_.c_str());
}


bool SimWheelController::enable()
{
    enabled_ = true;
}

bool SimWheelController::disable()
{
    enabled_ = false;
}

bool SimWheelController::isEnabled()
{
    return enabled_;
}

map<string, WheelUnit>* SimWheelController::getWheelUnits()
{
    return &wheelunits_;
}

bool SimWheelController::checkWatchdog()
{
    return true;
}

bool SimWheelController::writeWheelStates()
{
    std_msgs::Int32 FR_msg_rot, FL_msg_rot, BR_msg_rot, BL_msg_rot;
    std_msgs::Int32 FR_msg_vel, FL_msg_vel, BR_msg_vel, BL_msg_vel; 

    FR_msg_rot.data = wheelunits_.at("FR").getSetAngleLowLevel();
    FR_msg_vel.data = wheelunits_.at("FR").getSetVelocityLowLevel();
    FL_msg_rot.data = wheelunits_.at("FL").getSetAngleLowLevel();
    FL_msg_vel.data = wheelunits_.at("FL").getSetVelocityLowLevel();
    BR_msg_rot.data = wheelunits_.at("BR").getSetAngleLowLevel();
    BR_msg_vel.data = wheelunits_.at("BR").getSetVelocityLowLevel();
    BL_msg_rot.data = wheelunits_.at("BL").getSetAngleLowLevel();
    BL_msg_vel.data = wheelunits_.at("BL").getSetVelocityLowLevel();  

    // Check error values
    bool angle_error_too_large  = false;
    bool speed_too_large        = false;
    BOOST_FOREACH(const wheelunit_map::value_type& wheelunit, wheelunits_)
    {
        if(fabs(wheelunit.second.getSetAngleRad() - wheelunit.second.getMeasuredAngleRad()) > stopstart_angle_treshold_)
        {
            angle_error_too_large = true;
            ROS_DEBUG_NAMED(ROS_NAME, "Angle error %s too large (Set: %.2f, Measured: %.2f).", wheelunit.first.c_str(), wheelunit.second.getSetAngleRad(), wheelunit.second.getMeasuredAngleRad());
        } 

        if(wheelunit.second.getMeasuredVelocityLowLevel() > stopstart_speed_treshold_*1000)
        {
            speed_too_large = true;
            ROS_DEBUG_NAMED(ROS_NAME, "Speed %s too large (%d).", wheelunit.first.c_str(), wheelunit.second.getMeasuredVelocityLowLevel());
        }
    }

    // Set speed to zero if a wheelunit has to rotate more thatn a certain amount (with hysteresis)
    if(angle_error_too_large)
    {
        // Wait for totally rotated
        stopstart_angle_treshold_ = WHEELUNIT_START_MOVE_ANGLE_ERR_VAL;
        ROS_DEBUG_NAMED(ROS_NAME, "Setting drive speed to zero to turn wheels, caster_error.");

        FR_msg_vel.data = 0;
        FL_msg_vel.data = 0;
        BR_msg_vel.data = 0;
        BL_msg_vel.data = 0;

        // Keep current orientation of the wheels if the speed error is above a certain threshold
        if(speed_too_large)
        {
            // Wait for speed error small enough
            FR_msg_rot.data = wheelunits_.at("FR").getMeasuredAngleLowLevel();
            FL_msg_rot.data = wheelunits_.at("FL").getMeasuredAngleLowLevel();
            BR_msg_rot.data = wheelunits_.at("BR").getMeasuredAngleLowLevel();
            BL_msg_rot.data = wheelunits_.at("BL").getMeasuredAngleLowLevel();

            ROS_DEBUG_NAMED(ROS_NAME, "Keeping orientation to slow down or speedup, wheel_error.");
        }
    }
    else 
        stopstart_angle_treshold_ = WHEELUNIT_STOP_MOVE_ANGLE_ERR_VAL;


    // Publish setpoints    
    FR_caster_pub_.publish(FR_msg_rot);
    FR_wheel_pub_.publish(FR_msg_vel); 

    FL_caster_pub_.publish(FL_msg_rot);
    FL_wheel_pub_.publish(FL_msg_vel);

    BR_caster_pub_.publish(BR_msg_rot);
    BR_wheel_pub_.publish(BR_msg_vel);

    BL_caster_pub_.publish(BL_msg_rot);
    BL_wheel_pub_.publish(BL_msg_vel);  

    return true;
}

bool SimWheelController::UpdateWheelUnitControllers()
{
    if(!writeWheelStates())
        return false;

    pair<string, SimWheelUnitController*> a_pair;
    BOOST_FOREACH(a_pair, wheelunit_controllers_) 
        a_pair.second->update(a_pair.first);

    return true;
}

bool SimWheelController::PublishWheelUnitStates()
{
    ROS_DEBUG_NAMED(ROS_NAME, "Sim PublishWheelUnitStates");

    // Fill message with lowlevel values
    rose_base_msgs::wheelunit_states wheelunit_states; 
    wheelunit_states.angle_FR       = (float)wheelunits_.at("FR").measured_rotation_;   
    wheelunit_states.angle_FL       = (float)wheelunits_.at("FL").measured_rotation_;
    wheelunit_states.angle_BR       = (float)wheelunits_.at("BR").measured_rotation_;
    wheelunit_states.angle_BL       = (float)wheelunits_.at("BL").measured_rotation_;

    wheelunit_states.velocity_FR    = wheelunits_.at("FR").measured_velocity_;
    wheelunit_states.velocity_FL    = wheelunits_.at("FL").measured_velocity_;
    wheelunit_states.velocity_BR    = wheelunits_.at("BR").measured_velocity_;
    wheelunit_states.velocity_BL    = wheelunits_.at("BL").measured_velocity_;

    wheelunit_states.diff_FR        = wheelunits_.at("FR").measured_drive_encoder_diff_;
    wheelunit_states.diff_FL        = wheelunits_.at("FL").measured_drive_encoder_diff_;
    wheelunit_states.diff_BR        = wheelunits_.at("BR").measured_drive_encoder_diff_;
    wheelunit_states.diff_BL        = wheelunits_.at("BL").measured_drive_encoder_diff_;

    wheelunit_states.dT_FR          = wheelunits_.at("FR").dT_;
    wheelunit_states.dT_FL          = wheelunits_.at("FL").dT_;
    wheelunit_states.dT_BR          = wheelunits_.at("BR").dT_;
    wheelunit_states.dT_BL          = wheelunits_.at("BL").dT_;

    wheelunit_states_pub_.publish(wheelunit_states);

    // Publish wheel transforms    
    FR_transform_->setTransform(0.0, 0.0, -(wheelunits_.at("FR").getMeasuredAngleRad()), 0.58/2.0, -0.48/2.0, 0.0);
    FR_transform_->Broadcast();

    FL_transform_->setTransform(0.0, 0.0, -(wheelunits_.at("FL").getMeasuredAngleRad()), 0.58/2.0, 0.48/2.0, 0.0);
    FL_transform_->Broadcast();

    BR_transform_->setTransform(0.0, 0.0, -(wheelunits_.at("BR").getMeasuredAngleRad()), -0.58/2.0, -0.48/2.0, 0.0);
    BR_transform_->Broadcast();

    BL_transform_->setTransform(0.0, 0.0, -(wheelunits_.at("BL").getMeasuredAngleRad()), -0.58/2.0, 0.48/2.0, 0.0);
    BL_transform_->Broadcast();
}

void SimWheelController::CB_WheelUnitStatesRequest(const rose_base_msgs::wheelunit_statesGoalConstPtr& goal, SMC* smc)
{
    // Transfer data to the wheel units 
    wheelunits_.at("FR").set_rotation_ = goal->requested_state.angle_FR;
    wheelunits_.at("FL").set_rotation_ = goal->requested_state.angle_FL;
    wheelunits_.at("BR").set_rotation_ = goal->requested_state.angle_BR;
    wheelunits_.at("BL").set_rotation_ = goal->requested_state.angle_BL;

    wheelunits_.at("FR").set_velocity_ = goal->requested_state.velocity_FR;
    wheelunits_.at("FL").set_velocity_ = goal->requested_state.velocity_FL;
    wheelunits_.at("BR").set_velocity_ = goal->requested_state.velocity_BR;
    wheelunits_.at("BL").set_velocity_ = goal->requested_state.velocity_BL;

    ROS_DEBUG_NAMED(ROS_NAME, "Sim CB_WheelUnitStatesRequest: FR[%d, %2.1f] FL[%d, %2.1f] BR[%d, %2.1f] BL[%d, %2.1f]", goal->requested_state.angle_FR, goal->requested_state.velocity_FR, goal->requested_state.angle_FL, goal->requested_state.velocity_FL, goal->requested_state.angle_BR, goal->requested_state.velocity_BR, goal->requested_state.angle_BL, goal->requested_state.velocity_BL);

    // Write to the simulation controller
    rose_base_msgs::wheelunit_statesResult server_result;
    if(writeWheelStates())  //!  @todo OH what if canceled in the mean time?
    {
        ROS_DEBUG_NAMED(ROS_NAME, "Successfully set requested wheelunits states.");

        server_result.return_code = SUCCESS;
        smc_->sendServerResult(true, server_result);
    }
    else
    {
        ROS_DEBUG_NAMED(ROS_NAME, "Failed to set requested wheelunits states.");

        server_result.return_code = LOWLEVEL_ERROR;
        smc_->sendServerResult(false, server_result);
    }
}

void SimWheelController::CB_FR_enc_pos(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_DEBUG_NAMED(ROS_NAME, "CB_FR_enc_pos: %d", msg->data);
    wheelunits_.at("FR").measured_rotation_             = msg->data;   
}

void SimWheelController::CB_FR_enc_vel(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_DEBUG_NAMED(ROS_NAME, "CB_FR_enc_vel: %d", msg->data);
    wheelunits_.at("FR").dT_                            = ros::Time::now().toSec() - FR_prev_T_.toSec();  
    FR_prev_T_                                          = ros::Time::now();  
    wheelunits_.at("FR").measured_velocity_             = (float)msg->data;                                                         // [pulses/s]
    wheelunits_.at("FR").measured_drive_encoder_diff_   = (int)(wheelunits_.at("FR").measured_velocity_*wheelunits_.at("FR").dT_);  // [pulses]
}

void SimWheelController::CB_FL_enc_pos(const std_msgs::Int32::ConstPtr& msg)
{
    WheelUnit wheel_unit("Only For Functions", -1);
    ROS_DEBUG_NAMED(ROS_NAME, "CB_FL_enc_pos: %d", msg->data);
    wheelunits_.at("FL").measured_rotation_             = msg->data;    
}

void SimWheelController::CB_FL_enc_vel(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_DEBUG_NAMED(ROS_NAME, "CB_FL_enc_vel: %d", msg->data);
    wheelunits_.at("FL").dT_                            = ros::Time::now().toSec() - FL_prev_T_.toSec();  
    FL_prev_T_                                          = ros::Time::now();  
    wheelunits_.at("FL").measured_velocity_             = (float)msg->data; 
    wheelunits_.at("FL").measured_drive_encoder_diff_   = (int)(wheelunits_.at("FL").measured_velocity_*wheelunits_.at("FL").dT_);
}

void SimWheelController::CB_BR_enc_pos(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_DEBUG_NAMED(ROS_NAME, "CB_BR_enc_pos: %d", msg->data);
    wheelunits_.at("BR").measured_rotation_             = msg->data;   
}

void SimWheelController::CB_BR_enc_vel(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_DEBUG_NAMED(ROS_NAME, "CB_BR_enc_vel: %d", msg->data);
    wheelunits_.at("BR").dT_                            = ros::Time::now().toSec() - BR_prev_T_.toSec();  
    BR_prev_T_                                          = ros::Time::now();  
    wheelunits_.at("BR").measured_velocity_             = (float)msg->data;
    wheelunits_.at("BR").measured_drive_encoder_diff_   = (int)(wheelunits_.at("BR").measured_velocity_*wheelunits_.at("BR").dT_);
}

void SimWheelController::CB_BL_enc_pos(const std_msgs::Int32::ConstPtr& msg)
{
    WheelUnit wheel_unit("Only For Functions", -1);
    ROS_DEBUG_NAMED(ROS_NAME, "CB_BL_enc_pos: %d", msg->data);
    wheelunits_.at("BL").measured_rotation_             = msg->data;       
}

void SimWheelController::CB_BL_enc_vel(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_DEBUG_NAMED(ROS_NAME, "CB_BL_enc_vel: %d", msg->data);
    wheelunits_.at("BL").dT_                            = ros::Time::now().toSec() - BL_prev_T_.toSec();  
    BL_prev_T_                                          = ros::Time::now();  
    wheelunits_.at("BL").measured_velocity_             = (float)msg->data;
    wheelunits_.at("BL").measured_drive_encoder_diff_   = (int)(wheelunits_.at("BL").measured_velocity_*wheelunits_.at("BL").dT_);
}
