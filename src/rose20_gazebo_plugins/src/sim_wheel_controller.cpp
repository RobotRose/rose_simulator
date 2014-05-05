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
{
    n_           	= n;
    name_       	= name;
    enabled_ 	 	= false;

    // Publishers
    wheelunit_states_pub_   = n.advertise<roscomm::wheelunit_states>("/wheel_controller/wheelunit_states", 1);
    FR_caster_pub_          = n.advertise<std_msgs::Float64>("/sim_wheel_controller/FR_caster/req_pos", 1);
    FR_wheel_pub_           = n.advertise<std_msgs::Float64>("/sim_wheel_controller/FR_wheel/req_vel", 1);
    FL_caster_pub_          = n.advertise<std_msgs::Float64>("/sim_wheel_controller/FL_caster/req_pos", 1);
    FL_wheel_pub_           = n.advertise<std_msgs::Float64>("/sim_wheel_controller/FL_wheel/req_vel", 1);
    BR_caster_pub_          = n.advertise<std_msgs::Float64>("/sim_wheel_controller/BR_caster/req_pos", 1);
    BR_wheel_pub_           = n.advertise<std_msgs::Float64>("/sim_wheel_controller/BR_wheel/req_vel", 1);
    BL_caster_pub_          = n.advertise<std_msgs::Float64>("/sim_wheel_controller/BL_caster/req_pos", 1);
    BL_wheel_pub_           = n.advertise<std_msgs::Float64>("/sim_wheel_controller/BL_wheel/req_vel", 1);

    // Subscribers
    wheelunit_states_sub_   = n_.subscribe("/drive_controller/wheelunit_states_request", 1, &SimWheelController::CB_WheelUnitStatesRequest, this);
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
    wheelunit_controllers_.insert( std::pair<string, SimWheelUnitController*>("FL", new SimWheelUnitController(n_, "FL", 1)));
    wheelunit_controllers_.insert( std::pair<string, SimWheelUnitController*>("BR", new SimWheelUnitController(n_, "BR", 1)));
    wheelunit_controllers_.insert( std::pair<string, SimWheelUnitController*>("BL", new SimWheelUnitController(n_, "BL", 1)));

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
    std_msgs::Float64 msg_rot;
    std_msgs::Float64 msg_vel;    

    msg_rot.data = wheelunits_.at("FR").getAngleLowLevel();
    msg_vel.data = wheelunits_.at("FR").getVelocityLowLevel();
    FR_caster_pub_.publish(msg_rot);
    FR_wheel_pub_.publish(msg_vel); 

    msg_rot.data = wheelunits_.at("FL").getAngleLowLevel();
    msg_vel.data = wheelunits_.at("FL").getVelocityLowLevel();
    FL_caster_pub_.publish(msg_rot);
    FL_wheel_pub_.publish(msg_vel);

    msg_rot.data = wheelunits_.at("BR").getAngleLowLevel();
    msg_vel.data = wheelunits_.at("BR").getVelocityLowLevel(); 
    BR_caster_pub_.publish(msg_rot);
    BR_wheel_pub_.publish(msg_vel);

    msg_rot.data = wheelunits_.at("BL").getAngleLowLevel();
    msg_vel.data = wheelunits_.at("BL").getVelocityLowLevel();  
    BL_caster_pub_.publish(msg_rot);
    BL_wheel_pub_.publish(msg_vel);  

    return true;
}

bool SimWheelController::UpdateWheelUnitControllers()
{
    pair<string, SimWheelUnitController*> a_pair;
    BOOST_FOREACH(a_pair, wheelunit_controllers_) 
        a_pair.second->update();

    return true;
}

bool SimWheelController::PublishWheelUnitStates()
{
    ROS_DEBUG_NAMED(ROS_NAME, "Sim PublishWheelUnitStates");

    // Fill message with lowlevel values
    roscomm::wheelunit_states wheelunit_states; 
    wheelunit_states.angle_FR       = (float)wheelunits_.at("FR").measured_rotation_;   
    wheelunit_states.angle_FL       = (float)wheelunits_.at("FL").measured_rotation_;
    wheelunit_states.angle_BR       = (float)wheelunits_.at("BR").measured_rotation_;
    wheelunit_states.angle_BL       = (float)wheelunits_.at("BL").measured_rotation_;

    wheelunit_states.velocity_FR    = wheelunits_.at("FR").measured_velocity_;
    wheelunit_states.velocity_FL    = wheelunits_.at("FL").measured_velocity_;
    wheelunit_states.velocity_BR    = wheelunits_.at("BR").measured_velocity_;
    wheelunit_states.velocity_BL    = wheelunits_.at("BL").measured_velocity_;

    wheelunit_states_pub_.publish(wheelunit_states);

    // Publish wheel transforms    
    FR_transform_->setTransform(0.0, 0.0, -wheelunits_.at("FR").getMeasuredAngleRad(), 0.39, -0.29, 0.0);
    FR_transform_->Broadcast();

    FL_transform_->setTransform(0.0, 0.0, -wheelunits_.at("FL").getMeasuredAngleRad(), 0.39, 0.29, 0.0);
    FL_transform_->Broadcast();

    BR_transform_->setTransform(0.0, 0.0, -wheelunits_.at("BR").getMeasuredAngleRad(), -0.39, -0.29, 0.0);
    BR_transform_->Broadcast();

    BL_transform_->setTransform(0.0, 0.0, -wheelunits_.at("BL").getMeasuredAngleRad(), -0.39, 0.29, 0.0);
    BL_transform_->Broadcast();
}

void SimWheelController::CB_WheelUnitStatesRequest(const roscomm::wheelunit_states::ConstPtr& wheelunit_states)
{
        // Transfer data to the wheel units 
    wheelunits_.at("FR").set_rotation_ = wheelunit_states->angle_FR;
    wheelunits_.at("FL").set_rotation_ = wheelunit_states->angle_FL;
    wheelunits_.at("BR").set_rotation_ = wheelunit_states->angle_BR;
    wheelunits_.at("BL").set_rotation_ = wheelunit_states->angle_BL;

    wheelunits_.at("FR").set_velocity_ = wheelunit_states->velocity_FR;
    wheelunits_.at("FL").set_velocity_ = wheelunit_states->velocity_FL;
    wheelunits_.at("BR").set_velocity_ = wheelunit_states->velocity_BR;
    wheelunits_.at("BL").set_velocity_ = wheelunit_states->velocity_BL;

    ROS_DEBUG_NAMED(ROS_NAME, "Sim CB_WheelUnitStatesRequest: FR[%2.1f, %2.1f] FL[%2.1f, %2.1f] BR[%2.1f, %2.1f] BL[%2.1f, %2.1f]", wheelunit_states->angle_FR, wheelunit_states->velocity_FR, wheelunit_states->angle_FL, wheelunit_states->velocity_FL, wheelunit_states->angle_BR, wheelunit_states->velocity_BR, wheelunit_states->angle_BL, wheelunit_states->velocity_BL);

    writeWheelStates();
}

void SimWheelController::CB_FR_enc_pos(const std_msgs::Float64::ConstPtr& msg)
{
    ROS_DEBUG_NAMED("SimWheelController", "CB_FR_enc_pos: %.2f", msg->data);
    wheelunits_.at("FR").measured_rotation_ = msg->data;
}

void SimWheelController::CB_FR_enc_vel(const std_msgs::Float64::ConstPtr& msg)
{
    ROS_DEBUG_NAMED("SimWheelController", "CB_FR_enc_vel: %.2f", msg->data);
    wheelunits_.at("FR").measured_velocity_ = msg->data;
}

void SimWheelController::CB_FL_enc_pos(const std_msgs::Float64::ConstPtr& msg)
{
    ROS_DEBUG_NAMED("SimWheelController", "CB_FL_enc_pos: %.2f", msg->data);
    wheelunits_.at("FL").measured_rotation_ = msg->data;
}

void SimWheelController::CB_FL_enc_vel(const std_msgs::Float64::ConstPtr& msg)
{
    ROS_DEBUG_NAMED("SimWheelController", "CB_FL_enc_vel: %.2f", msg->data);
    wheelunits_.at("FL").measured_velocity_ = msg->data;
}

void SimWheelController::CB_BR_enc_pos(const std_msgs::Float64::ConstPtr& msg)
{
    ROS_DEBUG_NAMED("SimWheelController", "CB_BR_enc_pos: %.2f", msg->data);
    wheelunits_.at("BR").measured_rotation_ = msg->data;
}

void SimWheelController::CB_BR_enc_vel(const std_msgs::Float64::ConstPtr& msg)
{
    ROS_DEBUG_NAMED("SimWheelController", "CB_BR_enc_vel: %.2f", msg->data);
    wheelunits_.at("BR").measured_velocity_ = msg->data;
}

void SimWheelController::CB_BL_enc_pos(const std_msgs::Float64::ConstPtr& msg)
{
    ROS_DEBUG_NAMED("SimWheelController", "CB_BL_enc_pos: %.2f", msg->data);
    wheelunits_.at("BL").measured_rotation_ = msg->data;
}

void SimWheelController::CB_BL_enc_vel(const std_msgs::Float64::ConstPtr& msg)
{
    ROS_DEBUG_NAMED("SimWheelController", "CB_BL_enc_vel: %.2f", msg->data);
    wheelunits_.at("BL").measured_velocity_ = msg->data;
}
