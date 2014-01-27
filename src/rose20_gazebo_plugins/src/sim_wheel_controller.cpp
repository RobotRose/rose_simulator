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

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sim_wheel_controller");
    ros::NodeHandle n;
    ros::Rate r(15);        

    SimWheelController* sim_wheel_controller = new SimWheelController("sim_wheel_controller", n);

    map<string, WheelUnit>* wheel_units = sim_wheel_controller->getWheelUnits();


    float   velocity            = 0.0;
    float   orientation         = 0.0;
    float   vel_step            = (2.0*M_PI)/4.0;
    float   orientation_step    = M_PI/16.0;
    bool    stop                = false;
    ros::Time begin;

    while(n.ok() && !stop)
    {
        begin = ros::Time::now();

        if(!sim_wheel_controller->isEnabled())
            continue;

        // Check the watchdog
        if(!sim_wheel_controller->checkWatchdog())
        {
            ROS_WARN_NAMED("sim_wheel_controller", "Watchdog error!");

            sim_wheel_controller->disable();
            continue;
        }

        if(kbhit())
        {
            uint c = getchar();
            ROS_DEBUG_NAMED("sim_wheel_controller", "Key pressed: %c", (char)c);
            switch(c)
            {
                case '+':   
                    velocity += vel_step;           
                    for(auto it = wheel_units->begin(); it != wheel_units->end(); it++)
                    {
                        it->second.setVelocityRadPerSec(velocity);
                    }
                    break;
                case '-':   
                    velocity -= vel_step;           
                    for(auto it = wheel_units->begin(); it != wheel_units->end(); it++)
                    {
                        it->second.setVelocityRadPerSec(velocity);
                    }
                    break;
                case '*':
                    orientation += orientation_step;
                    ROS_INFO_NAMED("sim_wheel_controller", "Orientation: %.2f", orientation);
                    for(auto it = wheel_units->begin(); it != wheel_units->end(); it++)
                    {
                        it->second.setAngleRad(orientation);    
                    }
                    break;
                case '/':
                    orientation -= orientation_step;
                    ROS_INFO_NAMED("sim_wheel_controller", "Orientation: %.2f", orientation);
                    for(auto it = wheel_units->begin(); it != wheel_units->end(); it++)
                    {
                        it->second.setAngleRad(orientation);
                    }           
                    break;
                case 's':
                    velocity    = 0.0;
                    orientation = 0.0;
                    for(auto it = wheel_units->begin(); it != wheel_units->end(); it++)
                    {
                        it->second.setAngleRad(orientation);
                        it->second.setVelocityRadPerSec(velocity);  
                    }       
                    break;
                case 'x':
                    stop = true;
                    break;
            }
            sim_wheel_controller->writeWheelStates();

            ROS_INFO_NAMED("sim_wheel_controller", "Velocity: %f", velocity);

            ROS_INFO_NAMED("sim_wheel_controller", "Angle   : %f", orientation);
        }

        // Update and publish the wheelunit states if succesfull
        if(sim_wheel_controller->UpdateWheelUnitStates())
            sim_wheel_controller->PublishWheelUnitStates();

        ros::spinOnce();
        r.sleep();

        ros::Time end = ros::Time::now();
        ros::Duration d = end - begin;
        ROS_DEBUG_NAMED("sim_wheel_controller", "Rate: %.2f", 1.0/d.toSec());   
    }

    
    delete sim_wheel_controller;

    return 0;
}

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

    ROS_INFO_NAMED(name_, "Started %s", name_.c_str()); 

    enable();
}

SimWheelController::~SimWheelController()
{
    disable();
    ROS_INFO_NAMED(name_, "Stopped %s", name_.c_str());
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

bool SimWheelController::UpdateWheelUnitStates()
{
    // Done via callback
    return true;
}

bool SimWheelController::PublishWheelUnitStates()
{
    ROS_DEBUG_NAMED(name_, "Sim PublishWheelUnitStates");

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
}

void SimWheelController::CB_WheelUnitStatesRequest(const roscomm::wheelunit_states::ConstPtr& wheelunit_states)
{
    ROS_INFO_NAMED(name_, "Sim CB_WheelUnitStatesRequest");

    // Transfer data to the wheel units 
    wheelunits_.at("FR").set_rotation_ = wheelunit_states->angle_FR;
    wheelunits_.at("FL").set_rotation_ = wheelunit_states->angle_FL;
    wheelunits_.at("BR").set_rotation_ = wheelunit_states->angle_BR;
    wheelunits_.at("BL").set_rotation_ = wheelunit_states->angle_BL;

    wheelunits_.at("FR").set_velocity_ = wheelunit_states->velocity_FR;
    wheelunits_.at("FL").set_velocity_ = wheelunit_states->velocity_FL;
    wheelunits_.at("BR").set_velocity_ = wheelunit_states->velocity_BR;
    wheelunits_.at("BL").set_velocity_ = wheelunit_states->velocity_BL;

    writeWheelStates();
}

void SimWheelController::CB_FR_enc_pos(const std_msgs::Float64::ConstPtr& msg)
{
    wheelunits_.at("FR").measured_rotation_ = msg->data;
}

void SimWheelController::CB_FR_enc_vel(const std_msgs::Float64::ConstPtr& msg)
{
    wheelunits_.at("FR").measured_velocity_ = msg->data;
}

void SimWheelController::CB_FL_enc_pos(const std_msgs::Float64::ConstPtr& msg)
{
    wheelunits_.at("FL").measured_rotation_ = msg->data;
}

void SimWheelController::CB_FL_enc_vel(const std_msgs::Float64::ConstPtr& msg)
{
    wheelunits_.at("FL").measured_velocity_ = msg->data;
}

void SimWheelController::CB_BR_enc_pos(const std_msgs::Float64::ConstPtr& msg)
{
    wheelunits_.at("BR").measured_rotation_ = msg->data;
}

void SimWheelController::CB_BR_enc_vel(const std_msgs::Float64::ConstPtr& msg)
{
    wheelunits_.at("BR").measured_velocity_ = msg->data;
}

void SimWheelController::CB_BL_enc_pos(const std_msgs::Float64::ConstPtr& msg)
{
    wheelunits_.at("BL").measured_rotation_ = msg->data;
}

void SimWheelController::CB_BL_enc_vel(const std_msgs::Float64::ConstPtr& msg)
{
    wheelunits_.at("BL").measured_velocity_ = msg->data;
}
