#include "rose20_gazebo_plugins/sim_wheel_controller_node.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sim_wheel_unit_controller");
    ros::NodeHandle n;
    ros::Rate r(10);        

    SimWheelController* sim_wheel_controller = new SimWheelController("sim_wheel_controller", n);

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
            ROS_WARN_NAMED(ROS_NAME, "Watchdog error!");
            sim_wheel_controller->disable();
            continue;
        }

        if(kbhit())
        {
            uint c = getchar();
            ROS_DEBUG_NAMED(ROS_NAME, "Key pressed: %c", (char)c);
            switch(c)
            {
                case 'x':
                    stop = true;
                    break;
            }
        }

        // Update and publish the wheelunit states if succesfull
        if(sim_wheel_controller->UpdateWheelUnitControllers())
            sim_wheel_controller->PublishWheelUnitStates();

        ros::spinOnce();
        r.sleep();

        ros::Time end = ros::Time::now();
        ros::Duration d = end - begin;
        ROS_DEBUG_NAMED(ROS_NAME, "Rate: %.2f", 1.0/d.toSec());   
    }

    
    delete sim_wheel_controller;

    return 0;
}
