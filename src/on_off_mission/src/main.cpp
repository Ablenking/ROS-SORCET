
#include <ros/ros.h>
#include "taskmanager.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "off_mission_main");


    off_mission::TaskManager m;

    ros::AsyncSpinner aspin(2);
    aspin.start();
    ros::waitForShutdown();
    return(0);
}
