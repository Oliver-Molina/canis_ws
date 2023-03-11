
#include <ros/ros.h>
#include "../include/gait_planner.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "gait_planner_node");
    ros::NodeHandle nh_private("~");

    GaitPlanner gp = GaitPlanner(nh_private);
    gp.Init();
    
    ros::spin();
    return 0;
}