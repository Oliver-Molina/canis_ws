
#include <ros/ros.h>
#include "../include/gait_planner.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "gait_planner_node");
    ros::NodeHandle nh_private("~");

    GaitPlanner gp = GaitPlanner(nh_private);
    gp.Init();
    ros::Timer timer = nh_private.createTimer(ros::Duration(1.0 / gp.operating_freq), &GaitPlanner::Frame_CB, &gp);
    ros::spin();
    return 0;
}