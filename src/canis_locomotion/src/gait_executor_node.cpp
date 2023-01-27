
#include <ros/ros.h>
#include "../include/gait_executor.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gait_executor_node");

    ros::NodeHandle nh_private("~");

    GaitExecutor ge = GaitExecutor(nh_private);
    ge.Init();

    ros::Timer timer = nh_private.createTimer(ros::Duration(1.0 / lp.operating_freq), &GaitExecutor::Vel_Update, &ge);
    
    ros::spin();
    return 0;
}