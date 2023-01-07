
#include <ros/ros.h>
#include "../include/canis_locomotion.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "locomotion_node");

    ros::NodeHandle nh_private("~");
    //tf::TransformListener tf(ros::Duration(10));
    LocomotionProcessor lp = LocomotionProcessor(nh_private);
    lp.Init();

    ros::Timer timer = nh_private.createTimer(ros::Duration(1.0 / lp.operating_freq), &LocomotionProcessor::Pos_Update, &lp);
    
    ros::spin();
    return 0;
}