
#include <ros/ros.h>
#include "../include/leg_inverse_kinematics.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "locomotion_node");

    ros::NodeHandle nh_private("~");
    //tf::TransformListener tf(ros::Duration(10));
    LegInverseKinematicsProcessor IK = LegInverseKinematicsProcessor(nh_private);
    
    ros::spin();
    return 0;
}