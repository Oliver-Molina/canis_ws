
#include <ros/ros.h>
#include "../include/com_kinematics.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "com_kinematics_node");

    ros::NodeHandle nh_private("~");
    //tf::TransformListener tf(ros::Duration(10));
    ComKinematicsProcessor IK = ComKinematicsProcessor(nh_private);
    
    ros::spin();
    return 0;
}