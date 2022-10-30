#include <ros/ros.h>

// Subs
ros::Subscriber SuperiorRightSub;
ros::Subscriber SuperiorLeftSub;
ros::Subscriber InferiorRightSub;
ros::Subscriber InferiorLeftSub;

// Pubs
ros::Publisher superior_right_shoulder_pub;
ros::Publisher superior_left_shoulder_pub;
ros::Publisher inferior_right_shoulder_pub;
ros::Publisher inferior_left_shoulder_pub;

ros::Publisher superior_right_arm_pub;
ros::Publisher superior_left_arm_pub;
ros::Publisher inferior_right_arm_pub;
ros::Publisher inferior_left_arm_pub;

ros::Publisher superior_right_forearm_pub;
ros::Publisher superior_left_forearm_pub;
ros::Publisher inferior_right_forearm_pub;
ros::Publisher inferior_left_forearm_pub;

// msgs
std_msgs::Float64 superior_right_shoulder_msg;
std_msgs::Float64 superior_right_arm_msg;
std_msgs::Float64 superior_right_forearm_msg;

std_msgs::Float64 superior_left_shoulder_msg;
std_msgs::Float64 superior_left_arm_msg;
std_msgs::Float64 superior_left_forearm_msg;

std_msgs::Float64 inferior_right_shoulder_msg;
std_msgs::Float64 inferior_right_arm_msg;
std_msgs::Float64 inferior_right_forearm_msg;

std_msgs::Float64 inferior_left_shoulder_msg;
std_msgs::Float64 inferior_left_arm_msg;
std_msgs::Float64 inferior_left_forearm_msg;