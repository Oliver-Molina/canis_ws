#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <string>
#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>

#include "../include/fileio.h"
#include "../include/kinematics.h"
#include "../include/robot_const.h"
#include "../include/ros_setup.h"

using namespace std;

#define shoulder_length 0.055
#define arm_length 0.105
#define forearm_length 0.1

#define precision 0.003 // Works: 0.003, 0.005
#define leng sqrt(pow(arm_length + forearm_length, 2) + shoulder_length * shoulder_length)
#define edge ((int) (leng / precision))

double shoulder_ang = 0;
double arm_ang = 0;
double forearm_ang = 0;

int arrayLen = getFileSize("/home/nick/canis_ws/src/canis_kinematics_lut/textfiles/shoulder_array.txt");

double* newShoulderArray = readFileToArray("/home/nick/canis_ws/src/canis_kinematics_lut/textfiles/shoulder_array.txt", arrayLen);
double* newArmArray = readFileToArray("/home/nick/canis_ws/src/canis_kinematics_lut/textfiles/arm_array.txt", arrayLen);
double* newForearmArray = readFileToArray("/home/nick/canis_ws/src/canis_kinematics_lut/textfiles/forearm_array.txt", arrayLen);

void Superior_Right_Leg_Pos_CB(const geometry_msgs::PointStamped::ConstPtr& Point) {
    double x = Point->point.x;
    double y = Point->point.y;
    double z = Point->point.z;

    superior_right_shoulder_msg.data = lerpLUT(newShoulderArray, precision, leng, edge, x, y, z);
    superior_right_arm_msg.data = lerpLUT(newArmArray, precision, leng, edge, x, y, z);
    superior_right_forearm_msg.data = lerpLUT(newForearmArray, precision, leng, edge, x, y, z);

    superior_right_shoulder_pub.publish(superior_right_shoulder_msg);
    superior_right_arm_pub.publish(superior_right_arm_msg);
    superior_right_forearm_pub.publish(superior_right_forearm_msg);
}

void Superior_Left_Leg_Pos_CB(const geometry_msgs::PointStamped::ConstPtr& Point) {
    double x = Point->point.x;
    double y = Point->point.y;
    double z = Point->point.z;

    superior_left_shoulder_msg.data = lerpLUT(newShoulderArray, precision, leng, edge, x, y, z);
    superior_left_arm_msg.data = lerpLUT(newArmArray, precision, leng, edge, x, y, z);
    superior_left_forearm_msg.data = lerpLUT(newForearmArray, precision, leng, edge, x, y, z);

    superior_left_shoulder_pub.publish(superior_left_shoulder_msg);
    superior_left_arm_pub.publish(superior_left_arm_msg);
    superior_left_forearm_pub.publish(superior_left_forearm_msg);
}

void Inferior_Right_Leg_Pos_CB(const geometry_msgs::PointStamped::ConstPtr& Point) {
    double x = Point->point.x;
    double y = Point->point.y;
    double z = Point->point.z;

    inferior_right_shoulder_msg.data = lerpLUT(newShoulderArray, precision, leng, edge, x, y, z);
    inferior_right_arm_msg.data = lerpLUT(newArmArray, precision, leng, edge, x, y, z);
    inferior_right_forearm_msg.data = lerpLUT(newForearmArray, precision, leng, edge, x, y, z);

    inferior_right_shoulder_pub.publish(inferior_right_shoulder_msg);
    inferior_right_arm_pub.publish(inferior_right_arm_msg);
    inferior_right_forearm_pub.publish(inferior_right_forearm_msg);
}

void Inferior_Left_Leg_Pos_CB(const geometry_msgs::PointStamped::ConstPtr& Point) {
    double x = Point->point.x;
    double y = Point->point.y;
    double z = Point->point.z;

    inferior_left_shoulder_msg.data = lerpLUT(newShoulderArray, precision, leng, edge, x, y, z);
    inferior_left_arm_msg.data = lerpLUT(newArmArray, precision, leng, edge, x, y, z);
    inferior_left_forearm_msg.data = lerpLUT(newForearmArray, precision, leng, edge, x, y, z);

    inferior_left_shoulder_pub.publish(inferior_left_shoulder_msg);
    inferior_left_arm_pub.publish(inferior_left_arm_msg);
    inferior_left_forearm_pub.publish(inferior_left_forearm_msg);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "talker");
    ros::NodeHandle nh_;

    SuperiorRightSub = nh_.subscribe<geometry_msgs::PointStamped>("/desired_pos/superior/right", 1000, &Superior_Right_Leg_Pos_CB);
    SuperiorLeftSub = nh_.subscribe<geometry_msgs::PointStamped>("/desired_pos/superior/left", 1000, &Superior_Left_Leg_Pos_CB);
    InferiorRightSub = nh_.subscribe<geometry_msgs::PointStamped>("/desired_pos/inferior/right", 1000, &Inferior_Right_Leg_Pos_CB);
    InferiorLeftSub = nh_.subscribe<geometry_msgs::PointStamped>("/desired_pos/inferior/left", 1000, &Inferior_Left_Leg_Pos_CB);

    superior_right_shoulder_pub = nh_.advertise<std_msgs::Float64>("/actuation/leg/shoulder/superior/right", 1000);
    superior_left_shoulder_pub = nh_.advertise<std_msgs::Float64>("/actuation/leg/shoulder/superior/left", 1000);
    inferior_right_shoulder_pub = nh_.advertise<std_msgs::Float64>("/actuation/leg/shoulder/inferior/right", 1000);
    inferior_left_shoulder_pub = nh_.advertise<std_msgs::Float64>("/actuation/leg/shoulder/inferior/left", 1000);

    superior_right_arm_pub = nh_.advertise<std_msgs::Float64>("/actuation/leg/arm/superior/right", 1000);
    superior_left_arm_pub = nh_.advertise<std_msgs::Float64>("/actuation/leg/arm/superior/left", 1000);
    inferior_right_arm_pub = nh_.advertise<std_msgs::Float64>("/actuation/leg/arm/inferior/right", 1000);
    inferior_left_arm_pub = nh_.advertise<std_msgs::Float64>("/actuation/leg/arm/inferior/left", 1000);

    superior_right_forearm_pub = nh_.advertise<std_msgs::Float64>("/actuation/leg/forearm/superior/right", 1000);
    superior_left_forearm_pub = nh_.advertise<std_msgs::Float64>("/actuation/leg/forearm/superior/left", 1000);
    inferior_right_forearm_pub = nh_.advertise<std_msgs::Float64>("/actuation/leg/forearm/inferior/right", 1000);
    inferior_left_forearm_pub = nh_.advertise<std_msgs::Float64>("/actuation/leg/forearm/inferior/left", 1000);

    ros::spin();


    return 1;
}
