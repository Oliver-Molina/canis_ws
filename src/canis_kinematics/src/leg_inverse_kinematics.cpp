#include <math.h>
#include <iomanip>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <thread>
#include <tf/tf.h>

#include "../include/leg_inverse_kinematics.h"


LegInverseKinematicsProcessor::LegInverseKinematicsProcessor(const ros::NodeHandle &nh_private_) {
    
    SuperiorRightSub = nh_.subscribe<geometry_msgs::PointStamped>("/command/leg/pos/superior/right", 1000, &LegInverseKinematicsProcessor::Superior_Right_Leg_Pos_CB, this);
    SuperiorLeftSub = nh_.subscribe<geometry_msgs::PointStamped>("/command/leg/pos/superior/left", 1000, &LegInverseKinematicsProcessor::Superior_Left_Leg_Pos_CB, this);
    InferiorRightSub = nh_.subscribe<geometry_msgs::PointStamped>("/command/leg/pos/inferior/right", 1000, &LegInverseKinematicsProcessor::Inferior_Right_Leg_Pos_CB, this);
    InferiorLeftSub = nh_.subscribe<geometry_msgs::PointStamped>("/command/leg/pos/inferior/left", 1000, &LegInverseKinematicsProcessor::Inferior_Left_Leg_Pos_CB, this);

    superior_right_shoulder_pub = nh_.advertise<std_msgs::Float64>("/command/leg/angle/shoulder/superior/right", 1000);
    superior_left_shoulder_pub = nh_.advertise<std_msgs::Float64>("/command/leg/angle/shoulder/superior/left", 1000);
    inferior_right_shoulder_pub = nh_.advertise<std_msgs::Float64>("/command/leg/angle/shoulder/inferior/right", 1000);
    inferior_left_shoulder_pub = nh_.advertise<std_msgs::Float64>("/command/leg/angle/shoulder/inferior/left", 1000);

    superior_right_arm_pub = nh_.advertise<std_msgs::Float64>("/command/leg/angle/arm/superior/right", 1000);
    superior_left_arm_pub = nh_.advertise<std_msgs::Float64>("/command/leg/angle/arm/superior/left", 1000);
    inferior_right_arm_pub = nh_.advertise<std_msgs::Float64>("/command/leg/angle/arm/inferior/right", 1000);
    inferior_left_arm_pub = nh_.advertise<std_msgs::Float64>("/command/leg/angle/arm/inferior/left", 1000);

    superior_right_forearm_pub = nh_.advertise<std_msgs::Float64>("/command/leg/angle/forearm/superior/right", 1000);
    superior_left_forearm_pub = nh_.advertise<std_msgs::Float64>("/command/leg/angle/forearm/superior/left", 1000);
    inferior_right_forearm_pub = nh_.advertise<std_msgs::Float64>("/command/leg/angle/forearm/inferior/right", 1000);
    inferior_left_forearm_pub = nh_.advertise<std_msgs::Float64>("/command/leg/angle/forearm/inferior/left", 1000);

    test_pwm = nh_.advertise<std_msgs::String>("/test_pwm", 1000);

    debug_pub = nh_.advertise<std_msgs::String>("/debug", 1000);

    nh_.param<double>("/shoulder_length", shoulder_length, 0.055);
    nh_.param<double>("/arm_length", arm_length, 0.105);
    nh_.param<double>("/forearm_length", forearm_length, 0.136);
}

void LegInverseKinematicsProcessor::Superior_Right_Leg_Pos_CB(const geometry_msgs::PointStamped::ConstPtr& Point) {

    double superior_right_x = Point->point.x;
    double superior_right_y = -Point->point.y;
    double superior_right_z = Point->point.z;

    double dyz = sqrt(superior_right_y * superior_right_y + superior_right_z * superior_right_z);

    double lyz = sqrt(dyz * dyz - shoulder_length * shoulder_length);

    double g1 = atan2(superior_right_y, superior_right_z);

    double g2 = atan2(shoulder_length, lyz);

    double lxz = sqrt(lyz * lyz + superior_right_x * superior_right_x);

    double n = (lxz * lxz - forearm_length * forearm_length - arm_length * arm_length) * 1 / (2 * arm_length);

    double a1 = atan2(superior_right_x, lyz);

    double a2 = -acos((arm_length + n) / lxz);

    double forearm_extensor_pos = acos(n / forearm_length);

    double arm_extensor_pos = -(a1 + a2);

    double shoulder_abductor_pos = -(g1 + g2) + M_PI;

    superior_right_shoulder_msg.data = shoulder_abductor_pos;
    superior_right_arm_msg.data = arm_extensor_pos;
    superior_right_forearm_msg.data = forearm_extensor_pos;

    superior_right_shoulder_pub.publish(superior_right_shoulder_msg);
    superior_right_arm_pub.publish(superior_right_arm_msg);
    superior_right_forearm_pub.publish(superior_right_forearm_msg);

}

void LegInverseKinematicsProcessor::Superior_Left_Leg_Pos_CB(const geometry_msgs::PointStamped::ConstPtr&Point) {

    double superior_left_x = Point->point.x;
    double superior_left_y = Point->point.y;
    double superior_left_z = Point->point.z;
    
    double dyz = sqrt(superior_left_y * superior_left_y + superior_left_z * superior_left_z);
    
    double lyz = sqrt(dyz * dyz - shoulder_length * shoulder_length);
    
    double g1 = atan2(superior_left_y, superior_left_z);
    
    double g2 = atan2(shoulder_length, lyz);
    
    double lxz = sqrt(lyz * lyz + superior_left_x * superior_left_x);
    
    double n = (lxz * lxz - forearm_length * forearm_length - arm_length * arm_length) * 1 / (2 * arm_length);
    
    double a1 = atan2(superior_left_x, lyz);
    
    double a2 = -acos((arm_length + n) / lxz);
    
    double forearm_extensor_pos = acos(n / forearm_length);
    
    double arm_extensor_pos = -(a1 + a2);
    
    double shoulder_abductor_pos = (-(g1 + g2)) + M_PI;
    

    superior_left_shoulder_msg.data = shoulder_abductor_pos;
    superior_left_arm_msg.data = arm_extensor_pos;
    superior_left_forearm_msg.data = forearm_extensor_pos;

    superior_left_shoulder_pub.publish(superior_left_shoulder_msg);
    superior_left_arm_pub.publish(superior_left_arm_msg);
    superior_left_forearm_pub.publish(superior_left_forearm_msg);

    std::ostringstream stringstream;
    stringstream << std::fixed << "x: " << superior_left_x << " y: " << superior_left_y << " z: " << superior_left_z << std::endl;
    stringstream << std::fixed << "dyz: " << dyz << std::endl;
    stringstream << std::fixed << "lyz: " << lyz << std::endl;
    stringstream << std::fixed << "g1: " << g1 << std::endl;
    stringstream << std::fixed << "g2: " << g2 << std::endl;
    stringstream << std::fixed << "lxz: " << lxz << std::endl;
    stringstream << std::fixed << "n: " << n << std::endl;
    stringstream << std::fixed << "a1: " << a1 << std::endl;
    stringstream << std::fixed << "a2: " << a2 << std::endl;
    stringstream << std::fixed << "forearm: " << forearm_extensor_pos << std::endl;
    stringstream << std::fixed << "arm: " << arm_extensor_pos << std::endl;
    stringstream << std::fixed << "shoulder: " << shoulder_abductor_pos << std::endl;
    std::string str = stringstream.str();
    debug_msg.data = str.c_str();
    //debug_pub.publish(debug_msg);



    //debug_msg.data = "Hello";
    //debug_pub.publish(debug_msg);
}

void LegInverseKinematicsProcessor::Inferior_Right_Leg_Pos_CB(const geometry_msgs::PointStamped::ConstPtr& Point) {

    double inferior_right_x = Point->point.x;
    double inferior_right_y = -Point->point.y;
    double inferior_right_z = Point->point.z;

    double dyz = sqrt(inferior_right_y * inferior_right_y + inferior_right_z * inferior_right_z);

    double lyz = sqrt(dyz * dyz - shoulder_length * shoulder_length);

    double g1 = atan2(inferior_right_y, inferior_right_z);

    double g2 = atan2(shoulder_length, lyz);

    double lxz = sqrt(lyz * lyz + inferior_right_x * inferior_right_x);

    double n = (lxz * lxz - forearm_length * forearm_length - arm_length * arm_length) * 1 / (2 * arm_length);

    double a1 = atan2(inferior_right_x, lyz);

    double a2 = -acos((arm_length + n) / lxz);

    double forearm_extensor_pos = acos(n / forearm_length);

    double arm_extensor_pos = -(a1 + a2);

    double shoulder_abductor_pos = (-(g1 + g2)) + M_PI;

    inferior_right_shoulder_msg.data = shoulder_abductor_pos;
    inferior_right_arm_msg.data = arm_extensor_pos;
    inferior_right_forearm_msg.data = forearm_extensor_pos;

    inferior_right_shoulder_pub.publish(inferior_right_shoulder_msg);
    inferior_right_arm_pub.publish(inferior_right_arm_msg);
    inferior_right_forearm_pub.publish(inferior_right_forearm_msg);

    std::ostringstream stringstream;
    //stringstream << std::fixed << "x: " << inferior_right_x << " y: " << inferior_right_y << " z: " << inferior_right_z << std::endl;
    //stringstream << std::fixed << "dyz: " << dyz << std::endl;
    //stringstream << std::fixed << "lyz: " << lyz << std::endl;
    //stringstream << std::fixed << "g1: " << g1 << std::endl;
    //stringstream << std::fixed << "g2: " << g2 << std::endl;
    //stringstream << std::fixed << "lxz: " << lxz << std::endl;
    //stringstream << std::fixed << "n: " << n << std::endl;
    //stringstream << std::fixed << "a1: " << a1 << std::endl;
    //stringstream << std::fixed << "a2: " << a2 << std::endl;
    //stringstream << std::fixed << "forearm: " << forearm_extensor_pos << std::endl;
    //stringstream << std::fixed << "arm: " << arm_extensor_pos << std::endl;
    //stringstream << std::fixed << "shoulder: " << shoulder_abductor_pos << std::endl;
    std::string str = stringstream.str();
    debug_msg.data = str.c_str();
    //debug_pub.publish(debug_msg);

}

void LegInverseKinematicsProcessor::Inferior_Left_Leg_Pos_CB(const geometry_msgs::PointStamped::ConstPtr& Point) {

    double inferior_left_x = Point->point.x;
    double inferior_left_y = -Point->point.y;
    double inferior_left_z = Point->point.z;

    double dyz = sqrt(inferior_left_y * inferior_left_y + inferior_left_z * inferior_left_z);

    double lyz = sqrt(dyz * dyz - shoulder_length * shoulder_length);

    double g1 = atan2(inferior_left_y, inferior_left_z);

    double g2 = atan2(shoulder_length, lyz);

    double lxz = sqrt(lyz * lyz + inferior_left_x * inferior_left_x);

    double n = (lxz * lxz - forearm_length * forearm_length - arm_length * arm_length) * 1 / (2 * arm_length);

    double a1 = atan2(inferior_left_x, lyz);

    double a2 = -acos((arm_length + n) / lxz);

    double forearm_extensor_pos = acos(n / forearm_length);

    double arm_extensor_pos = -(a1 + a2);

    double shoulder_abductor_pos = (-(g1 - g2)) - M_PI;

    inferior_left_shoulder_msg.data = shoulder_abductor_pos;
    inferior_left_arm_msg.data = arm_extensor_pos;
    inferior_left_forearm_msg.data = forearm_extensor_pos;

    inferior_left_shoulder_pub.publish(inferior_left_shoulder_msg);
    inferior_left_arm_pub.publish(inferior_left_arm_msg);
    inferior_left_forearm_pub.publish(inferior_left_forearm_msg);

    std::ostringstream stringstream;
    stringstream << std::fixed << "x: " << inferior_left_x << " y: " << inferior_left_y << " z: " << inferior_left_z << std::endl;
    stringstream << std::fixed << "dyz: " << dyz << std::endl;
    stringstream << std::fixed << "lyz: " << lyz << std::endl;
    stringstream << std::fixed << "g1: " << g1 << std::endl;
    stringstream << std::fixed << "g2: " << g2 << std::endl;
    stringstream << std::fixed << "lxz: " << lxz << std::endl;
    stringstream << std::fixed << "n: " << n << std::endl;
    stringstream << std::fixed << "a1: " << a1 << std::endl;
    stringstream << std::fixed << "a2: " << a2 << std::endl;
    stringstream << std::fixed << "forearm: " << forearm_extensor_pos << std::endl;
    stringstream << std::fixed << "arm: " << arm_extensor_pos << std::endl;
    stringstream << std::fixed << "shoulder: " << shoulder_abductor_pos << std::endl;
    std::string str = stringstream.str();
    debug_msg.data = str.c_str();
    //debug_pub.publish(debug_msg);
}


