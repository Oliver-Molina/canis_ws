#include <math.h>
#include <iomanip>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>

#include "../include/canis_locomotion.h"


LocomotionProcessor::LocomotionProcessor(const ros::NodeHandle &nh_private_) {
    
    vel_sub = nh_.subscribe<geometry_msgs::TwistStamped>("/command/twist", 1000, &LocomotionProcessor::Twist_CB, this);

    sr_pub = nh_.advertise<geometry_msgs::PointStamped>("/desired_pos/superior/right", 1000);
    sl_pub = nh_.advertise<geometry_msgs::PointStamped>("/desired_pos/superior/left", 1000);
    ir_pub = nh_.advertise<geometry_msgs::PointStamped>("/desired_pos/inferior/right", 1000);
    il_pub = nh_.advertise<geometry_msgs::PointStamped>("/desired_pos/inferior/left", 1000);

    debug_pub = nh_.advertise<std_msgs::String>("/debug", 1000);

    // #### Robot Params ####
    shoulder_length = 0.055;
    arm_length = 0.105;
    forearm_length = 0.136;

    body_width = 0.2;
    center_to_front = 0.1;
    center_to_back = 0.1;

    operating_freq = 30; // TBD, more testing

    // #### State Variables ####
    timer = nh.createTimer(ros::Duration(1.0 / operating_freq), LocomotionProcessor::Pos_Update);

    x_vel = 0;
    y_vel = 0;
    theta_vel = 0;

    walking_z = 0.1;

        // Note, these are relative to COM
    sr_x = 0;
    sr_y = 0;
    sr_z = 0;

    sr_l = 0;

    sl_x = 0;
    sl_y = 0;
    sl_z = 0;

    sl_l = 0;

    ir_x = 0;
    ir_y = 0;
    ir_z = 0;

    ir_l = 0;

    il_x = 0;
    il_y = 0;
    il_z = 0;

    il_l = 0;

    moving = 0;
}

void LocomotionProcessor::Twist_CB(const twist_msgs::TwistStamped::ConstPtr& twist) {

    x_vel = twist.twist.linear.x;
    theta_vel = twist.twist.angular.z;

    LocomotionProcessor::Command_SR();
    LocomotionProcessor::Command_SL();
    LocomotionProcessor::Command_IR();
    LocomotionProcessor::Command_IL();

}

void LocomotionProcessor::Command_SR() {
    
    sr_msg.point.x = sr_x - center_to_front;
    sr_msg.point.y = sr_y + body_width / 2.0;
    sr_msg.point.z = sr_z;

    sr_msg.header.stamp = ros::Time::now();

    sr_l = sqrt(sr_x * sr_x + sr_y * sr_y);

    sr_pub.publish(sr_msg);
    
}

void LocomotionProcessor::Command_SL() {

    sl_msg.point.x = sl_x - center_to_front;
    sl_msg.point.y = sl_y - body_width / 2.0;
    sl_msg.point.z = sl_z;

    sl_msg.header.stamp = ros::Time::now();

    sl_l = sqrt(sl_x * sl_x + sl_y * sl_y);

    sl_pub.publish(sl_msg);
    
}

void LocomotionProcessor::Command_IR() {
    
    ir_msg.point.x = ir_x + center_to_back;
    ir_msg.point.y = ir_y + body_width / 2.0;
    ir_msg.point.z = ir_z;

    ir_msg.header.stamp = ros::Time::now();

    ir_l = sqrt(ir_x * ir_x + ir_y * ir_y);

    ir_pub.publish(ir_msg);
    
}

void LocomotionProcessor::Command_IL() {
    
    il_msg.point.x = il_x + center_to_back;
    il_msg.point.y = il_y - body_width / 2.0;
    il_msg.point.z = il_z;

    il_msg.header.stamp = ros::Time::now();

    il_l = sqrt(il_x * il_x + il_y * il_y);

    il_pub.publish(il_msg);
    
}

void LocomotionProcessor::Init() {

    sr_x = center_to_front;
    sr_y = -body_width / 2.0;
    sr_z = -walking_z;

    sl_x = center_to_front;
    sl_y = body_width / 2.0;
    sl_z = -walking_z;

    ir_x = -center_to_back;
    ir_y = -body_width / 2.0;
    ir_z = -walking_z;

    il_x = -center_to_back;
    il_y = body_width / 2.0;
    il_z = -walking_z;
    
    LocomotionProcessor::Command_SR();
    LocomotionProcessor::Command_SL();
    LocomotionProcessor::Command_IR();
    LocomotionProcessor::Command_IL();

}

void LocomotionProcessor::Pos_Update(const ros::TimerEvent& event) {
    double dx = x_vel / operating_freq;
    double dtheta = theta_vel / operating_freq;

    double sr_x_new;
    double sr_y_new;
    double sr_z_new;

    double sl_x_new;
    double sl_y_new;
    double sl_z_new;

    double ir_x_new;
    double ir_y_new;
    double ir_z_new;
    
    double il_x_new;
    double il_y_new;
    double il_z_new;

    sr_x_new = (sr_l * sr_y * dtheta - dx);
    sl_x_new = (sl_l * sl_y * dtheta - dx);
    ir_x_new = (ir_l * ir_y * dtheta - dx);
    il_x_new = (il_l * il_y * dtheta - dx);

    sr_y_new = (-sr_l * sr_x * dtheta - dy);
    sl_y_new = (-sl_l * sl_x * dtheta - dy);
    ir_y_new = (-ir_l * ir_x * dtheta - dy);
    il_y_new = (-il_l * il_x * dtheta - dy);

    sr_x += sr_x_new;
    sl_x += sl_x_new;
    ir_x += ir_x_new;
    il_x += il_x_new;

    sr_y += sr_y_new;
    sl_y += sl_y_new;
    ir_y += ir_y_new;
    il_y += il_y_new;
}

