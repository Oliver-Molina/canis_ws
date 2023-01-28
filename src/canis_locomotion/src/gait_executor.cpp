#include <math.h>
#include <iomanip>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <robot_core/Gait.h>
#include <robot_core/GaitVec.h>
#include <tf/tf.h>

#include "../include/gait_executor.h"

// Functionality ######
// Takes a vector of Gait positions and moves through them at a set speed
// Stops Safely at the end
// Controls COM for balance

// Assumptions #########
// Each Vec is indeed a valid 

// IO ########
// Subs 
// gaitVec: Vector<Point[4]>

// Pubs 
// sr leg: Point
// sl leg: Point
// ir leg: Point
// il leg: Point




GaitExecutor::GaitExecutor(const ros::NodeHandle &nh_private_) {
    
    gait_sub = nh_.subscribe<robot_core::GaitVec>("/path/gait", 1000, &GaitExecutor::Gait_CB, this);
    vel_sub = nh_.subscribe<std_msgs::Float64>("/command/velocity", 1000, &GaitExecutor::Vel_CB, this);
    reset_sub = nh_.subscribe<std_msgs::Bool>("/reset/gait", 1000, &GaitExecutor::Reset_CB, this);

    sr_pub = nh_.advertise<geometry_msgs::PointStamped>("/command/pos/superior/right", 1000);
    sl_pub = nh_.advertise<geometry_msgs::PointStamped>("/command/pos/superior/left", 1000);
    ir_pub = nh_.advertise<geometry_msgs::PointStamped>("/command/pos/inferior/right", 1000);
    il_pub = nh_.advertise<geometry_msgs::PointStamped>("/command/pos/inferior/left", 1000);

    debug_pub = nh_.advertise<std_msgs::String>("/debug", 1000);

    // #### Robot Params ####
    nh_.param<double>("/shoulder_length", shoulder_length, 0.055);
    nh_.param<double>("/arm_length", arm_length, 0.105);
    nh_.param<double>("/forearm_length", forearm_length, 0.136);
    nh_.param<double>("/body_width", body_width, 0.038);
    nh_.param<double>("/center_to_front", center_to_front, 0.1);
    nh_.param<double>("/center_to_back", center_to_back, 0.1);

    nh_.param<double>("/frequency", operating_freq, 30);
    nh_.param<double>("/walking_height", walking_z, 0.15);
    //nh_.param<double>("/triangle_boundary", safe_bound_triangle, 0.03);
    //nh_.param<double>("/relative_step_rate", relative_step_rate, 10.0);
    nh_.param<double>("/step_height", step_height, 0.05);

    // #### Robot Variables ####
    gait_vec = {};
    walking_vel = 0;
    gait_current;
    gait_next;
    percent_step = 0;

    // #### Leg Positions & Variables ####
    sr_x = 0; // Superior Right X (Relative to Center of Mass)
    sr_y = 0; // Superior Right Y (Relative to Center of Mass)
    sr_z = 0; // Superior Right Z (Relative to Center of Mass)

    sl_x = 0;
    sl_y = 0;
    sl_z = 0;

    ir_x = 0;
    ir_y = 0;
    ir_z = 0;

    il_x = 0;
    il_y = 0;
    il_z = 0;
}

void GaitExecutor::Gait_CB(const robot_core::GaitVec::ConstPtr& gait) { // Should actually take a gait vec with some way of *Tweaking* the path

    //x_vel = twist->twist.linear.x;
    //theta_vel = twist->twist.angular.z;

}

void GaitExecutor::Vel_CB(const std_msgs::Float64::ConstPtr& vel) { // Should actually take a gait vec with some way of *Tweaking* the path

    //x_vel = twist->twist.linear.x;
    //theta_vel = twist->twist.angular.z;

}

void GaitExecutor::Reset_CB(const std_msgs::Bool::ConstPtr& reset) { // Should actually take a gait vec with some way of *Tweaking* the path

    //x_vel = twist->twist.linear.x;
    //theta_vel = twist->twist.angular.z;

}

void GaitExecutor::Command_SR() {
    
    sr_msg.point.x = sr_x - center_to_front;
    sr_msg.point.y = sr_y + body_width / 2.0;
    sr_msg.point.z = sr_z;

    sr_msg.header.stamp = ros::Time::now();
    sr_pub.publish(sr_msg);
    
}
void GaitExecutor::Command_SL() {

    sl_msg.point.x = sl_x - center_to_front;
    sl_msg.point.y = sl_y - body_width / 2.0;
    sl_msg.point.z = sl_z;

    sl_msg.header.stamp = ros::Time::now();
    sl_pub.publish(sl_msg);
    
}
void GaitExecutor::Command_IR() {
    
    ir_msg.point.x = ir_x + center_to_back;
    ir_msg.point.y = ir_y + body_width / 2.0;
    ir_msg.point.z = ir_z;

    ir_msg.header.stamp = ros::Time::now();
    ir_pub.publish(ir_msg);
    
}
void GaitExecutor::Command_IL() {
    
    il_msg.point.x = il_x + center_to_back;
    il_msg.point.y = il_y - body_width / 2.0;
    il_msg.point.z = il_z;

    il_msg.header.stamp = ros::Time::now();
    il_pub.publish(il_msg);
    
}


void GaitExecutor::Move_Body(double x, double y) {
    sr_x = center_to_front - x;
    sr_y = -body_width / 2.0 - y;
    sr_z = -walking_z;

    sl_x = center_to_front - x;
    sl_y = body_width / 2.0 - y;
    sl_z = -walking_z;

    ir_x = -center_to_back - x;
    ir_y = -body_width / 2.0 - y;
    ir_z = -walking_z;

    il_x = -center_to_back - x;
    il_y = body_width / 2.0 - y;
    il_z = -walking_z;

    GaitExecutor::Command_SR();
    GaitExecutor::Command_SL();
    GaitExecutor::Command_IR();
    GaitExecutor::Command_IL();
    
}

Gait GaitExecutor::Gait_Lerp(Gait gait_i, Gait gait_f, double percent) {

}

void GaitExecutor::Vel_Update(const ros::TimerEvent& event) {

}


