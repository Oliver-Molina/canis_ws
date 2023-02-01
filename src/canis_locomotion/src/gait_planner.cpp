#include <math.h>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <robot_core/Gait.h>
#include <robot_core/GaitVec.h>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "../include/gait_planner.h"

// Functionality ######


// Assumptions #########


// IO ########
// Subs 


// Pubs 





GaitPlanner::GaitPlanner(const ros::NodeHandle &nh_private_) {
    
    gait_sub = nh_.subscribe<robot_core::Gait>("/odometry/gait/current", 1000, &GaitPlanner::Gait_CB, this);
    vel_sub = nh_.subscribe<geometry_msgs::TwistStamped>("/command/velocity", 1000, &GaitPlanner::Vel_CB, this);
    reset_sub = nh_.subscribe<std_msgs::Bool>("/reset/gait", 1000, &GaitPlanner::Reset_CB, this);
    percent_sub = nh_.subscribe<std_msgs::Float64>("/odometry/percent", 1000, &GaitPlanner::Percent_CB, this);

    gait_pub = nh_.advertise<robot_core::Gait>("/command/gait/next", 1000);

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
    nh_.param<double>("/step_height", step_height, 0.05);

    // #### Robot Gait Variables ####
    gait_current;
    gait_command;
    x_vel = 0;
    theta_vel = 0;
    delta_dist = 0;
    delta_theta = 0;
    margin = 1.0 - 0.0005;
    on = false;
    percent = 0;
    mode = Mode::Halted;

    // #### Testing ####
    delta_angle = 0.05;
    angle = 0.0;
    radius = 0.05;
    period = 2 * M_PI * radius / x_vel;
    rot_freq = 1 / period;
    vel_pub = nh_.advertise<geometry_msgs::TwistStamped>("/command/velocity", 1000);
}

void GaitPlanner::Gait_CB(const robot_core::Gait::ConstPtr& gait) { 
    gait_current = *gait;
}

void GaitPlanner::Vel_CB(const geometry_msgs::TwistStamped::ConstPtr& twist) { 

    x_vel = twist->twist.linear.x;
    theta_vel = twist->twist.angular.z;

    // Notes: When Halting, no state changes allowed till squared off

    if (x_vel == 0 & theta_vel == 0) {
        // FWD  -> Halt
        // Turn -> Halt
        // Halt -> No Change
    }
    if (x_vel == 0 & theta_vel != 0) {
        // FWD  -> Halt -> Turn
        // Turn -> No Change
        // Halt -> Turn
    }
    if (x_vel != 0 & theta_vel == 0) {
        // FWD  -> No Change
        // Turn -> Halt -> FWD
        // Halt -> FWD
    }
    if (x_vel != 0 & theta_vel != 0) {
        // FWD  -> Halt
        // Turn -> Halt
        // Halt -> No Change
    }

}

void GaitPlanner::Reset_CB(const std_msgs::Bool::ConstPtr& reset) {


    Init();
    delta_dist = 0;
    delta_theta = 0;
    angle = 0;

    on = reset->data;
    gait_pub.publish(gait_command);

}

void GaitPlanner::Percent_CB(const std_msgs::Float64::ConstPtr& percent_msg) {
    percent = percent_msg->data;
}


void GaitPlanner::Init() {

    gait_current.sr.x = center_to_front;
    gait_current.sr.y = -body_width / 2.0;
    gait_current.sr.z = 0;

    gait_current.sl.x = center_to_front;
    gait_current.sl.y = body_width / 2.0;
    gait_current.sl.z = 0;

    gait_current.ir.x = -center_to_back;
    gait_current.ir.y = -body_width / 2.0;
    gait_current.ir.z = 0;

    gait_current.il.x = -center_to_back;
    gait_current.il.y = body_width / 2.0;
    gait_current.il.z = 0;

    gait_current.com.position.x = 0;
    gait_current.com.position.y = 0;
    gait_current.com.position.z = walking_z;

    gait_current.com.orientation.x = 0;
    gait_current.com.orientation.y = 0;
    gait_current.com.orientation.z = 0;
    gait_current.com.orientation.w = 1;

    gait_command = gait_current;

}

void GaitPlanner::Pose_Update(const ros::TimerEvent& event) {
    if (percent >= margin) {
        /*gait_command.com.position.x = radius * cos(angle);
        gait_command.com.position.y = radius * sin(angle);

        gait_pub.publish(gait_command);
        angle += delta_angle;
        if (angle > 2 * M_PI) angle -= 2 * M_PI;*/

        // 
    }
    
    // #### Testing #### !Remove before merge
    //debug({angle}, "Ang: | ");
    TwistStamped twist_msg;
    twist_msg.twist.linear.x = 0.01;
    twist_msg.twist.angular.z = 0.01;
    vel_pub.publish(twist_msg);
}

void GaitPlanner::debug(std::vector<double> values, std::string message) {
    // Requires:
    //  message have as many | as values in vector
    //  not end on a | character
    std::vector<std::string> split_message;
    std::stringstream stream(message);
    std::string segment;
    while(std::getline(stream, segment, '|')) {
        split_message.push_back(segment);
    }
    
    std::stringstream ss;
    for (int i = 0; i < values.size(); i++) {
        ss << split_message[i] << values[i];
    }
    ss << split_message[values.size()];

    std::string str = ss.str();
    debug_msg.data = str.c_str();
    debug_pub.publish(debug_msg);
}

