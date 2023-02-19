#include <math.h>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <robot_core/Gait.h>
#include <robot_core/GaitVec.h>
#include <robot_core/PathQuat.h>
//#include <robot_core/PathQuat.h>
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
    mode = Mode::Halt;
    path = {};
    gait_queue = {};

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
    pose_current = gait_current.com;
}

void GaitPlanner::Vel_CB(const geometry_msgs::TwistStamped::ConstPtr& twist) { 

    x_vel = twist->twist.linear.x;
    theta_vel = twist->twist.angular.z;

}

void GaitPlanner::Path_CB(const PathQuat::ConstPtr& path) {
    this->path = (*path).poses;
    gait_queue.clear();
    calculatePath();
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
    if (percent > margin && !gait_queue.empty()) {
        gait_pub.publish(gait_queue[0]);
        gait_queue.erase(gait_queue.begin());
    }
}


void GaitPlanner::Init() {

    gait_current = zeroGait();
    gait_command = gait_current;
    pose_current = zeroPose();
    pose_command = pose_current;

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
}

std::vector<Gait> GaitPlanner::turn(double turn_rad) {

}
std::vector<Gait> GaitPlanner::halt() { // Bad practice fix later, nick is lazy
    std::vector<Gait> out;
    out.push_back(zeroGait());
    return out;
}
std::vector<Gait> GaitPlanner::walk(double dist) {
    
}
geometry_msgs::Pose GaitPlanner::safePose(double dist) {
    
}
Gait GaitPlanner::zeroGait() {
    
    Gait out;

    out.sr.x = center_to_front;
    out.sr.y = -body_width / 2.0;
    out.sr.z = 0;

    out.sl.x = center_to_front;
    out.sl.y = body_width / 2.0;
    out.sl.z = 0;

    out.ir.x = -center_to_back;
    out.ir.y = -body_width / 2.0;
    out.ir.z = 0;

    out.il.x = -center_to_back;
    out.il.y = body_width / 2.0;
    out.il.z = 0;

    out.com.position.x = 0;
    out.com.position.y = 0;
    out.com.position.z = walking_z;

    out.com.orientation.x = 0;
    out.com.orientation.y = 0;
    out.com.orientation.z = 0;
    out.com.orientation.w = 1;

    return out;
}
geometry_msgs::Pose GaitPlanner::zeroPose() {
    geometry_msgs::Pose out;

    out.position.x = 0;
    out.position.y = 0;
    out.position.z = walking_z;

    out.orientation.x = 0;
    out.orientation.y = 0;
    out.orientation.z = 0;
    out.orientation.w = 1;

    return out;
}
Gait GaitPlanner::transformGait(Gait gait, Pose transform) {

}
Gait GaitPlanner::normalize_gait(Gait gait) {
    Gait out = gait;

    // Shift to origin first
    out.sr.x -= gait.com.position.x;
    out.sr.y -= gait.com.position.y;
    out.sr.z -= gait.com.position.z;

    out.sl.x -= gait.com.position.x;
    out.sl.y -= gait.com.position.y;
    out.sl.z -= gait.com.position.z;

    out.ir.x -= gait.com.position.x;
    out.ir.y -= gait.com.position.y;
    out.ir.z -= gait.com.position.z;

    out.il.x -= gait.com.position.x;
    out.il.y -= gait.com.position.y;
    out.il.z -= gait.com.position.z;

    out.com.position.x = 0;
    out.com.position.y = 0;
    out.com.position.z = 0;

    // Rotate Each Foot  
    tf::Quaternion q(
        gait.com.orientation.x,
        gait.com.orientation.y,
        gait.com.orientation.z,
        gait.com.orientation.w);
    tf::Matrix3x3 m(q);

    tf::Vector3 sr_vec(out.sr.x, out.sr.y, out.sr.z);
    tf::Vector3 sl_vec(out.sl.x, out.sl.y, out.sl.z);
    tf::Vector3 ir_vec(out.ir.x, out.ir.y, out.ir.z);
    tf::Vector3 il_vec(out.il.x, out.il.y, out.il.z);
    
    tf::Vector3 sr_norm, sl_norm, ir_norm, il_norm;

    sr_norm = m.inverse()*sr_vec;
    sl_norm = m.inverse()*sl_vec;
    ir_norm = m.inverse()*ir_vec;
    il_norm = m.inverse()*il_vec;

    out.sr.x = sr_norm.getX();
    out.sr.y = sr_norm.getY();
    out.sr.z = sr_norm.getZ();

    out.sl.x = sl_norm.getX();
    out.sl.y = sl_norm.getY();
    out.sl.z = sl_norm.getZ();

    out.ir.x = ir_norm.getX();
    out.ir.y = ir_norm.getY();
    out.ir.z = ir_norm.getZ();

    out.il.x = il_norm.getX();
    out.il.y = il_norm.getY();
    out.il.z = il_norm.getZ();

    out.com.orientation.x = 0;
    out.com.orientation.y = 0;
    out.com.orientation.z = 0;
    out.com.orientation.w = 1;

    return out;
}

void GaitPlanner::calculatePath() {
    for (int path_index = 0; path_index < path.size(); path_index++) {
        auto vec = pathCommand(path[path_index], path[path_index - 1]);
        gait_queue.insert( gait_queue.end(), vec.begin(), vec.end() );
    }
}

std::vector<Gait> GaitPlanner::pathCommand(nav_msgs::Odometry end, nav_msgs::Odometry start) {
    auto translated_end = translate(end, start);
    auto dist = translated_end.pose.pose.position.x;
    auto rad = translated_end.twist.twist.angular.z;
    if (dist == 0 && rad != 0) {
        return turn(rad);
    }
    else if (dist != 0 && rad == 0) {
        return walk(dist);
    }
}

nav_msgs::Odometry translate(nav_msgs::Odometry end, nav_msgs::Odometry start) {

}
/*
void GaitPlanner::debug(std::vector<double> values, std::string message) {
    // Requires:
    //  message have as many | as values in vector
    //  not end on a | character
    std::vector<std::string> split_message;
    std::stringstream stream(message);
    std::string segment;
    int char_count = 0;
    while(std::getline(stream, segment, '|')) {
        split_message.push_back(segment);
        char_count++;
    }
    
    std::stringstream ss;
    if (values.size() != char_count) {
        ss << "[Debug Error] Message: (";
        for (int i = 0; i < values.size(); i++) {
            ss << split_message[i] << "|";
        }
        ss << " & Values: ";
        for (int i = 0; i < values.size(); i++) {
            ss << values[i] << " ";
        }
        ss << "don't match." << std::endl;

        std::string str = ss.str();
        debug_msg.data = str.c_str();
        debug_pub.publish(debug_msg);
    }
    else {
        for (int i = 0; i < values.size(); i++) {
            ss << split_message[i] << values[i];
        }
        ss << split_message[values.size()];

        std::string str = ss.str();
        debug_msg.data = str.c_str();
        debug_pub.publish(debug_msg);
    }
}
std::vector<Gait> 
*/