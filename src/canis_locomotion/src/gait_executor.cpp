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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
    
    gait_sub = nh_.subscribe<robot_core::Gait>("/path/gait", 1000, &GaitExecutor::Gait_CB, this);
    vel_sub = nh_.subscribe<geometry_msgs::TwistStamped>("/command/velocity", 1000, &GaitExecutor::Vel_CB, this);
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
    nh_.param<double>("/step_height", step_height, 0.05);

    // #### Robot Variables ####
    gait_vec = {};
    gait_current;
    gait_next;
    percent_step = 0.;
    x_vel = 0;
    theta_vel = 0;
    gait_index = 0;



    // #### Leg Positions & Variables ####
    sr.x = 0; // Superior Right X (Relative to Center of Mass)
    sr.y = 0; // Superior Right Y (Relative to Center of Mass)
    sr.z = 0; // Superior Right Z (Relative to Center of Mass)

    sl.x = 0;
    sl.y = 0;
    sl.z = 0;

    ir.x = 0;
    ir.y = 0;
    ir.z = 0;

    il.x = 0;
    il.y = 0;
    il.z = 0;

}

void GaitExecutor::Gait_Replace_CB(const robot_core::GaitVec::ConstPtr& gait) { 
    //gait_vec = gait->gaits;
    //gait_current = gait_vec[0];
    //gait_next = gait_vec[1];
}

void GaitExecutor::Gait_CB(const robot_core::Gait::ConstPtr& gait) { 
    gait_next = *gait;//{{gait->sr.x, gait->sr.y, gait->sr.z}, {gait->sl.x, gait->sl.y, gait->sl.z}, {gait->ir.x, gait->ir.y, gait->ir.z}, {gait->il.x, gait->il.y, gait->il.z}, {gait->com.x, gait->com.y, gait->com.z}, gait->foot};
}

void GaitExecutor::Vel_CB(const geometry_msgs::TwistStamped::ConstPtr& twist) { 

    x_vel = twist->twist.linear.x;
    theta_vel = twist->twist.angular.z;

}

void GaitExecutor::Reset_CB(const std_msgs::Bool::ConstPtr& reset) {

    //x_vel = twist->twist.linear.x;
    //theta_vel = twist->twist.angular.z;

}

void GaitExecutor::Command_SR() {
    
    sr_msg.point.x = sr.x - center_to_front;
    sr_msg.point.y = sr.y + body_width / 2.0;
    sr_msg.point.z = sr.z;

    sr_msg.header.stamp = ros::Time::now();
    sr_pub.publish(sr_msg);
    
}
void GaitExecutor::Command_SL() {

    sl_msg.point.x = sl.x - center_to_front;
    sl_msg.point.y = sl.y - body_width / 2.0;
    sl_msg.point.z = sl.z;

    sl_msg.header.stamp = ros::Time::now();
    sl_pub.publish(sl_msg);
    
}
void GaitExecutor::Command_IR() {
    
    ir_msg.point.x = ir.x + center_to_back;
    ir_msg.point.y = ir.y + body_width / 2.0;
    ir_msg.point.z = ir.z;

    ir_msg.header.stamp = ros::Time::now();
    ir_pub.publish(ir_msg);
    
}
void GaitExecutor::Command_IL() {
    
    il_msg.point.x = il.x + center_to_back;
    il_msg.point.y = il.y - body_width / 2.0;
    il_msg.point.z = il.z;

    il_msg.header.stamp = ros::Time::now();
    il_pub.publish(il_msg);
    
}


void GaitExecutor::Move_Body(double x, double y) {
    sr.x = center_to_front - x;
    sr.y = -body_width / 2.0 - y;
    sr.z = -walking_z;

    sl.x = center_to_front - x;
    sl.y = body_width / 2.0 - y;
    sl.z = -walking_z;

    ir.x = -center_to_back - x;
    ir.y = -body_width / 2.0 - y;
    ir.z = -walking_z;

    il.x = -center_to_back - x;
    il.y = body_width / 2.0 - y;
    il.z = -walking_z;

    GaitExecutor::Command_SR();
    GaitExecutor::Command_SL();
    GaitExecutor::Command_IR();
    GaitExecutor::Command_IL();
    
}

void GaitExecutor::Vel_Update(const ros::TimerEvent& event) {
    double delta_dist = 0; 
    
    if (gait_current.foot.data != 0) {
        delta_dist = gait_next.sr.x - gait_current.sr.x; 
    }

    else {
        delta_dist = gait_next.sl.x - gait_current.sl.x; 
    }

    double dpercent = x_vel / (delta_dist * operating_freq);
 
    if (gait_current == gait_next) { // stop mode

    }

    else if (percent_step >= 1) {   // change gait
        percent_step = 0;
        gait_current = gait_next;
    }

    else {                          // continue gait
        percent_step += dpercent;
        Command_Body();
    }

    // Two cases, middle of step so percent is incremented
    // end of step where next step is taken, legs are stepped up and down
}

void GaitExecutor::Command_Body() {
    Gait store = gait_lerp(gait_current, gait_next, percent_step);

    sr = store.sr;
    sl = store.sl;
    ir = store.ir;
    il = store.il;
}

double double_lerp(double x1, double x2, double percent) {
    return x1 + (x2 - x1) * percent;
}

Point point_lerp(Point p1, Point p2, double percent) {
    Point out;

    out.x = double_lerp(p1.x, p2.x, percent);
    out.y = double_lerp(p1.y, p2.y, percent);
    out.z = double_lerp(p1.z, p2.z, percent);

    return out;
}

Gait gait_lerp(Gait g1, Gait g2, double percent) {
    Gait out;

    out.sr = point_lerp(g1.sr, g2.sr, percent);
    out.sl = point_lerp(g1.sl, g2.sl, percent);
    out.ir = point_lerp(g1.ir, g2.ir, percent);
    out.il = point_lerp(g1.il, g2.il, percent);
    out.com.position = point_lerp(g1.com.position, g2.com.position, percent);

    tf2::Quaternion quat_tf1, quat_tf2;
    geometry_msgs::Quaternion quat_msg;
    tf2::convert(g1.com.orientation, quat_tf1);
    tf2::convert(g2.com.orientation, quat_tf2);
    out.com.orientation = tf2::toMsg(quat_tf1.tf2::Quaternion::slerp(quat_tf2, percent)); // = point_lerp(g1.com.position, g2.com.position, percent);
    
    out.foot = g1.foot;

    return out;
}


