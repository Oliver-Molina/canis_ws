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
    
    //gait_sub = nh_.subscribe<robot_core::Gait>("/odometry/gait/current", 1000, &GaitPlanner::Gait_CB, this);
    //vel_sub = nh_.subscribe<geometry_msgs::TwistStamped>("/command/velocity", 1000, &GaitPlanner::Vel_CB, this);
    //reset_sub = nh_.subscribe<std_msgs::Bool>("/reset/gait", 1000, &GaitPlanner::Reset_CB, this);
    
    
    percent_sub = nh_.subscribe<std_msgs::Float64>("/odometry/percent", 1000, &GaitPlanner::Percent_CB, this);
    path_sub = nh_.subscribe<robot_core::PathQuat>("/command/path", 10, &GaitPlanner::Path_CB, this);
    reset_sub = nh_.subscribe<std_msgs::Bool>("/reset/gait", 1000, &GaitPlanner::Reset_CB, this);

    gait_pub = nh_.advertise<robot_core::Gait>("/command/gait/next", 1000);
    debug_pub = nh_.advertise<std_msgs::String>("/debug", 1000);
    test_leg_position_pub = nh_.advertise<std_msgs::String>("/test_leg_position", 1000);

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
    nh_.param<double>("/leg_x_offset", leg_x_offset, 0.00);
    nh_.param<double>("/leg_x_separation", leg_x_separation, 0.00);

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
    mode = WalkMode::Halt;
    path = {};
    step_turn_rad = 2 * M_PI / 48;

    // #### Testing ####
    delta_angle = 0.05;
    angle = 0.0;
    radius = 0.05;
    period = 2 * M_PI * radius / x_vel;
    rot_freq = 1 / period;
}


// Standard Gaits: SR, SL, IR, IL, Halt
void GaitPlanner::InitializeGaits() {
    double half_width = body_width / 2 + shoulder_length;
    double eighth_len = (center_to_front + center_to_back) / 8;

    
    

    // SR Gait Forward
    sr_fwd.com.orientation.x = 0;
    sr_fwd.com.orientation.y = 0;
    sr_fwd.com.orientation.z = 0;
    sr_fwd.com.orientation.w = 1;
    sr_fwd.com.position.x = 0;
    sr_fwd.com.position.y = 0;
    sr_fwd.com.position.z = walking_z;
    sr_fwd.sr.x = eighth_len+leg_x_offset;
    sr_fwd.sr.y = -half_width;
    sr_fwd.sr.z = 0;
    sr_fwd.sl.x = 3 * eighth_len+leg_x_offset;
    sr_fwd.sl.y = half_width;
    sr_fwd.sl.z = 0;
    sr_fwd.ir.x = -eighth_len+leg_x_separation+leg_x_offset;
    sr_fwd.ir.y = -half_width;
    sr_fwd.ir.z = 0;
    sr_fwd.il.x = -3 * eighth_len+leg_x_separation+leg_x_offset;
    sr_fwd.il.y = half_width;
    sr_fwd.il.z = 0;
    sr_fwd.foot.data = 1;

    // SL Gait Forward
    sl_fwd.com.orientation.x = 0;
    sl_fwd.com.orientation.y = 0;
    sl_fwd.com.orientation.z = 0;
    sl_fwd.com.orientation.w = 1;
    sl_fwd.com.position.x = 0;
    sl_fwd.com.position.y = 0;
    sl_fwd.com.position.z = walking_z;
    sl_fwd.sr.x = 3 * eighth_len+leg_x_offset;
    sl_fwd.sr.y = -half_width;
    sl_fwd.sr.z = 0;
    sl_fwd.sl.x = eighth_len+leg_x_offset;
    sl_fwd.sl.y = half_width;
    sl_fwd.sl.z = 0;
    sl_fwd.ir.x = -3 * eighth_len+leg_x_separation+leg_x_offset;
    sl_fwd.ir.y = -half_width;
    sl_fwd.ir.z = 0;
    sl_fwd.il.x = -eighth_len+leg_x_separation+leg_x_offset;
    sl_fwd.il.y = half_width;
    sl_fwd.il.z = 0;
    sl_fwd.foot.data = 2;

    // IR Gait Forward
    ir_fwd.com.orientation.x = 0;
    ir_fwd.com.orientation.y = 0;
    ir_fwd.com.orientation.z = 0;
    ir_fwd.com.orientation.w = 1;
    ir_fwd.com.position.x = 0;
    ir_fwd.com.position.y = 0;
    ir_fwd.com.position.z = walking_z;
    ir_fwd.sr.x = 2 * eighth_len+leg_x_offset;
    ir_fwd.sr.y = -half_width;
    ir_fwd.sr.z = 0;
    ir_fwd.sl.x = 4 * eighth_len+leg_x_offset;
    ir_fwd.sl.y = half_width;
    ir_fwd.sl.z = 0;
    ir_fwd.ir.x = -4 * eighth_len+leg_x_separation+leg_x_offset;
    ir_fwd.ir.y = -half_width;
    ir_fwd.ir.z = 0;
    ir_fwd.il.x = -2 * eighth_len+leg_x_separation+leg_x_offset;
    ir_fwd.il.y = half_width;
    ir_fwd.il.z = 0;
    ir_fwd.foot.data = 3;

     // IL Gait Forward
    il_fwd.com.orientation.x = 0;
    il_fwd.com.orientation.y = 0;
    il_fwd.com.orientation.z = 0;
    il_fwd.com.orientation.w = 1;
    il_fwd.com.position.x = 0;
    il_fwd.com.position.y = 0;
    il_fwd.com.position.z = walking_z;
    il_fwd.sr.x = 4 * eighth_len+leg_x_offset;
    il_fwd.sr.y = -half_width;
    il_fwd.sr.z = 0;
    il_fwd.sl.x = 2 * eighth_len+leg_x_offset;
    il_fwd.sl.y = half_width;
    il_fwd.sl.z = 0;
    il_fwd.ir.x = -2 * eighth_len+leg_x_separation+leg_x_offset;
    il_fwd.ir.y = -half_width;
    il_fwd.ir.z = 0;
    il_fwd.il.x = -4 * eighth_len+leg_x_separation+leg_x_offset;
    il_fwd.il.y = half_width;
    il_fwd.il.z = 0;
    il_fwd.foot.data = 4;


    halt = zeroGait();

    // IL Gait Turn
    il_turn = zeroGait();
    il_turn.foot.data = 4;

    // SR Gait Turn
    sr_turn = il_turn;
    sr_turn.sr = rotate2D(step_turn_rad, sr_turn.sr);
    sr_turn.sl = rotate2D(step_turn_rad, sr_turn.sl);
    sr_turn.ir = rotate2D(step_turn_rad, sr_turn.ir);
    sr_turn.il = rotate2D(-4 * step_turn_rad, sr_turn.il);
    sr_turn.foot.data = 1;

    // SL Gait Turn
    sl_turn = sr_turn;
    sl_turn.sr = rotate2D(-4 * step_turn_rad, sl_turn.sr);
    sl_turn.sl = rotate2D(step_turn_rad, sl_turn.sl);
    sl_turn.ir = rotate2D(step_turn_rad, sl_turn.ir);
    sl_turn.il = rotate2D(step_turn_rad, sl_turn.il);
    sr_turn.foot.data = 2;

    // IR Gait Turn
    ir_turn = sl_turn;
    ir_turn.sr = rotate2D(step_turn_rad, ir_turn.sr);
    ir_turn.sl = rotate2D(-4 * step_turn_rad, ir_turn.sl);
    ir_turn.ir = rotate2D(step_turn_rad, ir_turn.ir);
    ir_turn.il = rotate2D(step_turn_rad, ir_turn.il);
    sr_turn.foot.data = 3;
    
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
    std::queue<Gait> empty;
    std::vector<Gait> gait_path = calculatePath();
    for (Gait gait : gait_path) {
        empty.push(gait);
    }
    std::swap(gait_queue, empty);
    gait_pub.publish(gait_queue.front());
    gait_queue.pop();
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
        gait_pub.publish(gait_queue.front());
        gait_queue.pop();
    }
}


void GaitPlanner::Init() {

    gait_current = zeroGait();
    gait_command = gait_current;
    pose_current = zeroPose();
    pose_command = pose_current;
    gait_queue.push(zeroGait());
    InitializeGaits();

}

Gait GaitPlanner::zeroGait() {
    
    Gait out;

    out.sr.x = center_to_front+leg_x_offset;
    out.sr.y = -body_width / 2.0 - shoulder_length;
    out.sr.z = 0;

    out.sl.x = center_to_front+leg_x_offset;
    out.sl.y = body_width / 2.0 + shoulder_length;
    out.sl.z = 0;

    out.ir.x = -center_to_back+leg_x_separation+leg_x_offset;
    out.ir.y = -body_width / 2.0 - shoulder_length;
    out.ir.z = 0;

    out.il.x = -center_to_back+leg_x_separation+leg_x_offset;
    out.il.y = body_width / 2.0 + shoulder_length;
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

std::vector<Gait> GaitPlanner::calculatePath() {
    std::vector<Gait> gait_path;
    for (int path_index = 1; path_index < path.size(); path_index++) {
        std::vector<Gait> temp = pathCommand(path[path_index], path[path_index - 1]);
        gait_path.insert(gait_path.end(), temp.begin(), temp.end());
    }
    
    return gait_path;
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
    std::vector<Gait> empty_vec;
    return empty_vec;
}

nav_msgs::Odometry translate(nav_msgs::Odometry end, nav_msgs::Odometry start) {
    nav_msgs::Odometry translate_end = end;

    double x = translate_end.pose.pose.position.x - start.pose.pose.position.x;
    double y = translate_end.pose.pose.position.y - start.pose.pose.position.y;
    double z = translate_end.pose.pose.position.z - start.pose.pose.position.z;

    tf2::Quaternion quat_i, quat_o;
    tf2::convert(start.pose.pose.orientation, quat_i);
    tf2::convert(end.pose.pose.orientation, quat_o);


    tf2::Matrix3x3 m_i;
    m_i.setRotation(quat_i); 
    tf2::Vector3 out_pos_vec(x, y, z);
    out_pos_vec = m_i.inverse() * out_pos_vec;
    quat_o *= quat_i.inverse();

    translate_end.pose.pose.position.x = out_pos_vec.x();
    translate_end.pose.pose.position.y = out_pos_vec.y();
    translate_end.pose.pose.position.z = out_pos_vec.z();

    //translate_end.pose.pose.orientation = tf::createQuaternionFromMsg(quat_o);
    tf2::convert(quat_o, translate_end.pose.pose.orientation);
    return translate_end;

}

std::vector<Gait> GaitPlanner::walk(double dist) {
    std::vector<Gait> gait_vector;
    double steps_to_take = dist / ((center_to_back + center_to_front) / 4);
    int steps_taken = 0;
    
    while (steps_taken < steps_to_take) {
        switch(mode) {
            case WalkMode::Halt: {
                gait_vector.push_back(il_fwd);
                mode = WalkMode::SR; //Next Step
            }
            break;

            case WalkMode::SR: {
                gait_vector.push_back(sr_fwd);
                mode = WalkMode::IL;
            }
            break;

            case WalkMode::SL: {
                gait_vector.push_back(sl_fwd);
                mode = WalkMode::IR;
            }
            break;

            case WalkMode::IR: {
                gait_vector.push_back(ir_fwd);
                mode = WalkMode::SR;
            }
            break;

            case WalkMode::IL: {
                gait_vector.push_back(il_fwd);
                mode = WalkMode::SL;
            }
            break;

            default: {


    
            }
        }
        steps_taken++;
    }
    return gait_vector;
}

std::vector<Gait> GaitPlanner::turn(double rad) {
    std::vector<Gait> gait_vector;
    double steps_to_take = rad / step_turn_rad;
    int steps_taken = 0;
    if (rad >= 0) {
        while (steps_taken < steps_to_take) {
            switch(mode) {
                case WalkMode::Halt: {
                    gait_vector.push_back(il_turn);
                    mode = WalkMode::IL;
                }
                break;

                case WalkMode::SR: {
                    gait_vector.push_back(sr_turn);
                    mode = WalkMode::SL;
                }
                break;

                case WalkMode::SL: {
                    gait_vector.push_back(sl_turn);
                    mode = WalkMode::IR;
                }
                break;

                case WalkMode::IR: {
                    gait_vector.push_back(ir_turn);
                    mode = WalkMode::IL;
                }
                break;

                case WalkMode::IL: {
                    gait_vector.push_back(il_turn);
                    mode = WalkMode::SR;
                }
                break;

                default: {


        
                }
            }
            steps_taken++;
        }
    }
    else {
        while (steps_taken < steps_to_take) {
            switch(mode) {
                case WalkMode::Halt: {
                    gait_vector.push_back(il_turn);
                    mode = WalkMode::IL;
                }
                break;

                case WalkMode::SR: {
                    gait_vector.push_back(sr_turn);
                    mode = WalkMode::IL;
                }
                break;

                case WalkMode::SL: {
                    gait_vector.push_back(sl_turn);
                    mode = WalkMode::SR;
                }
                break;

                case WalkMode::IR: {
                    gait_vector.push_back(ir_turn);
                    mode = WalkMode::SL;
                }
                break;

                case WalkMode::IL: {
                    gait_vector.push_back(il_turn);
                    mode = WalkMode::IR;
                }
                break;

                default: {


        
                }
            }
            steps_taken++;
        }
    }
    return gait_vector;
}

geometry_msgs::Point rotate2D(double rad, geometry_msgs::Point point) {
    tf2::Quaternion tf2_quat;
    tf2_quat.setRPY(0, 0, rad);

    tf2::Matrix3x3 tf2_mat;
    tf2_mat.setRotation(tf2_quat); 
    tf2::Vector3 tf2_point(point.x, point.y, point.z);
    tf2::Vector3 tf2_point_out = tf2_mat * tf2_point;

    geometry_msgs::Point point_out;
    point_out.x = tf2_point.getX();
    point_out.y = tf2_point.getY();
    point_out.z = tf2_point.getZ();
    return point_out;
}

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


    for (int i = 0; i < values.size() && i < char_count; i++) {
        ss << split_message[i] << values[i];
    }
    ss << split_message[values.size()];

    std::string str = ss.str();
    debug_msg.data = str.c_str();
    debug_pub.publish(debug_msg);
    
}

void GaitPlanner::debug(std::string message) {
    debug_msg.data = message.c_str();
    debug_pub.publish(debug_msg);
}

void GaitPlanner::print_gait(Gait gait) {
    std::vector<double> values_vec;
    values_vec.push_back(gait.com.position.x); 
    values_vec.push_back(gait.com.position.y); 
    values_vec.push_back(gait.com.position.z);

    std::vector<double> values_vec_sr;
    values_vec.push_back(gait.sr.x);
    values_vec.push_back(gait.sr.y);
    values_vec.push_back(gait.sr.z);

    std::vector<double> values_vec_sl;
    values_vec.push_back(gait.sl.x);
    values_vec.push_back(gait.sl.y);
    values_vec.push_back(gait.sl.z);

    std::vector<double> values_vec_ir;
    values_vec.push_back(gait.ir.x);
    values_vec.push_back(gait.ir.y);
    values_vec.push_back(gait.ir.z);

    std::vector<double> values_vec_il;
    values_vec.push_back(gait.il.x);
    values_vec.push_back(gait.il.y);
    values_vec.push_back(gait.il.z);

    debug(values_vec, "COM: x: |, y: |, z: |\nSR: x: |, y: |, z: |\nSL x: |, y: |, z: |\nIR: x: |, y: |, z: | \nIL: x: |, y: |, z: | ");

}
