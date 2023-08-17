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
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "../include/gait_planner.h"

// Functionality ######


// Assumptions #########


// IO ########


GaitPlanner::GaitPlanner(const ros::NodeHandle &nh_private_) {
    
    // Subscribers
    vel_sub = nh_.subscribe<geometry_msgs::TwistStamped>("/command/velocity", 1000, &GaitPlanner::Vel_CB, this);
    reset_sub = nh_.subscribe<std_msgs::Bool>("/reset/gait", 1000, &GaitPlanner::Reset_CB, this);
    crouch_sub = nh_.subscribe<std_msgs::Bool>("/crouch", 10, &GaitPlanner::crouch_CB, this);
    sit_sub = nh_.subscribe<std_msgs::Bool>("/sit", 10, &GaitPlanner::sit_CB, this);
    lay_down_sub = nh_.subscribe<std_msgs::Bool>("/layDown", 10, &GaitPlanner::lay_down_CB, this);

    // Publishers
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

    Init();
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

void GaitPlanner::Init() {
    current_gait = zeroGait();
    next_gait = current_gait;
    pose_current = zeroPose();
    pose_command = pose_current;
    gaits.push(zeroGait());
    InitializeGaits();
}

void GaitPlanner::Vel_CB(const geometry_msgs::TwistStamped::ConstPtr& twist) { 

    x_vel = twist->twist.linear.x;
    theta_vel = twist->twist.angular.z;

}

void GaitPlanner::Reset_CB(const std_msgs::Bool::ConstPtr& reset) {
    if(reset)
    {
        testing_leg_position = false;
        mode = still;
        step = Halt;
        x_vel = 0;
        theta_vel = 0;
        delta_percent = 0;
        Init();
        delta_dist = 0;
        delta_theta = 0;
        angle = 0;
        gait_pub.publish(next_gait);
    }
}

void GaitPlanner::Frame_CB(const ros::TimerEvent& event) {
    #if DEBUG_GAIT_EXECUTOR
    debug((std::string)__func__+" Executing...");
    #endif
    debug(std::to_string(delta_percent));

    // Increment percent step according to velocity & operating frequency
    percent_step += delta_percent;
    if (percent_step >= margin) { 
        percent_step = 0;
        previous_gait = next_gait;
        next_gait = gaits.front();
        gaits.pop();
        gaits.push(previous_gait);
    }

    switch(mode){
        case still:         // Retain current position
            current_gait = zeroGait();
            break;
        case walking:       // Cycle four gaits
            current_gait = gait_lerp(previous_gait, next_gait, percent_step);
            current_gait = gait_raise_foot(current_gait);
            break;
        case crouching:     // Cycle two gaits
            current_gait = gait_lerp(previous_gait, next_gait, percent_step);
            break;
        case sitting:      // Lower back legs and cycle that position
            current_gait = gait_lerp(previous_gait, next_gait, percent_step);
            break;
        case laying_down:   // Don't command the motors
            current_gait = gait_lerp(previous_gait, next_gait, percent_step);
            break;
        case recovering:    // Do something with imu/stability data
            break;
        case manual:        // Manual leg control only update off of current leg positions
            break;
        default:
            break;
    }

    gait_pub.publish(current_gait);
}

void GaitPlanner::crouch_CB(const std_msgs::Bool::ConstPtr &crouch){
    #if DEBUG_GAIT_EXECUTOR
    debug((std::string)__func__+" Executing...");
    #endif
    // Clear gait queue and fill with two gait instances that cycle back and forth
    if(!crouch->data)
        return;
    mode = crouching;
    delta_percent = 0.020;
    percent_step = 0;
    while(!gaits.empty()) gaits.pop();

    Gait low;
    low.sr.x = -0.05;
    low.sl.x = -0.05;
    low.ir.x = -0.05;
    low.il.x = -0.05;

    low.sr.y = -0.05;
    low.sl.y = 0.05;
    low.ir.y = -0.05;
    low.il.y = 0.05;

    low.sr.z = -0.12;
    low.sl.z = -0.12;
    low.ir.z = -0.12;
    low.il.z = -0.12;

    Gait high = low;

    high.sr.z = -0.14;
    high.sl.z = -0.14;
    high.ir.z = -0.14;
    high.il.z = -0.14;

    previous_gait = current_gait;
    gaits.push(high);
    gaits.push(low);
}

void GaitPlanner::sit_CB(const std_msgs::Bool::ConstPtr &sit){
    #if DEBUG_GAIT_EXECUTOR
    debug((std::string)__func__+" Executing...");
    #endif
    // Transition from current position to back legs down
    if(!sit->data)
        return;
    mode = sitting;
    delta_percent = 0.015;
    percent_step = 0;
    while(!gaits.empty()) gaits.pop();
    // rostopic pub /manual_position std_msgs/String "0 -0.15 -0.055 -0.15"

    Gait sit_gait;
    sit_gait.sr.x = -0.15;
    sit_gait.sl.x = -0.15;
    sit_gait.ir.x = -0.0;
    sit_gait.il.x = -0.0;

    sit_gait.sr.y = -0.05;
    sit_gait.sl.y = 0.05;
    sit_gait.ir.y = -0.05;
    sit_gait.il.y = 0.05;

    sit_gait.sr.z = -0.15;
    sit_gait.sl.z = -0.15;
    sit_gait.ir.z = -0.09;
    sit_gait.il.z = -0.09;
    previous_gait = current_gait;
    gaits.push(sit_gait);
    
}
void GaitPlanner::lay_down_CB(const std_msgs::Bool::ConstPtr &lay_down){
    #if DEBUG_GAIT_EXECUTOR
    debug((std::string)__func__+" Executing...");
    #endif
    // Move all legs to x,y origin and reduce z until at the ground
    if(!lay_down->data)
        return;
    mode = laying_down;
    delta_percent = 0.015;
    percent_step = 0;
    while(!gaits.empty()) gaits.pop();
    Gait lay_down_gait;
    //rostopic pub /manual_position std_msgs/String "1 0.045 0.055 -0.06"

    lay_down_gait.sr.z = -0.06;
    lay_down_gait.sl.z = -0.06;
    lay_down_gait.ir.z = -0.06;
    lay_down_gait.il.z = -0.06;

    lay_down_gait.sr.x = 0.045;
    lay_down_gait.sl.x = 0.045;
    lay_down_gait.ir.x = 0.045;
    lay_down_gait.il.x = 0.045;

    lay_down_gait.sr.y = -0.055;
    lay_down_gait.sl.y = 0.055;
    lay_down_gait.ir.y = -0.055;
    lay_down_gait.il.y = 0.055;

    previous_gait = current_gait;
    gaits.push(lay_down_gait);
}

Gait gait_lerp(Gait g1, Gait g2, double percent) {
    #if DEBUG_GAIT_EXECUTOR
    debug((std::string)__func__+" Executing...");
    #endif
    Gait out;

    out.sr = point_lerp(g1.sr, g2.sr, percent);
    out.sl = point_lerp(g1.sl, g2.sl, percent);
    out.ir = point_lerp(g1.ir, g2.ir, percent);
    out.il = point_lerp(g1.il, g2.il, percent);
    out.com.position = point_lerp(g1.com.position, g2.com.position, percent);

    tf2::Quaternion quat_tf1, quat_tf2;
    tf2::convert(g1.com.orientation, quat_tf1);
    tf2::convert(g2.com.orientation, quat_tf2);
    out.com.orientation = tf2::toMsg(quat_tf1.tf2::Quaternion::slerp(quat_tf2, percent)); // = point_lerp(g1.com.position, g2.com.position, percent);
    
    out.foot = g1.foot;

    return out;
}

Gait GaitPlanner::gait_raise_foot(Gait gait) {
    #if DEBUG_GAIT_EXECUTOR
    debug((std::string)__func__+" Executing...");
    #endif
    switch (gait.foot.data) {
        case 1: {
            double desired_z = -step_height*4*(percent_step)*(percent_step - 1);
            gait.sr.z += desired_z;
            break; 
        }
        case 2: {
            double desired_z = -step_height*4*(percent_step)*(percent_step - 1);
            gait.sl.z += desired_z;
            break; 
        }
        case 3: {
            double desired_z = -step_height*4*(percent_step)*(percent_step - 1);
            gait.ir.z += desired_z;
            break; 
        }
        case 4: {
            double desired_z = -step_height*4*(percent_step)*(percent_step - 1);
            gait.il.z += desired_z;
            break; 
        }
        default: {
            break; 
        }
    }
    return gait;
}

std::vector<Gait> GaitPlanner::walk(double dist) {
    std::vector<Gait> gait_vector;
    double steps_to_take = dist / ((center_to_back + center_to_front) / 4);
    int steps_taken = 0;
    
    while (steps_taken < steps_to_take) {
        switch(step) {
            case WalkMode::Halt: {
                gait_vector.push_back(il_fwd);
                step = WalkMode::SR; //Next Step
            }
            break;

            case WalkMode::SR: {
                gait_vector.push_back(sr_fwd);
                step = WalkMode::IL;
            }
            break;

            case WalkMode::SL: {
                gait_vector.push_back(sl_fwd);
                step = WalkMode::IR;
            }
            break;

            case WalkMode::IR: {
                gait_vector.push_back(ir_fwd);
                step = WalkMode::SR;
            }
            break;

            case WalkMode::IL: {
                gait_vector.push_back(il_fwd);
                step = WalkMode::SL;
            }
            break;

            default:
            break;
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
            switch(step) {
                case WalkMode::Halt: {
                    gait_vector.push_back(il_turn);
                    step = WalkMode::IL;
                }
                break;

                case WalkMode::SR: {
                    gait_vector.push_back(sr_turn);
                    step = WalkMode::SL;
                }
                break;

                case WalkMode::SL: {
                    gait_vector.push_back(sl_turn);
                    step = WalkMode::IR;
                }
                break;

                case WalkMode::IR: {
                    gait_vector.push_back(ir_turn);
                    step = WalkMode::IL;
                }
                break;

                case WalkMode::IL: {
                    gait_vector.push_back(il_turn);
                    step = WalkMode::SR;
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
            switch(step) {
                case WalkMode::Halt: {
                    gait_vector.push_back(il_turn);
                    step = WalkMode::IL;
                }
                break;

                case WalkMode::SR: {
                    gait_vector.push_back(sr_turn);
                    step = WalkMode::IL;
                }
                break;

                case WalkMode::SL: {
                    gait_vector.push_back(sl_turn);
                    step = WalkMode::SR;
                }
                break;

                case WalkMode::IR: {
                    gait_vector.push_back(ir_turn);
                    step = WalkMode::SL;
                }
                break;

                case WalkMode::IL: {
                    gait_vector.push_back(il_turn);
                    step = WalkMode::IR;
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
