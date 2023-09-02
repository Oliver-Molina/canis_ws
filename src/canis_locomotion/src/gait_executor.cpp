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
#define DEBUG_GAIT_EXECUTOR 1
#define DEBUG_LEG_POSITIONS 0


#define superior_right 0
#define superior_left 1
#define inferior_right 2
#define inferior_left 3

GaitExecutor::GaitExecutor(const ros::NodeHandle &nh_private_) {
    
    raw_gait_sub = nh_.subscribe<robot_core::Gait>("/command/gait/raw", 1000, &GaitExecutor::Raw_Gait_CB, this);
    normalized_gait_sub = nh_.subscribe<robot_core::Gait>("/command/gait/processed", 1000, &GaitExecutor::Processed_Gait_CB, this);

    sr_pub = nh_.advertise<geometry_msgs::PointStamped>("/command/leg/pos/superior/right", 1000);
    sl_pub = nh_.advertise<geometry_msgs::PointStamped>("/command/leg/pos/superior/left", 1000);
    ir_pub = nh_.advertise<geometry_msgs::PointStamped>("/command/leg/pos/inferior/right", 1000);
    il_pub = nh_.advertise<geometry_msgs::PointStamped>("/command/leg/pos/inferior/left", 1000);
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
    nh_.param<double>("/leg_x_offset", leg_x_offset, 0.00);
    nh_.param<double>("/leg_x_separation", leg_x_separation, 0.00);
}

void GaitExecutor::Raw_Gait_CB(const robot_core::Gait::ConstPtr& gait) { 
    #if DEBUG_GAIT_EXECUTOR
    debug((std::string)__func__+" Executing...");
    #endif
    gait_out = *gait;
    normalize_gait();
    recenter_leg_positions();
    Command_Body();
}

void GaitExecutor::Processed_Gait_CB(const robot_core::Gait::ConstPtr& gait){
    #if DEBUG_GAIT_EXECUTOR
    debug((std::string)__func__+" Executing...");
    #endif
    gait_out = *gait;
    Command_Body();
}

void GaitExecutor::Command_Body() {
    GaitExecutor::Command_IR();
    GaitExecutor::Command_IL();
    GaitExecutor::Command_SR();
    GaitExecutor::Command_SL();
}

void GaitExecutor::Command_SR() {
    #if DEBUG_GAIT_EXECUTOR
    debug((std::string)__func__+" Executing...");
    #endif

    sr_msg.point = gait_out.sr;

    #if DEBUG_LEG_POSITIONS
    std::vector<double> values_vec;
    values_vec.push_back(sr_msg.point.x);
    values_vec.push_back(sr_msg.point.y);
    values_vec.push_back(sr_msg.point.z);
    debug(values_vec, (std::string)__func__+" x: |, y: |, z: | ");
    #endif
    sr_msg.header.stamp = ros::Time::now();
    sr_pub.publish(sr_msg);    
}

void GaitExecutor::Command_SL() {
    #if DEBUG_GAIT_EXECUTOR
    debug((std::string)__func__+" Executing...");
    #endif

    sl_msg.point = gait_out.sl;

    #if DEBUG_LEG_POSITIONS
    std::vector<double> values_vec;
    values_vec.push_back(sl_msg.point.x);
    values_vec.push_back(sl_msg.point.y);
    values_vec.push_back(sl_msg.point.z);
    debug(values_vec, (std::string)__func__+" x: |, y: |, z: | ");
    #endif
    sl_msg.header.stamp = ros::Time::now();
    sl_pub.publish(sl_msg);
    
}

void GaitExecutor::Command_IR() {
    #if DEBUG_GAIT_EXECUTOR
    debug((std::string)__func__+" Executing...");
    #endif

    ir_msg.point = gait_out.ir;

    #if DEBUG_LEG_POSITIONS
    std::vector<double> values_vec;
    values_vec.push_back(ir_msg.point.x);
    values_vec.push_back(ir_msg.point.y);
    values_vec.push_back(ir_msg.point.z);
    debug(values_vec, (std::string)__func__+" x: |, y: |, z: | ");
    #endif
    ir_msg.header.stamp = ros::Time::now();
    ir_pub.publish(ir_msg);
    
}

void GaitExecutor::Command_IL() {
    #if DEBUG_GAIT_EXECUTOR
    debug((std::string)__func__+" Executing...");
    #endif

    il_msg.point = gait_out.il;

    #if DEBUG_LEG_POSITIONS
    std::vector<double> values_vec;
    values_vec.push_back(il_msg.point.x);
    values_vec.push_back(il_msg.point.y);
    values_vec.push_back(il_msg.point.z);
    debug(values_vec, (std::string)__func__+" x: |, y: |, z: | ");
    #endif
    il_msg.header.stamp = ros::Time::now();
    il_pub.publish(il_msg);
    
}

void GaitExecutor::debug(std::vector<double> values, std::string message) {
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

void GaitExecutor::debug(std::string message) {
    debug_msg.data = message.c_str();
    debug_pub.publish(debug_msg);
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

Point GaitExecutor::recenter_point(Point point, int leg){
    Point newPoint;
    switch(leg){
        case superior_right:
            newPoint.x = point.x - center_to_front;
            newPoint.y = point.y + body_width / 2.0;
            newPoint.z = point.z;
            break;
        case superior_left:
            newPoint.x = point.x - center_to_front;
            newPoint.y = point.y - body_width / 2.0;
            newPoint.z = point.z;
            break;
        case inferior_right:
            newPoint.x = point.x + center_to_front;
            newPoint.y = point.y + body_width / 2.0;
            newPoint.z = point.z;
            break;
        case inferior_left:
            newPoint.x = point.x + center_to_front;
            newPoint.y = point.y - body_width / 2.0;
            newPoint.z = point.z;
            break;
        default:
            break;
    }
    return newPoint;
}

void GaitExecutor::recenter_leg_positions() {
    gait_out.sr = recenter_point(gait_out.sr, superior_right);
    gait_out.sl = recenter_point(gait_out.sl, superior_left);
    gait_out.ir = recenter_point(gait_out.ir, inferior_right);
    gait_out.il = recenter_point(gait_out.il, inferior_left);
}

void GaitExecutor::normalize_gait() {
    #if DEBUG_GAIT_EXECUTOR
    debug((std::string)__func__+" Executing...");
    #endif

    // Shift to origin first
    gait_out.sr.x -= gait_out.com.position.x;
    gait_out.sr.y -= gait_out.com.position.y;
    gait_out.sr.z -= gait_out.com.position.z;

    gait_out.sl.x -= gait_out.com.position.x;
    gait_out.sl.y -= gait_out.com.position.y;
    gait_out.sl.z -= gait_out.com.position.z;

    gait_out.ir.x -= gait_out.com.position.x;
    gait_out.ir.y -= gait_out.com.position.y;
    gait_out.ir.z -= gait_out.com.position.z;

    gait_out.il.x -= gait_out.com.position.x;
    gait_out.il.y -= gait_out.com.position.y;
    gait_out.il.z -= gait_out.com.position.z;

    gait_out.com.position.x = 0;
    gait_out.com.position.y = 0;
    gait_out.com.position.z = 0;

    // Rotate Each Foot  
    tf::Quaternion q(
        gait_out.com.orientation.x,
        gait_out.com.orientation.y,
        gait_out.com.orientation.z,
        gait_out.com.orientation.w);
    tf::Matrix3x3 m(q);

    tf::Vector3 sr_vec(gait_out.sr.x, gait_out.sr.y, gait_out.sr.z);
    tf::Vector3 sl_vec(gait_out.sl.x, gait_out.sl.y, gait_out.sl.z);
    tf::Vector3 ir_vec(gait_out.ir.x, gait_out.ir.y, gait_out.ir.z);
    tf::Vector3 il_vec(gait_out.il.x, gait_out.il.y, gait_out.il.z);

    
    tf::Vector3 sr_norm, sl_norm, ir_norm, il_norm;

    sr_norm = m.inverse()*sr_vec;
    sl_norm = m.inverse()*sl_vec;
    ir_norm = m.inverse()*ir_vec;
    il_norm = m.inverse()*il_vec;

    gait_out.sr.x = sr_norm.getX();
    gait_out.sr.y = sr_norm.getY();
    gait_out.sr.z = sr_norm.getZ();

    gait_out.sl.x = sl_norm.getX();
    gait_out.sl.y = sl_norm.getY();
    gait_out.sl.z = sl_norm.getZ();

    gait_out.ir.x = ir_norm.getX();
    gait_out.ir.y = ir_norm.getY();
    gait_out.ir.z = ir_norm.getZ();

    gait_out.il.x = il_norm.getX();
    gait_out.il.y = il_norm.getY();
    gait_out.il.z = il_norm.getZ();

    gait_out.com.orientation.x = 0;
    gait_out.com.orientation.y = 0;
    gait_out.com.orientation.z = 0;
    gait_out.com.orientation.w = 1;
}

void GaitExecutor::print_gait(Gait gait) {
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

    #if DEBUG_GAIT_EXECUTOR
    debug(values_vec, "COM: x: |, y: |, z: |\nSR: x: |, y: |, z: |\nSL x: |, y: |, z: |\nIR: x: |, y: |, z: | \nIL: x: |, y: |, z: | ");
    #endif

}

/*
Default Leg Positions

SR:

rostopic pub /manual_position std_msgs/String "0 -0.15 -0.055 -0.15"
"0 0 -0.055 -0.14"

SL:

"1 0 -0.055 -0.14"

IR:

"2 -0.05 -0.055 -0.14"

IL:

"3 -0 -0.055 -0.14"


*/