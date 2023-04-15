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
    
    gait_sub = nh_.subscribe<robot_core::Gait>("/command/gait/next", 1000, &GaitExecutor::Gait_CB, this);
    vel_sub = nh_.subscribe<geometry_msgs::TwistStamped>("/command/velocity", 1000, &GaitExecutor::Vel_CB, this);
    reset_sub = nh_.subscribe<std_msgs::Bool>("/reset/gait", 1000, &GaitExecutor::Reset_CB, this);

    sr_pub = nh_.advertise<geometry_msgs::PointStamped>("/command/leg/pos/superior/right", 1000);
    sl_pub = nh_.advertise<geometry_msgs::PointStamped>("/command/leg/pos/superior/left", 1000);
    ir_pub = nh_.advertise<geometry_msgs::PointStamped>("/command/leg/pos/inferior/right", 1000);
    il_pub = nh_.advertise<geometry_msgs::PointStamped>("/command/leg/pos/inferior/left", 1000);

    pose_pub = nh_.advertise<robot_core::Gait>("/odometry/gait/current", 1000);
    pose_norm_pub = nh_.advertise<robot_core::Gait>("/debug/gait", 1000);
    percent_pub = nh_.advertise<std_msgs::Float64>("/odometry/percent", 1000);

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
    gait_next;
    gait_normalized;
    percent_step = 0.;
    x_vel = 0;
    theta_vel = 0;
    delta_percent = 0;

    // #### Testing ####
    percent_dist = 0;
    percent_theta = 0;

}

void GaitExecutor::Gait_CB(const robot_core::Gait::ConstPtr& gait) { 
    gait_next = *gait;
    double delta_dist = 0;
    if (gait_current.foot.data != 1) {
        delta_dist = sqrt((gait_next.sr.x-gait_current.sr.x)*(gait_next.sr.x-gait_current.sr.x)+
                        (gait_next.sr.y-gait_current.sr.y)*(gait_next.sr.y-gait_current.sr.y)+
                        (gait_next.sr.z-gait_current.sr.z)*(gait_next.sr.z-gait_current.sr.z));
    }
    else {
        delta_dist = sqrt((gait_next.sl.x-gait_current.sl.x)*(gait_next.sl.x-gait_current.sl.x)+
                        (gait_next.sl.y-gait_current.sl.y)*(gait_next.sl.y-gait_current.sl.y)+
                        (gait_next.sl.z-gait_current.sl.z)*(gait_next.sl.z-gait_current.sl.z));
    }
    tf2::Quaternion quat_tf1, quat_tf2;
    tf2::convert(gait_current.com.orientation, quat_tf1);
    tf2::convert(gait_next.com.orientation, quat_tf2);

    double delta_theta = (quat_tf1 * quat_tf2.inverse()).getAngleShortestPath();
    percent_dist = (delta_dist == 0) ? 0 : x_vel / (delta_dist * operating_freq);
    percent_theta = (delta_theta == 0) ? 0 : theta_vel / (delta_theta * operating_freq);

    delta_percent = std::max(percent_theta, percent_dist);
}

void GaitExecutor::Vel_CB(const geometry_msgs::TwistStamped::ConstPtr& twist) { 

    x_vel = twist->twist.linear.x;
    theta_vel = twist->twist.angular.z;

}

void GaitExecutor::Reset_CB(const std_msgs::Bool::ConstPtr& reset) {

    if(reset)
    {
        x_vel = 0;
        theta_vel = 0;
    }
}

void GaitExecutor::Command_SR() {

    sr_msg.point.x = gait_normalized.sr.x - center_to_front;
    sr_msg.point.y = gait_normalized.sr.y + body_width / 2.0;
    sr_msg.point.z = gait_normalized.sr.z;

    sr_msg.header.stamp = ros::Time::now();
    sr_pub.publish(sr_msg);    
}

void GaitExecutor::Command_SL() {

    sl_msg.point.x = gait_normalized.sl.x - center_to_front;
    sl_msg.point.y = gait_normalized.sl.y - body_width / 2.0;
    sl_msg.point.z = gait_normalized.sl.z;

    sl_msg.header.stamp = ros::Time::now();
    sl_pub.publish(sl_msg);
    
}

void GaitExecutor::Command_IR() {
    
    ir_msg.point.x = gait_normalized.ir.x + center_to_back;
    ir_msg.point.y = gait_normalized.ir.y + body_width / 2.0;
    ir_msg.point.z = gait_normalized.ir.z;

    ir_msg.header.stamp = ros::Time::now();
    ir_pub.publish(ir_msg);
    
}

void GaitExecutor::Command_IL() {
    
    il_msg.point.x = gait_normalized.il.x + center_to_back;
    il_msg.point.y = gait_normalized.il.y - body_width / 2.0;
    il_msg.point.z = gait_normalized.il.z;

    il_msg.header.stamp = ros::Time::now();
    il_pub.publish(il_msg);
    
}

void GaitExecutor::Init() {

    gait_current.sr.x = center_to_front;
    gait_current.sr.y = -body_width / 2.0 - shoulder_length;
    gait_current.sr.z = 0;

    gait_current.sl.x = center_to_front;
    gait_current.sl.y = body_width / 2.0 + shoulder_length;
    gait_current.sl.z = 0;

    gait_current.ir.x = -center_to_back - 0.05;
    gait_current.ir.y = -body_width / 2.0 - shoulder_length;
    gait_current.ir.z = 0;

    gait_current.il.x = -center_to_back - 0.05;
    gait_current.il.y = body_width / 2.0 + shoulder_length;
    gait_current.il.z = 0;

    gait_current.com.position.x = 0;
    gait_current.com.position.y = 0;
    gait_current.com.position.z = walking_z;

    gait_current.com.orientation.x = 0;
    gait_current.com.orientation.y = 0;
    gait_current.com.orientation.z = 0;
    gait_current.com.orientation.w = 1;

    gait_normalized = normalize_gait(gait_current);
    gait_next = gait_current;
    percent_step = 0;

    GaitExecutor::Command_Body();
    
}

void GaitExecutor::Pose_Update(const ros::TimerEvent& event) {
    percent_step += delta_percent;
    percent_msg.data = percent_step;              
    percent_pub.publish(percent_msg);
    
    if (percent_step >= 1) { 
        percent_step = 0;
        gait_current = gait_next;
    }
    Command_Body();

}

void GaitExecutor::Command_Body() {
    Gait gait_linterped = gait_lerp(gait_current, gait_next, percent_step);
    Gait gait_raised_foot = gait_raise_foot(gait_linterped);
    gait_normalized = normalize_gait(gait_raised_foot);
    print_gait(gait_normalized);

    debug(std::to_string(x_vel));
    GaitExecutor::Command_SR();
    GaitExecutor::Command_SL();
    GaitExecutor::Command_IR();
    GaitExecutor::Command_IL();

    pose_pub.publish(gait_raised_foot);
    pose_norm_pub.publish(gait_normalized);
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
Gait GaitExecutor::gait_lerp(Gait g1, Gait g2, double percent) {
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
Gait GaitExecutor::normalize_gait(Gait gait) {
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

Gait GaitExecutor::gait_raise_foot(Gait gait) {
    switch (gait.foot.data) {
        case 1: {
            double desired_z = -step_height*4*(percent_step)*(percent_step - 1);
            desired_z = 1;
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

    debug(values_vec, "COM: x: |, y: |, z: |\nSR: x: |, y: |, z: |\nSL x: |, y: |, z: |\nIR: x: |, y: |, z: | \nIL: x: |, y: |, z: | ");

}