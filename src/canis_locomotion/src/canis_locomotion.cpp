#include <math.h>
#include <iomanip>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>

#include "../include/canis_locomotion.h"

#include <robot_core/Gait.h>

// TODO:
// Condition to check if safe to keep moving forwards

LocomotionProcessor::LocomotionProcessor(const ros::NodeHandle &nh_private_) {
    
    vel_sub = nh_.subscribe<geometry_msgs::TwistStamped>("/command/twist", 1000, &LocomotionProcessor::Twist_CB, this);

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
    nh_.param<double>("/triangle_boundary", safe_bound_triangle, 0.03);
    nh_.param<double>("/relative_step_rate", relative_step_rate, 10.0);
    nh_.param<double>("/step_height", step_height, 0.05);

    // #### Robot Variables ####
    x_vel = 0;
    y_vel = 0;
    theta_vel = 0;
    turning_rad = 0;
    state = IR_Next;
    total_len = (center_to_front + center_to_back);
    

    // #### Leg Positions & Variables ####
    sr_x = 0; // Superior Right X (Relative to Center of Mass)
    sr_y = 0; // Superior Right Y (Relative to Center of Mass)
    sr_z = 0; // Superior Right Z (Relative to Center of Mass)

    sr_l = 0; // Superior Right Tangential XY Extention (Relative to SHOULDER)

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

    sr_cx = 0;
    sr_cy = 0; 
    sr_cz = 0; 

    sl_cx = 0;
    sl_cy = 0;
    sl_cz = 0;

    ir_cx = 0;
    ir_cy = 0;
    ir_cz = 0;

    il_cx = 0;
    il_cy = 0;
    il_cz = 0;

    leg_delta = 0;

    step_pull_distance = 0.125 * total_len;
}

void LocomotionProcessor::Twist_CB(const geometry_msgs::TwistStamped::ConstPtr& twist) {

    x_vel = twist->twist.linear.x;
    theta_vel = twist->twist.angular.z;

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
    sr_y = -body_width / 2.0 + shoulder_length;
    sr_z = -walking_z;

    sl_x = center_to_front;
    sl_y = body_width / 2.0 + shoulder_length;
    sl_z = -walking_z;

    ir_x = -center_to_back;
    ir_y = -body_width / 2.0 + shoulder_length;
    ir_z = -walking_z;

    il_x = -center_to_back;
    il_y = body_width / 2.0 + shoulder_length;
    il_z = -walking_z;

    il_x += (center_to_front + center_to_back) / 4.0;
    sr_x -= (center_to_front + center_to_back) / 4.0;
    
    LocomotionProcessor::Command_SR();
    LocomotionProcessor::Command_SL();
    LocomotionProcessor::Command_IR();
    LocomotionProcessor::Command_IL();

    // On Ros Init, Legs should move to des positions

}

void LocomotionProcessor::Move_Body(double x, double y) {
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

    LocomotionProcessor::Command_SR();
    LocomotionProcessor::Command_SL();
    LocomotionProcessor::Command_IR();
    LocomotionProcessor::Command_IL();
    
}


void LocomotionProcessor::Vel_Update(const ros::TimerEvent& event) {

    if (theta_vel == 0) {
        turning_rad = INFINITY;
    }
    else {
        turning_rad = x_vel / theta_vel;
    }

    double dx = x_vel / operating_freq;
    double dtheta = theta_vel / operating_freq;
    double dy = 0;

    /*std::ostringstream stringstream;
    stringstream << std::fixed << "state: " << state << std::endl;
    std::string str = stringstream.str();
    debug_msg.data = str.c_str();
    debug_pub.publish(debug_msg);*/

    //if (1 /* Safe() */) {

        double sr_x_new = (sr_y * dtheta - dx);
        double sl_x_new = (sl_y * dtheta - dx);
        double ir_x_new = (ir_y * dtheta - dx);
        double il_x_new = (il_y * dtheta - dx);

        double sr_y_new = (-sr_x * dtheta - dy);
        double sl_y_new = (-sl_x * dtheta - dy);
        double ir_y_new = (-ir_x * dtheta - dy);
        double il_y_new = (-il_x * dtheta - dy);

        double sr_z_new = 0;
        double sl_z_new = 0;
        double ir_z_new = 0;
        double il_z_new = 0;

    //}

    //if (1 /* Safe() */) {
        switch(state) { 
            case State::Halt: {    // nop
                //std::ostringstream stringstream;
                //stringstream << std::fixed << "xvel: " << x_vel << " , thetavel" << theta_vel;
                //std::string str = stringstream.str();
                //debug_msg.data = str.c_stPossg);
                break;
            }

            case State::IR_Next:
                if (Stable()) {
                    
                    ir_z += step_height;
                    ir_x_new = 0;
                    ir_y_new = 0;
                    leg_delta = 0;

                    LocomotionProcessor::Command_IR();

                    state = State::IR_Step;
                }
                break;

            case State::IR_Step:  
                ir_x_new = 0;
                ir_y_new = 0;

                leg_delta += relative_step_rate * x_vel / operating_freq;
                ir_x += relative_step_rate * x_vel / operating_freq;
                
                if (leg_delta >= (center_to_front + center_to_back) / 2.0) {
                    ir_z -= step_height;
                    LocomotionProcessor::Command_IR();

                    state = State::SR_Next;
                }
                break;

            case State::SR_Next:
                if (Stable()) {
                    sr_x_new = 0;
                    sr_y_new = 0;
                    
                    sr_z += step_height;
                    leg_delta = 0;

                    LocomotionProcessor::Command_SR();

                    state = State::SR_Step;
                }
                break;

            case State::SR_Step:  
                sr_x_new = 0;
                sr_y_new = 0;

                leg_delta += relative_step_rate * x_vel / operating_freq;
                sr_x += relative_step_rate * x_vel / operating_freq;
                
                if (leg_delta >= (center_to_front + center_to_back) / 2.0) {
                    sr_z -= step_height;
                    LocomotionProcessor::Command_SR();

                    state = State::IL_Next;
                }
                break;

            case State::IL_Next:
                if (Stable()) {
                    il_x_new = 0;
                    il_y_new = 0;
                    
                    il_z += step_height;
                    leg_delta = 0;

                    LocomotionProcessor::Command_IL();

                    state = State::IL_Step;
                }
                break;

            case State::IL_Step:  
                il_x_new = 0;
                il_y_new = 0;

                leg_delta += relative_step_rate * x_vel / operating_freq;
                il_x += relative_step_rate * x_vel / operating_freq;
                
                if (leg_delta >= (center_to_front + center_to_back) / 2.0) {
                    il_z -= step_height;
                    LocomotionProcessor::Command_IL();

                    state = State::SL_Next;
                }
                break;

            case State::SL_Next:
                if (Stable()) {
                    sl_x_new = 0;
                    sl_y_new = 0;
                    
                    sl_z += step_height;
                    leg_delta = 0;

                    LocomotionProcessor::Command_SL();

                    state = State::SL_Step;
                }
                break;

            case State::SL_Step:  
                sl_x_new = 0;
                sl_y_new = 0;

                leg_delta += relative_step_rate * x_vel / operating_freq;
                sl_x += relative_step_rate * x_vel / operating_freq;
                
                if (leg_delta >= (center_to_front + center_to_back) / 2.0) {
                    sl_z -= step_height;
                    LocomotionProcessor::Command_IL();

                    state = State::IR_Next;
                }
                
            
            /*case State::IR_Step:  // ir step
                ir_x_new = step_vel / operating_freq; ir_y_new = 0;
                if (sqrt(ir_x * ir_x + ir_y * ir_y) - safe_bound > max_tangential) {
                    ir_z = -walking_z;
                    state = State::SR_Next;
                }
                break;

            case State::SR_Next:  // sr step
                if (Stable()) {
                    sr_x_new = 0;
                    sr_y_new = 0;
                    
                    sr_z = -walking_z + 0.05;

                    state = State::SR_Step;
                }
                break;
            
            case State::SR_Step:  // sr step
                sr_x_new = step_vel / operating_freq; sr_y_new = 0;
                if (sqrt(sr_x * sr_x + sr_y * sr_y) - safe_bound > max_tangential) {
                    sr_z = -walking_z;
                    state = State::IL_Next;
                }
                break;*/
        } 

        // Apply Velocity 

        sr_x += sr_x_new;
        sl_x += sl_x_new;
        ir_x += ir_x_new;
        il_x += il_x_new;

        sr_y += sr_y_new;
        sl_y += sl_y_new;
        ir_y += ir_y_new;
        il_y += il_y_new;   

        // Z only changes on state transition
    //}



    LocomotionProcessor::Command_SR();
    LocomotionProcessor::Command_SL();
    LocomotionProcessor::Command_IR();
    LocomotionProcessor::Command_IL();
}

bool LocomotionProcessor::Stable() {
    switch(state) { 
            case State::Halt: { // nop
                return true; // Should I have this just return true, or ony after completely reset to 
            }

            //  x   x
            //  
            //  
            //      x
            case State::IL_Next: { // Li, is sl, Lj is sr, Lk ir
                double Lix = sr_x;
                double Ljx = sl_x;
                double Lkx = ir_x;
                //il_x;

                double Liy = sr_y;
                double Ljy = sl_y;
                double Lky = ir_y;
                //il_y; 
                
                il_cx = (Lix + Ljx + Lkx) * 0.333333;
                il_cy = (Liy + Ljy + Lky) * 0.333333;

                double Lcx = il_cx;
                double Lcy = il_cy;

                double xi = (1 - safe_bound_triangle) * Lix + safe_bound_triangle * Lcx;
                double yi = (1 - safe_bound_triangle) * Liy + safe_bound_triangle * Lcy;

                double xj = (1 - safe_bound_triangle) * Ljx + safe_bound_triangle * Lcx;
                double yj = (1 - safe_bound_triangle) * Ljy + safe_bound_triangle * Lcy;

                double xk = (1 - safe_bound_triangle) * Lkx + safe_bound_triangle * Lcx;
                double yk = (1 - safe_bound_triangle) * Lky + safe_bound_triangle * Lcy;

                double d1 = safe_inter(xi, xj, yi, yj);
                double d2 = safe_inter(xj, xk, yj, yk);
                double d3 = safe_inter(xk, xi, yk, yi);

                return ((d1 > 0 && d2 > 0 && d3 > 0) || (d1 < 0 && d2 < 0 && d3 < 0));
            }
                //break;
                    
            case State::IL_Step: { // il step
                break;
            }

            //      x
            //  
            //  
            //  x   x
            case State::SL_Next: { // Li, is sr, Lj is ir, Lk il
            
                double Lix = sr_x;
                double Ljx = ir_x;
                double Lkx = il_x;
                
                double Liy = sr_y;
                double Ljy = ir_y;
                double Lky = il_y;
                
                sl_cx = (Lix + Ljx + Lkx) * 0.333333;
                sl_cy = (Liy + Ljy + Lky) * 0.333333;

                double Lcx = sl_cx;
                double Lcy = sl_cy;

                double xi = (1 - safe_bound_triangle) * Lix + safe_bound_triangle * Lcx;
                double yi = (1 - safe_bound_triangle) * Liy + safe_bound_triangle * Lcy;

                double xj = (1 - safe_bound_triangle) * Ljx + safe_bound_triangle * Lcx;
                double yj = (1 - safe_bound_triangle) * Ljy + safe_bound_triangle * Lcy;

                double xk = (1 - safe_bound_triangle) * Lkx + safe_bound_triangle * Lcx;
                double yk = (1 - safe_bound_triangle) * Lky + safe_bound_triangle * Lcy;

                double d1 = safe_inter(xi, xj, yi, yj);
                double d2 = safe_inter(xj, xk, yj, yk);
                double d3 = safe_inter(xk, xi, yk, yi);

                return ((d1 > 0 && d2 > 0 && d3 > 0) || (d1 < 0 && d2 < 0 && d3 < 0));
                //break;
            }

            case State::SL_Step: { // sl step

                break;
            }

            //  x   x
            //  
            //  
            //  x    
            case State::IR_Next:{ // Li, is sr, Lj is sl, Lk il
                double Lix = sr_x;
                double Ljx = sl_x;
                double Lkx = il_x;
                //il_x;

                double Liy = sr_y;
                double Ljy = sl_y;
                double Lky = il_y;
                //il_y; 
                
                ir_cx = (Lix + Ljx + Lkx) * 0.333333;
                ir_cy = (Liy + Ljy + Lky) * 0.333333;

                double Lcx = ir_cx;
                double Lcy = ir_cy;

                double xi = (1 - safe_bound_triangle) * Lix + safe_bound_triangle * Lcx;
                double yi = (1 - safe_bound_triangle) * Liy + safe_bound_triangle * Lcy;

                double xj = (1 - safe_bound_triangle) * Ljx + safe_bound_triangle * Lcx;
                double yj = (1 - safe_bound_triangle) * Ljy + safe_bound_triangle * Lcy;

                double xk = (1 - safe_bound_triangle) * Lkx + safe_bound_triangle * Lcx;
                double yk = (1 - safe_bound_triangle) * Lky + safe_bound_triangle * Lcy;

                double d1 = safe_inter(xi, xj, yi, yj);
                double d2 = safe_inter(xj, xk, yj, yk);
                double d3 = safe_inter(xk, xi, yk, yi);

                return ((d1 > 0 && d2 > 0 && d3 > 0) || (d1 < 0 && d2 < 0 && d3 < 0));
            }
                //break;
            
            case State::IR_Step: { // ir step

                break;
            }

            //  x   
            //  
            //  
            //  x   x
            case State::SR_Next: { // Li, is sl, Lj is ir, Lk il
                double Lix = sl_x;
                double Ljx = ir_x;
                double Lkx = il_x;
                //il_x;

                double Liy = sr_y;
                double Ljy = ir_y;
                double Lky = il_y;
                //il_y; 
                
                sr_cx = (Lix + Ljx + Lkx) * 0.333333;
                sr_cy = (Liy + Ljy + Lky) * 0.333333;

                double Lcx = sr_cx;
                double Lcy = sr_cy;

                double xi = (1 - safe_bound_triangle) * Lix + safe_bound_triangle * Lcx;
                double yi = (1 - safe_bound_triangle) * Liy + safe_bound_triangle * Lcy;

                double xj = (1 - safe_bound_triangle) * Ljx + safe_bound_triangle * Lcx;
                double yj = (1 - safe_bound_triangle) * Ljy + safe_bound_triangle * Lcy;

                double xk = (1 - safe_bound_triangle) * Lkx + safe_bound_triangle * Lcx;
                double yk = (1 - safe_bound_triangle) * Lky + safe_bound_triangle * Lcy;

                double d1 = safe_inter(xi, xj, yi, yj);
                double d2 = safe_inter(xj, xk, yj, yk);
                double d3 = safe_inter(xk, xi, yk, yi);

                return ((d1 > 0 && d2 > 0 && d3 > 0) || (d1 < 0 && d2 < 0 && d3 < 0));
                //break  ;
            }
            
            case State::SR_Step: { // sr step
                break;
            }
        } 
}

bool LocomotionProcessor::Safe() {

}

double safe_inter(double xa, double xb, double ya, double yb) {
    return (-xb) * (ya - yb) - (xa - xb) * (-yb);
}


