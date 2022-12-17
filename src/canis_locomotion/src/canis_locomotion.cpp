#include <math.h>
#include <iomanip>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>

#include "../include/canis_locomotion.h"

// TODO:
// Condition to check if safe to keep moving forwards

LocomotionProcessor::LocomotionProcessor(const ros::NodeHandle &nh_private_) {
    
    vel_sub = nh_.subscribe<geometry_msgs::TwistStamped>("/command/twist", 1000, &LocomotionProcessor::Twist_CB, this);

    sr_pub = nh_.advertise<geometry_msgs::PointStamped>("/desired_pos/superior/right", 1000);
    sl_pub = nh_.advertise<geometry_msgs::PointStamped>("/desired_pos/superior/left", 1000);
    ir_pub = nh_.advertise<geometry_msgs::PointStamped>("/desired_pos/inferior/right", 1000);
    il_pub = nh_.advertise<geometry_msgs::PointStamped>("/desired_pos/inferior/left", 1000);

    debug_pub = nh_.advertise<std_msgs::String>("/debug", 1000);

    //timer = nh_.createTimer(ros::Duration(1.0 / operating_freq), LocomotionProcessor::Pos_Update);

    // #### Robot Params ####
    shoulder_length = 0.055;
    arm_length = 0.105;
    forearm_length = 0.136;

    body_width = 0.2;
    center_to_front = 0.1;
    center_to_back = 0.1;

    operating_freq = 30; // TBD, more testing

    // Shoulder to Foot Max Extension
    max_extend = sqrt(shoulder_length * shoulder_length + (arm_length + forearm_length) * (arm_length + forearm_length));
    // Max Extension Tangential to the Floor
    max_tangential = sqrt(max_extend * max_extend - walking_z * walking_z);
    // Boundary for safe balancing triangle
    safe_bound_triangle = 0.03; // This is a percentage

    // #### State Variables ####
    x_vel = 0;
    y_vel = 0;
    theta_vel = 0;

    step_vel = 0.1;

    walking_z = 0.1;

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

    moving = 0;

    state = Halt;
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
    
    //LocomotionProcessor::Command_SR();
    //LocomotionProcessor::Command_SL();
    //LocomotionProcessor::Command_IR();
    //LocomotionProcessor::Command_IL();

    // On Ros Init, Legs should move to des positions

}

void LocomotionProcessor::Pos_Update(const ros::TimerEvent& event) {
    double dx = x_vel / operating_freq;
    double dtheta = theta_vel / operating_freq;
    double dy = 0;

    //if (1 /* Safe() */) {

        double sr_x_new = (sr_l * sr_y * dtheta - dx);
        double sl_x_new = (sl_l * sl_y * dtheta - dx);
        double ir_x_new = (ir_l * ir_y * dtheta - dx);
        double il_x_new = (il_l * il_y * dtheta - dx);

        double sr_y_new = (-sr_l * sr_x * dtheta - dy);
        double sl_y_new = (-sl_l * sl_x * dtheta - dy);
        double ir_y_new = (-ir_l * ir_x * dtheta - dy);
        double il_y_new = (-il_l * il_x * dtheta - dy);

        double sr_z_new = 0;
        double sl_z_new = 0;
        double ir_z_new = 0;
        double il_z_new = 0;

    //}

    //if (1 /* Safe() */) {
        switch(state) { 
            case State::Halt:     // nop
                if (x_vel != 0 && theta_vel != 0)
                    state = State::IL_Next;
                break;

            case State::IL_Next:  // il step
                // Move to safe triangle
                if (Stable()) {
                    il_x_new = 0;
                    il_y_new = 0;

                    il_z = -walking_z + 0.05;

                    state = State::IL_Step;
                }
                break;

            case State::IL_Step:  // il step
                // Move to safe triangle

                il_x_new = step_vel / operating_freq; il_y_new = 0;
                if (sqrt(il_x * il_x + il_y * il_y) - safe_bound > max_tangential) {
                    il_z = -walking_z;
                    state = State::SL_Next;
                }
                
                break;

            case State::SL_Next:  // sl step
                if (Stable()) {
                    sl_x_new = 0;
                    sl_y_new = 0;
                    
                    sl_z = -walking_z + 0.05;

                    state = State::SL_Step;
                }
                break;

            case State::SL_Step:  // sl step

                sl_x_new = step_vel / operating_freq; sl_y_new = 0;
                if (sqrt(sl_x * sl_x + sl_y * sl_y) - safe_bound > max_tangential) {
                    sl_z = -walking_z;
                    state = State::IR_Next;
                }

                break;

            case State::IR_Next:  // ir step
                if (Stable()) {
                    ir_x_new = 0;
                    ir_y_new = 0;
                    
                    ir_z = -walking_z + 0.05;

                    state = State::IR_Step;
                }
                break;
            
            case State::IR_Step:  // ir step
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
                break;
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


