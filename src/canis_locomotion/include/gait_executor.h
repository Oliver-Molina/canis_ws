#include <math.h>
#include <iomanip>
#include <iostream>
#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <robot_core/Gait.h>
#include <robot_core/GaitVec.h>

using namespace robot_core;
using namespace geometry_msgs;

class GaitExecutor {
    public:
        // Constructor
        GaitExecutor(const ros::NodeHandle &nh_private_);

        // Destructor
        ~GaitExecutor() = default;

        // Callback methods
        void Gait_Replace_CB(const robot_core::GaitVec::ConstPtr& gait);

        /**
         * Gait_CB (Gait Callback)
         *
         * Calculates delta_percent based off of the current gait and input gait to maintain th current velocity
         *
         * @param gait Input gait to be compared against the current gait
         */
        void Gait_CB(const robot_core::Gait::ConstPtr& gait);

        /**
         * Vel_CB (Velocity Callback)
         *
         * Stores the desired translational and angular velocity of the robot. 
         *
         * @param twist This expresses velocity of the dog broken into its linear and angular parts.
         */
        void Vel_CB(const geometry_msgs::TwistStamped::ConstPtr& twist);

        /**
         * 
         * Unimplemented
         * Reset_CB (Reset Callback)
         *
         * Sets the angular and translational velocity of the robot to zero.
         *
         * @param reset Boolean flag indicating that a velocity reset is desired
         */
        void Reset_CB(const std_msgs::Bool::ConstPtr& reset);


        /**
         * Pose_Update (Position Update)
         * 
         * Increments the precent_step and calls Command_Body to move to its next position along its movement.
         * 
         * @param event It's just a timer.
        */
        void Pose_Update(const ros::TimerEvent& event);

        // Operation Methods

        /**
         * Command_SR (Command Superior Ritgh Leg)
         * 
         * Updates the positon of the superior right leg.
        */
        void Command_SR();

        /**
         * Command_SL (Command Superior Left Leg)
         * 
         * Updates the positon of the superior left leg.
        */
        void Command_SL();

        /**
         * Command_IR (Command Inferior Right Leg)
         * 
         * Updates the positon of the inferior right leg.
        */
        void Command_IR();

       /**
         * Command_IL (Command inferior Left Leg)
         * 
         * Updates the positon of the inferior left leg.
        */
        void Command_IL();

        /**
         * Computes the desired elevation of the lifted foot
         */
        Gait gait_raise_foot(Gait gait);

        /**
         * Command_Body
         * 
         *  Linearily interpolate and normalize the current gait position.
        */
        void Command_Body();

        /**
         * Init
         * 
         * Sets the robot to its default idle standing position.
        */
        void Init();
        Gait normalize_gait(Gait gait);
        Gait gait_lerp(Gait g1, Gait g2, double percent);

        // Debugging
        void debug(std::vector<double> values, std::string message);
        void debug(std::string message);
        void print_gait(Gait gait);

        double operating_freq; // TBD, more testing



    private:
        /**
         * Node handlers
         */
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        /*
         * Robot Params (Can be passed as params)
         */

        double shoulder_length;
        double arm_length;
        double forearm_length;

        double max_extend;
        double max_tangential;

        double body_width;
        double center_to_front;
        double center_to_back;

        /*
         * Messages
         */

        geometry_msgs::PointStamped sr_msg;
        geometry_msgs::PointStamped sl_msg;
        geometry_msgs::PointStamped ir_msg;
        geometry_msgs::PointStamped il_msg;

        std_msgs::String debug_msg;


        /**
         * Publishers and subscribers
         */
        ros::Publisher sr_pub;
        ros::Publisher sl_pub;
        ros::Publisher ir_pub;
        ros::Publisher il_pub;
        ros::Publisher percent_pub;

        ros::Publisher pose_pub;
        ros::Publisher pose_norm_pub;

        ros::Publisher debug_pub;

        ros::Subscriber gait_sub;
        ros::Subscriber vel_sub;
        ros::Subscriber reset_sub;
    
        // #### Gait Variables ####
 
        double walking_z;
        double step_height;
        Gait gait_normalized;
        Gait gait_current;
        Gait gait_next;
        double percent_step;
        double x_vel;
        double theta_vel;
        double delta_percent;

        std_msgs::Float64 percent_msg;

        Point sr, sl, ir, il;

        // #### Testing ####
        double percent_dist;
        double percent_theta;

};
/**
 * double_lerp (Linear Interpolation double type)
 * @param x1 the current position value
 * @param x2 the desired position value
 * @param percent the percentage completion of the linear interpolation in decimal form
*/
double double_lerp(double x1, double x2, double percent);

/**
 * point_lerp (Linear Interpolation Point type)
 * @param x1 the current position value
 * @param x2 the desired position value
 * @param percent the percentage completion of the linear interpolation in decimal form
*/
Point point_lerp(Point p1, Point p2, double percent);