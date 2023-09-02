#include <math.h>
#include <iomanip>
#include <iostream>
#include <vector>
#include <algorithm>
#include <queue>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <regex>
#include <sstream>

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

        /**
         * Raw_Gait_CB (Gait Callback)
         *
         * Calculates delta_percent based off of the current gait and input gait to maintain th current velocity
         *
         * @param gait Input gait to be compared against the current gait
         */
        void Raw_Gait_CB(const robot_core::Gait::ConstPtr& gait);

        /**
         * Processed_Gait_CB (Gait Callback)
         *
         * Calculates delta_percent based off of the current gait and input gait to maintain th current velocity
         *
         * @param gait Input gait to be compared against the current gait
         */
        void Processed_Gait_CB(const robot_core::Gait::ConstPtr& gait);

        /**
         * Takes a point centered on the body's origin and recenters it on the origin of a given leg.
         * @param point The current point centered on the body
         * @param leg The specific leg to be recented about
         * @returns The new point recentered about the specified leg
        */
        Point recenter_point(Point point, int leg);

        // Operation Methods

        /**
         * Updates the internaly stored leg locations.
         * 
        */
        void recenter_leg_positions();

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
         * Command_Body
         * 
         *  Linearily interpolate and normalize the current gait position.
        */
        void Command_Body();

        void normalize_gait();

        // Debugging
        void debug(std::vector<double> values, std::string message);
        void debug(std::string message);
        void print_gait(Gait gait);

    private:
        // Node Handlers
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        // Robot Params (Can be passed as params)

        double shoulder_length;
        double arm_length;
        double forearm_length;
        double body_width;
        double center_to_front;
        double center_to_back;
        double operating_freq; // TBD, more testing
        double walking_z;
        double step_height;
        double leg_x_offset;
        double leg_x_separation;

        // Publishers, subscribers & messages

        ros::Publisher sr_pub;
        ros::Publisher sl_pub;
        ros::Publisher ir_pub;
        ros::Publisher il_pub;
        ros::Publisher debug_pub;

        ros::Subscriber raw_gait_sub;
        ros::Subscriber normalized_gait_sub;

        geometry_msgs::PointStamped sr_msg;
        geometry_msgs::PointStamped sl_msg;
        geometry_msgs::PointStamped ir_msg;
        geometry_msgs::PointStamped il_msg;
        std_msgs::String debug_msg;

        Gait gait_out;
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