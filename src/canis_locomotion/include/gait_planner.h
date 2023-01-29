#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <robot_core/Gait.h>
#include <robot_core/GaitVec.h>

using namespace robot_core;
using namespace geometry_msgs;

class GaitPlanner {
    public:
        // Constructor
        GaitPlanner(const ros::NodeHandle &nh_private_);

        // Destructor
        ~GaitPlanner() = default;

        // Callback methods
        void Gait_CB(const robot_core::Gait::ConstPtr& gait);
        void Vel_CB(const geometry_msgs::TwistStamped::ConstPtr& twist);
        void Reset_CB(const std_msgs::Bool::ConstPtr& reset);

        void Pose_Update(const ros::TimerEvent& event);

        // Operation Methods
        void Command_SR();
        void Command_SL();
        void Command_IR();
        void Command_IL();
        void Command_Body();
        void Init();
        void Calc_Deltas();

        // Public Variables
        double operating_freq;

        // Debugging
        void debug(std::vector<double> values, std::string message);


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
        ros::Publisher gait_pub;

        ros::Publisher debug_pub;

        ros::Subscriber gait_sub;
        ros::Subscriber vel_sub;
        ros::Subscriber reset_sub;
    
        // #### Gait Variables ####
 
        double walking_z;
        double step_height;
        Gait gait_command;
        Gait gait_current;
        double x_vel;
        double theta_vel;
        double delta_dist;
        double delta_theta;
        double margin_dist;
        double margin_theta;
        bool on;

        // #### Testing ####
        double delta_angle;
        double angle;
        double radius;
        double period;
        double rot_freq;
        ros::Publisher vel_pub;

};

double double_lerp(double x1, double x2, double percent);
Point point_lerp(Point p1, Point p2, double percent);
Gait gait_lerp(Gait g1, Gait g2, double percent);
Gait normalize_gait(Gait gait);