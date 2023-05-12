#include <iostream>
#include <vector>
#include <queue>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include <robot_core/Gait.h>
#include <robot_core/GaitVec.h>
#include <robot_core/PathQuat.h>


using namespace robot_core;
using namespace geometry_msgs;

enum WalkMode {Halt, SR, SL, IR, IL};

class GaitPlanner {
    public:
        // Constructor
        GaitPlanner(const ros::NodeHandle &nh_private_);

        // Destructor
        ~GaitPlanner() = default;

        // Callback methods
        void Gait_CB(const robot_core::Gait::ConstPtr& gait);
        void Vel_CB(const geometry_msgs::TwistStamped::ConstPtr& twist);
        void Path_CB(const PathQuat::ConstPtr& path);
        void Reset_CB(const std_msgs::Bool::ConstPtr& reset);
        void Percent_CB(const std_msgs::Float64::ConstPtr& percent_msg);
        void Pose_Update(const ros::TimerEvent& event);

        // Operation Methods
        void Command_SR();
        void Command_SL();
        void Command_IR();
        void Command_IL();
        void Command_Body();
        void Init();
        void Calc_Deltas();
        std::vector<Gait> calculatePath();
        std::vector<Gait> turn(double turn_rad);
        std::vector<Gait> walk(double dist);
        geometry_msgs::Pose safePose(double dist);
        geometry_msgs::Pose zeroPose();
        Gait zeroGait();
        Gait transformGait(Gait gait, Pose transform);
        Gait normalize_gait(Gait gait);
        std::vector<Gait> pathCommand(nav_msgs::Odometry end, nav_msgs::Odometry start);
        void InitializeGaits(); 

        // Public Variables
        double operating_freq;

        // Debugging
        void debug(std::vector<double> values, std::string message);
        void debug(std::string message);
        void print_gait(Gait gait);
    




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
        std_msgs::String test_leg_position_msg;


        /**
         * Publishers, subscribers & messages
         */
        ros::Publisher gait_pub;
        ros::Publisher debug_pub;
        ros::Publisher test_leg_position_pub;

        //ros::Subscriber gait_sub;
        ros::Subscriber vel_sub;
        ros::Subscriber reset_sub;
        ros::Subscriber percent_sub;
        ros::Subscriber path_sub;

        std_msgs::Float64 percent_msg;
    
        // #### Gait Variables ####
 
        double walking_z;
        double step_height;
        double leg_x_offset;
        double leg_x_separation;
        Gait gait_command;
        Gait gait_current;
        std::queue<Gait> gait_queue;
        geometry_msgs::Pose pose_command;
        geometry_msgs::Pose pose_current;
        double x_vel;
        double theta_vel;
        double delta_dist;
        double delta_theta;
        double margin;
        bool on;
        double percent;
        WalkMode mode;
        std::vector<nav_msgs::Odometry> path;
        double step_turn_rad;


        // #### Standard Gaits ####
        
        Gait sr_fwd;
        Gait sl_fwd;
        Gait ir_fwd;
        Gait il_fwd;

        Gait sr_turn;
        Gait sl_turn;
        Gait ir_turn;
        Gait il_turn;

        Gait halt;

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
nav_msgs::Odometry translate(nav_msgs::Odometry end, nav_msgs::Odometry start);
geometry_msgs::Point rotate2D(double rad, geometry_msgs::Point point);
