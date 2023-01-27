#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>

class GaitExecutor
{
    public:
        // Constructor
        GaitExecutor(const ros::NodeHandle &nh_private_);

        // Destructor
        ~GaitExecutor() = default;

        // Callback methods
        void Gait_CB(const robot_core::Gait::ConstPtr& gait);
        void Reset_CB(const std_msgs::Bool::ConstPtr& reset);

        void Vel_Update(const ros::TimerEvent& event)

        // Operation Methods
        void Command_SR();
        void Command_SL();
        void Command_IR();
        void Command_IL();

        void Move_Body(double x, double y);

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

        ros::Publisher debug_pub;

        ros::Subscriber gait_sub;
        ros::Subscriber vel_sub;
        ros::Subscriber reset_sub;
    
        // #### State Variables ####

        vector<Gait> gait_vec;
 
        double walking_vel;
        double walking_z;
        double step_height;

        double sr_x;
        double sr_y;
        double sr_z;

        double sl_x;
        double sl_y;
        double sl_z;

        double ir_x;   
        double ir_y;
        double ir_z;

        double il_x;
        double il_y;
        double il_z;

};

struct Gait {
    Point sr;
    Point sl;
    Point ir;
    Point il;
};

struct Point {
    double x;
    double y;
    double z;
};