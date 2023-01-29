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


enum Foot {SR, SL, IR, IL};

/*struct Point {
    double x;
    double y;
    double z;
};

struct Gait {
    Point sr;
    Point sl;
    Point ir;
    Point il;
    Point com;
    Foot foot;
};*/

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
        void Gait_CB(const robot_core::Gait::ConstPtr& gait);
        void Vel_CB(const geometry_msgs::TwistStamped::ConstPtr& twist);
        void Reset_CB(const std_msgs::Bool::ConstPtr& reset);

        void Vel_Update(const ros::TimerEvent& event);

        // Operation Methods
        void Command_SR();
        void Command_SL();
        void Command_IR();
        void Command_IL();

        void Move_Body(double x, double y);
        void Command_Body();

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

        std::vector<robot_core::Gait> gait_vec;
 
        double walking_z;
        double step_height;
        Gait gait_current;
        Gait gait_next;
        double percent_step;
        double x_vel;
        double theta_vel;
        int gait_index;

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

        Point sr, sl, ir, il;

};

double double_lerp(double x1, double x2, double percent);
Point point_lerp(Point p1, Point p2, double percent);
Gait gait_lerp(Gait g1, Gait g2, double percent);