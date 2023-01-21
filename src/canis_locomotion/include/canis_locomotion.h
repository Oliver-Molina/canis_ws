#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>

enum State {Halt, IL_Next, IL_Step, SL_Next, SL_Step, IR_Next, IR_Step, SR_Next, SR_Step};

class LocomotionProcessor
{
    public:
        // Constructor
        LocomotionProcessor(const ros::NodeHandle &nh_private_);

        // Destructor
        ~LocomotionProcessor() = default;

        // Callback methods
        void Twist_CB(const geometry_msgs::TwistStamped::ConstPtr& twist);

        // Operation Methods
        void Command_SR();
        void Command_SL();
        void Command_IR();
        void Command_IL();
        void Move_Body(double x, double y);
        void Start_Position();
        void Init();
        void Vel_Update(const ros::TimerEvent& event);
        bool Stable();
        bool Safe();

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

        double safe_bound_triangle;

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

        ros::Subscriber vel_sub;
    
        // #### State Variables ####

        double x_vel;
        double y_vel;
        double theta_vel;
        double turning_rad;
        double total_len;

        double step_vel;

        double walking_z;

        double safe_bound;

            // Note, these are relative to COM
        double sr_x;
        double sr_y;
        double sr_z;

        double sr_l;

        double sl_x;
        double sl_y;
        double sl_z;

        double sl_l;

        double ir_x;   
        double ir_y;
        double ir_z;

        double ir_l;

        double il_x;
        double il_y;
        double il_z;

        double il_l;

        double sr_cx;
        double sr_cy; 
        double sr_cz; 

        double sl_cx;
        double sl_cy;
        double sl_cz;

        double ir_cx;
        double ir_cy;
        double ir_cz;

        double il_cx;
        double il_cy;
        double il_cz;

        double leg_delta;
        double relative_step_rate;

        double step_height;

        bool moving;

        State state;
};

double safe_inter(double xa, double xb, double ya, double yb);