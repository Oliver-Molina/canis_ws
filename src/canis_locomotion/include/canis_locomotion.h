#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <twist_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>

enum State {Halt, IL_Next, IL_Step, SL_Next, SR_Step, IR_Next, IR_Step, SR_Next, SR_Step};

class LocomotionProcessor
{
    public:
        // Constructor
        LocomotionProcessor(const ros::NodeHandle &nh_private_);

        // Destructor
        ~LocomotionProcessor() = default;

        // Callback methods
        void Twist_CB(const twist_msgs::TwistStamped::ConstPtr& twist);

        // Operation Methods
        void Command_SR();
        void Command_SL();
        void Command_IR();
        void Command_IL();
        void Init();
        void Pos_Update(const ros::TimerEvent& event);
        bool Stablity();
        bool Safe();



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

        double body_width;
        double center_to_front;
        double center_to_back;

        double operating_freq; // TBD, more testing

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
        ros::Timer timer;

        double x_vel;
        double y_vel;
        double theta_vel;

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

        double ir_x;    time_steps_per_step = 5;
    current_step = 0;
        double ir_y;
        double ir_z;

        double ir_l;

        double il_x;
        double il_y;
        double il_z;

        double il_l;

        bool moving;

        State state;
};