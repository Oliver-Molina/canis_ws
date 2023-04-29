#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <geometry_msgs/PointStamped.h>

//#include <tf/transform_datatypes.h>
//#include <tf/message_filter.h>
//#include <tf/transform_listener.h>

class LegInverseKinematicsProcessor
{
    public:
        // Constructor
        LegInverseKinematicsProcessor(const ros::NodeHandle &nh_private_);

        // Destructor
        ~LegInverseKinematicsProcessor() = default;

        // Callback methods
        void Superior_Right_Leg_Pos_CB(const geometry_msgs::PointStamped::ConstPtr& Point);
        void Superior_Left_Leg_Pos_CB(const geometry_msgs::PointStamped::ConstPtr&Point);
        void Inferior_Right_Leg_Pos_CB(const geometry_msgs::PointStamped::ConstPtr& Point);
        void Inferior_Left_Leg_Pos_CB(const geometry_msgs::PointStamped::ConstPtr& Point);


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

        /*
         * Messages
         */

        std_msgs::Float64 superior_right_shoulder_msg;
        std_msgs::Float64 superior_left_shoulder_msg;
        std_msgs::Float64 inferior_right_shoulder_msg;
        std_msgs::Float64 inferior_left_shoulder_msg;

        std_msgs::Float64 superior_right_arm_msg;
        std_msgs::Float64 superior_left_arm_msg;
        std_msgs::Float64 inferior_right_arm_msg;
        std_msgs::Float64 inferior_left_arm_msg;

        std_msgs::Float64 superior_right_forearm_msg;
        std_msgs::Float64 superior_left_forearm_msg;
        std_msgs::Float64 inferior_right_forearm_msg;
        std_msgs::Float64 inferior_left_forearm_msg;

        std_msgs::String debug_msg;

        std_msgs::String test_pwm_msg;


        /**
         * Publishers and subscribers
         */
        ros::Publisher superior_right_shoulder_pub;
        ros::Publisher superior_left_shoulder_pub;
        ros::Publisher inferior_right_shoulder_pub;
        ros::Publisher inferior_left_shoulder_pub;

        ros::Publisher superior_right_arm_pub;
        ros::Publisher superior_left_arm_pub;
        ros::Publisher inferior_right_arm_pub;
        ros::Publisher inferior_left_arm_pub;

        ros::Publisher superior_right_forearm_pub;
        ros::Publisher superior_left_forearm_pub;
        ros::Publisher inferior_right_forearm_pub;
        ros::Publisher inferior_left_forearm_pub;

        ros::Publisher debug_pub;

        ros::Publisher test_pwm;

        ros::Subscriber SuperiorRightSub;
        ros::Subscriber SuperiorLeftSub;
        ros::Subscriber InferiorRightSub;
        ros::Subscriber InferiorLeftSub;
        
        /**
         * Service and parameter
         */
        //...

        /**
         * ROS params
         */
        //std::string input_topic_;
        
        
        /**
         * Runtime variables
         */
        //...

};
