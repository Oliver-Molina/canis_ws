#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sensor_msgs/Imu.h>

#include <geometry_msgs/Quaternion.h>



void Imu_CB(const sensor_msgs::Imu::ConstPtr& imu){
    static tf::TransformBroadcaster br;
    Quaternion quat_msg = imu->orientation;

    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0)); // can add xyz later, this just applies rpy
    tf::Quaternion quat_tf;
    tf2::fromMsg(quat_msg, quat_tf);
    transform.setRotation(quat_tf);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));
}

int main(int argc, char** argv){
    ros::init(argc, argv, "normalize_tf_node");
    
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("sensors/imu", 10, &Imu_CB);

    ros::spin();
    return 0;
};