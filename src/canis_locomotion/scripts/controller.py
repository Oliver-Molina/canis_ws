#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from robot_core.msg import PathQuat

vel = TwistStamped()

path = PathQuat()
odom0 = Odometry()
odom1 = Odometry()

vel_pub = rospy.Publisher('/command/velocity', TwistStamped, queue_size=10)
path_pub = rospy.Publisher('/command/path', PathQuat, queue_size=10)

def dist_cb(dist):

    vel.twist.linear.x = 0.003
    vel.twist.angular.z = 0.1

    odom1.pose.pose.position.x = dist.data
    odom0.pose.pose.orientation.w = 1
    odom1.pose.pose.orientation.w = 1
    path.poses.append(odom0)
    path.poses.append(odom1)
    
    vel_pub.publish(vel)
    path_pub.publish(path)


def main():
    rospy.init_node('controller', anonymous=True)
    rospy.Subscriber("/test/command/dist", Float64, dist_cb)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass