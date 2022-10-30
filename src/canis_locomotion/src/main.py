#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import PointStamped

import math
from math import sin, cos
rad = 0.05
xc = 0.0
zc = -0.12
ros_freq = 90
period = 1
updatetheta = (2 * math.pi) / (period * ros_freq)

def walker():
    pub1 = rospy.Publisher('/desired_pos/superior/right', PointStamped, queue_size=10)
    pub2 = rospy.Publisher('/desired_pos/superior/left', PointStamped, queue_size=10)
    pub3 = rospy.Publisher('/desired_pos/inferior/right', PointStamped, queue_size=10)
    pub4 = rospy.Publisher('/desired_pos/inferior/left', PointStamped, queue_size=10)
    
    rospy.init_node('walker', anonymous=True)
    rate = rospy.Rate(ros_freq)
    theta = 0
    point1 = PointStamped()
    point2 = PointStamped()
    point3 = PointStamped()
    point4 = PointStamped()
    while not rospy.is_shutdown():
        theta = theta + updatetheta
        if (theta > 2 * math.pi): 
            theta = theta - 2 * math.pi
        #point = PointStamped()
        point1.point.x = xc# + rad * sin(theta)
        point1.point.y = 0.065
        point1.point.z = zc + rad * cos(theta)
        pub1.publish(point1)

        point2.point.x = xc# + rad * sin(theta)
        point2.point.y = 0.065
        point2.point.z = zc + rad * cos(theta)
        pub2.publish(point2)

        point3.point.x = xc# + rad * sin(theta)
        point3.point.y = 0.065
        point3.point.z = zc + rad * cos(theta)
        pub3.publish(point3)

        point4.point.x = xc# + rad * sin(theta)
        point4.point.y = 0.065
        point4.point.z = zc + rad * cos(theta)
        pub4.publish(point4)
        rate.sleep()

if __name__ == '__main__':
    try:
        walker()
    except rospy.ROSInterruptException:
        pass