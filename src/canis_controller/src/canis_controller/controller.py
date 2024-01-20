#!/usr/bin/env python

import rospy

from std_msgs.msg import String, Float64
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from robot_core.msg import Gait, GaitVec, PathQuat
import canis_controller.RosCommunication as RosCommunication

import tkinter
from tkinter import *
import numpy as np 
import matplotlib.pyplot as plt

vel = TwistStamped()
path = PathQuat()
odom0 = Odometry()
odom1 = Odometry()

rosComms = RosCommunication.RosCommunicationWrapper()

mainWindow = Tk()
mainWindow.title("Canis Control Panel")
mainWindow.geometry("1024x768")

# Button Callbacks

# Subscriber Callbacks

def dist_cb(dist):

    vel.twist.linear.x = 0.003
    vel.twist.angular.z = 0.1

    odom1.pose.pose.position.x = dist.data
    odom0.pose.pose.orientation.w = 1
    odom1.pose.pose.orientation.w = 1
    path.poses.append(odom0)
    path.poses.append(odom1)
    
    rosComms.velocityPub.publish(vel)
    rosComms.pathPub.publish(path)


def main():
    rospy.init_node('controller', anonymous=True)
    rospy.Subscriber("/test/command/dist", Float64, dist_cb)
    mainWindow.mainloop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass