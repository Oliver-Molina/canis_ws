#!/usr/bin/env python

import rospy

from std_msgs.msg import String, Float64
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from robot_core.msg import Gait, GaitVec, PathQuat
import canis_controller.RosCommunication as RosCommunication
import canis_controller.canisBody as canisBody

import tkinter as tk
import numpy as np 
import matplotlib.pyplot as plt


class CanisUI(tk.Tk):

    class ButtonCallbacks:
        def __init__(self, mainWindow:tk.Tk, body:canisBody, rosComms:RosCommunication):
            self.mainWindow = mainWindow
            self.body = body
            self.rosComms = rosComms

        def initialize(self):
            # Velocity
            self.velocityTextBox = tk.Text(self.mainWindow, height=0, width=0)
            self.velocityTextBox.pack()
            self.velocityButton = tk.Button(self.mainWindow, text = "VelocityPub", command = self.velocityPubPressed)
            self.velocityButton.pack()

        def velocityPubPressed(self):
            self.rosComms.velocityPub.publish(self.body.vel)

    class SubscriberCallbacks:
        def __init__(self, mainWindow:tk.Tk, body:canisBody, rosComms:RosCommunication):
            self.mainWindow = mainWindow
            self.body = body
            self.rosComms = rosComms

        def initialize(self):
            # Dist
            rospy.Subscriber("/test/command/dist", Float64, self.distCallback)

        def distCallback(self, dist):
            self.body.vel.twist.linear.x = 0.003
            self.body.vel.twist.angular.z = 0.1

            self.body.odom1.pose.pose.position.x = dist.data
            self.body.odom0.pose.pose.orientation.w = 1
            self.body.odom1.pose.pose.orientation.w = 1
            self.body.path.poses.append(self.body.odom0)
            self.body.path.poses.append(self.body.odom1)
            
            self.rosComms.velocityPub.publish(self.body.vel)
            self.rosComms.pathPub.publish(self.body.path)
        

    def __init__(self):
        super().__init__()
        self.title("Canis Control Panel")
        self.geometry("1024x768")

        # ROS Init
        rospy.init_node('controller', anonymous=True)

        # Canis Body
        self.body = canisBody.canisBody()

        # ROS Communication Publishers
        self.rosComms = RosCommunication.RosCommunicationWrapper()

        self.buttonCallbacks = self.ButtonCallbacks(self, self.body, self.rosComms)
        self.buttonCallbacks.initialize()

        self.subscriberCallbacks = self.SubscriberCallbacks(self, self.body, self.rosComms)
        self.subscriberCallbacks.initialize()

def main():
    canisUI = CanisUI()
    canisUI.mainloop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass