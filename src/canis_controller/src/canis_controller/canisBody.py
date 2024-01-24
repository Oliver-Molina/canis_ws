import rospy

from std_msgs.msg import String, Float64
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from robot_core.msg import Gait, GaitVec, PathQuat
import canis_controller.RosCommunication as RosCommunication

class canisBody:
    vel = TwistStamped()
    path = PathQuat()
    odom0 = Odometry()
    odom1 = Odometry()