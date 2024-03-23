import rospy
from std_msgs.msg import String, Float64, Bool
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from robot_core.msg import PathQuat

class RosCommunicationWrapper:
    publisherQueueSize = 10
    subscriberQueueSize = 10
    def __init__(self):
        # Initialize Publishers
        self.velocityPub = rospy.Publisher('/command/velocity', TwistStamped, queue_size=self.publisherQueueSize)
        self.pathPub = rospy.Publisher('/command/path', PathQuat, queue_size=self.publisherQueueSize)
        self.resetPub = rospy.Publisher('/reset/gait', Bool, queue_size=self.publisherQueueSize)
        self.sitPub = rospy.Publisher('/sit', Bool, queue_size=self.publisherQueueSize)
        self.layDownPub = rospy.Publisher('/layDown', Bool, queue_size=self.publisherQueueSize)
        self.walkPub = rospy.Publisher('/walk', Bool, queue_size=self.publisherQueueSize)
        self.manualLegPub = rospy.Publisher('/manual_leg_position', String, queue_size=self.publisherQueueSize)

