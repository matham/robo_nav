import rospy
from geometry_msgs.msg import Twist
import time


class Bot(object):

    def run(self):
        rospy.init_node('robo_move', anonymous=True)
        velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        vel_msg = Twist()
        vel_msg.linear.x = .1

        while not rospy.is_shutdown():
            time.sleep(.1)
            velocity_publisher.publish(vel_msg)
