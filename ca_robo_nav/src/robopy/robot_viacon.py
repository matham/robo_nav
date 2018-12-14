#!/usr/bin/env python
from __future__ import print_function
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from people_msgs.msg import People, Person
import tf
from tf.msg import tfMessage
import time
from tf.transformations import quaternion_from_euler, quaternion_multiply, quaternion_inverse

import roslib
import rospy

from geometry_msgs.msg import Twist, TransformStamped


class PersonControl(object):

    def run(self):
        rospy.init_node('viacon_robot_detection', anonymous=True)
        self.tf_pub = rospy.Publisher('tf', tfMessage, queue_size=1)
        self.tf_pub = tf.TransformBroadcaster()
        self.viacon_sub = rospy.Subscriber("/vicon/jackal3/jackal3", TransformStamped, self.viacon_callback)
        rospy.spin()

    def viacon_callback(self, msg):
        q = msg.transform.rotation
        q = quaternion_multiply([q.x, q.y, q.z, q.w], quaternion_from_euler(0, 0, 45 / 180.0 * 3.14))
        self.tf_pub.sendTransform(
            (-msg.transform.translation.x, -msg.transform.translation.y, -msg.transform.translation.z),
            q,
            rospy.Time.now(), '/base_link', "/map")


if __name__ == '__main__':
    PersonControl().run()
