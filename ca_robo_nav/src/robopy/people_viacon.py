#!/usr/bin/env python
from __future__ import print_function
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from people_msgs.msg import People, Person
import tf
import time
from tf.transformations import quaternion_from_euler

import roslib
import rospy

from geometry_msgs.msg import Twist, TransformStamped


class PersonControl(object):

    vx = 0

    vy = 0

    gamma = .7

    last_pos = None

    def run(self):
        rospy.init_node('viacon_people_detection', anonymous=True)
        self.p_pub = rospy.Publisher('people', People, queue_size=1)
        self.viz_pub = rospy.Publisher('people_viz', Marker, queue_size=1)
        self.viacon_sub = rospy.Subscriber("/vicon/Helmet_1/Helmet_1", TransformStamped, self.viacon_callback)
        rospy.spin()

    def viacon_callback(self, msg):
        if self.last_pos is None:
            self.last_pos = -msg.transform.translation.x, -msg.transform.translation.y, msg.header.stamp.to_sec()
            return

        people = People()
        person = Person()

        people.header.stamp = msg.header.stamp
        people.header.frame_id = "map"

        last_x, last_y, last_t = self.last_pos
        self.last_pos = -msg.transform.translation.x, -msg.transform.translation.y, msg.header.stamp.to_sec()
        if msg.header.stamp.to_sec() - last_t <= 0:
            return

        self.vx = (1 - self.gamma) * self.vx + \
            self.gamma * (-msg.transform.translation.x - last_x) / (msg.header.stamp.to_sec() - last_t)
        self.vy = (1 - self.gamma) * self.vy + \
            self.gamma * (-msg.transform.translation.y - last_y) / (msg.header.stamp.to_sec() - last_t)

        person.position.x = -msg.transform.translation.x
        person.position.y = -msg.transform.translation.y

        self.last_pos = -msg.transform.translation.x, -msg.transform.translation.y, msg.header.stamp.to_sec()
        people.people = [person]
        self.p_pub.publish(people)


if __name__ == '__main__':
    PersonControl().run()
