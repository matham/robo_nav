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

from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""


moveBindings = {
        'i':(1,0,0,0),
        'o':(1,0,0,-1),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'u':(1,0,0,1),
        ',':(-1,0,0,0),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1),
        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

def getKey(settings, timeout):
    tty.setraw(sys.stdin.fileno())
    res, _, _ = select.select([sys.stdin], [], [], timeout)
    if not res:
        return None
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)


class PersonControl(object):

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        x = y = z = th = 0
        speed = 0.25
        status = 0
        turn = 1.

        people_marker = Marker()
        people_marker.header.frame_id = "map"
        people_marker.ns = "model"
        people_marker.type = 10
        people_marker.action = 0
        people_marker.mesh_resource = "package://social_nav_simulation/gazebo/models/human/meshes/walking.dae";
        people_marker.mesh_use_embedded_materials = True
        people_marker.scale.x = 1.0
        people_marker.scale.y = 1.0
        people_marker.scale.z = 1.0

        rospy.init_node('fake_people_detection', anonymous=True)
        p_pub = rospy.Publisher('people', People, queue_size=1)
        viz_pub = rospy.Publisher('people_viz', Marker, queue_size=1)
        people = People()
        person = Person()

        person.position.x = 0
        person.position.y = -5.

        s = rospy.get_time()
        while True:
            key = getKey(settings, .05)
            z = th = 0
            x = y = 0
            if key == 'i':
                y = 1
            elif key == 'k':
                y = -1
            elif key == 'j':
                x = -1
            elif key == 'l':
                x = 1
            elif (key == '\x03'):
                break

            delta_t = .05
            people.header.stamp = rospy.get_rostime()
            s = rospy.get_time()
            person.position.x += delta_t * person.velocity.x
            person.position.y += delta_t * person.velocity.y
            person.velocity.x = x*speed
            person.velocity.y = y*speed

            people_marker.pose.position.x = person.position.x
            people_marker.pose.position.y = person.position.y
            people.header.frame_id = "map"
            people_marker.pose.orientation = quaternion_from_euler(0, 0, th*turn)
            people.people = [person]
            p_pub.publish(people)
            #viz_pub.publish(people_marker)


if __name__ == '__main__':
    PersonControl().run()
