#!/usr/bin/env python
from __future__ import print_function
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose
from people_msgs.msg import People, Person, PeoplePrediction
import tf
import time
from tf.transformations import quaternion_from_euler

import roslib
import rospy
import math

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class GoalPath(object):

    goal_pos = (0, 0)

    num_predictions = 20

    probability = .33333

    path_prediction = []

    time_resolution = .5

    def __init__(self, goal_pos=(0, 0), **kwargs):
        super(GoalPath, self).__init__(**kwargs)
        self.goal_pos = goal_pos

    def predict_path(self, person):
        path_prediction = self.path_prediction = []
        goal_x, goal_y = self.goal_pos
        vx, vy = person.velocity.x, person.velocity.y
        px, py = person.position.x, person.position.y
        goal_dist = ((goal_x - px) ** 2 + (goal_y - py) ** 2) ** 0.5

        speed = (vx **2 + vy ** 2) ** 0.5
        theta = math.atan2(goal_y - py, goal_x - px)
        step_dist = speed * self.time_resolution
        if not speed:
            max_prediction = 0
        else:
            max_prediction = min(self.num_predictions, math.ceil(goal_dist / step_dist))

        step_x, step_y = step_dist * math.cos(theta), step_dist * math.sin(theta)
        last_x, last_y = px, py
        for i in range(1, self.num_predictions + 1):
            person_prediction = Person()
            person_prediction.position.z = 0
            person_prediction.velocity = person.velocity
            path_prediction.append(person_prediction)

            if i >= max_prediction:
                person_prediction.position.x = last_x
                person_prediction.position.y = last_y
            else:
                last_x = person_prediction.position.x = px +  i * step_x
                last_y = person_prediction.position.y = py +  i * step_y


class PersonPath(object):

    goals = []

    people_sub = None

    prediction_pub = None

    marker_pub = None

    time_resolution = .5

    num_predictions = 20

    def start_node(self):
        rospy.init_node('person_path_prediction', anonymous=True)
        self.people_sub = rospy.Subscriber("people", People, self.people_callback)

        self.goals = [GoalPath(goal_pos=(1.23, 1.46)), GoalPath(goal_pos=(-6.36, 0.29)), GoalPath(goal_pos=(6.57, 1.25))]

        self.prediction_pub = rospy.Publisher("people_prediction", PeoplePrediction, queue_size=10)
        self.marker_pub = rospy.Publisher("prediction_viz", MarkerArray, queue_size=10)

    def get_marker(self):
        marker = Marker()
        marker.type = 2  # sphere
        marker.action = 0
        marker.ns = "predictor";
        marker.pose.orientation.w = 1
        marker.color.r = 0
        marker.color.g = 0
        marker.color.b = 0.5
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        return marker

    def people_callback(self, people):
        assert len(people.people) == 1
        person = people.people[0]
        predictions = PeoplePrediction()
        markers = MarkerArray()

        markers.markers = []
        predictions.predicted_people = []

        # if the message stamp is empty, we assign the current time
        if people.header.stamp == rospy.Time():
            people.header.stamp = rospy.get_rostime()

        predictions.path_probabilities = probabilities = []
        for goal in self.goals:
            goal.predict_path(person)
            probabilities.append(Float64())
            probabilities[-1].data = goal.probability

        for i in range(self.num_predictions):
            people_one_timestep = People()
            people_one_timestep.people = []
            for goal in self.goals[::-1]:
                person_prediction = goal.path_prediction[i]

                people_one_timestep.people.append(person_prediction)

                prediction_marker = self.get_marker()
                prediction_marker.header.frame_id = people.header.frame_id
                prediction_marker.header.stamp = people.header.stamp
                prediction_marker.id = i

                prediction_marker.pose.position = person_prediction.position
                # the opacity of the marker is adjusted according to the prediction step
                prediction_marker.color.a = 1 - i / float(self.num_predictions)

                markers.markers.append(prediction_marker)

            # fill the header
            people_one_timestep.header.frame_id = people.header.frame_id
            people_one_timestep.header.stamp = people.header.stamp + \
              rospy.Duration(i * self.time_resolution)
            # push back the predictions for this time step to the prediction container
            predictions.predicted_people.append(people_one_timestep)

        # print([marker.pose.position.x for marker in markers.markers])
        self.prediction_pub.publish(predictions)
        self.marker_pub.publish(markers)

    def run(self):
        self.start_node()
        rospy.spin()

if __name__ == '__main__':
    PersonPath().run()