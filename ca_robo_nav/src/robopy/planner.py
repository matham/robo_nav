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

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class GoalPath(object):

    goal_pos = (0, 0)

    num_predictions = 20

    probability = .33333

    path_prediction = []

    time_resolution = .5

    offset = 0

    def __init__(self, offset=0, **kwargs):
        super(GoalPath, self).__init__(**kwargs)
        self.offset = offset

    def predict_path(self, person):
        path_prediction = self.path_prediction = []
        for i in range(self.num_predictions):
            person_prediction = Person()
            path_prediction.append(person_prediction)

            person_prediction.position.x = person.position.x + \
                (i * self.time_resolution * person.velocity.x) + self.offset
            person_prediction.position.y = person.position.y + \
                (i * self.time_resolution * person.velocity.y)
            person_prediction.position.z = person.position.z + \
                (i * self.time_resolution * person.velocity.z)

            # the velocity stays the same
            person_prediction.velocity = person.velocity


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

        self.goals = [GoalPath(offset=0), GoalPath(offset=5), GoalPath(offset=10)]

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
            for goal in self.goals:
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

        self.prediction_pub.publish(predictions)
        self.marker_pub.publish(markers)

    def run(self):
        self.start_node()
        rospy.spin()

if __name__ == '__main__':
    PersonPath().run()