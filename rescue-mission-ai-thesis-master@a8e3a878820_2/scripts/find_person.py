#!/usr/bin/env python
import rospy
import random
import math
import numpy as np
from types import SimpleNamespace
from numpy import linalg as LA
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from multi_drone_sar.msg import PersonIdAndPosition

class FindPeople:
    def __init__(self):
        rospy.init_node('find_people', anonymous=False)
        id = rospy.get_namespace()
        self.id = id
        self.people = [SimpleNamespace(**person) for person in rospy.get_param('people')]
        self.offset_x = rospy.get_param('~offset_x', 0)
        self.offset_y = rospy.get_param('~offset_y', 0)
        self.subscribed = []
        queue_size = rospy.get_param('~queue_size', 10)
        self.proximity_threshold = rospy.get_param('~proximity_threshold', 15)
        self.pub_found_people = rospy.Publisher('~found_person', PersonIdAndPosition, queue_size=queue_size)
        rospy.Subscriber('/airsim_node{}odom_local_ned'.format(id), Odometry, self.odom_cb, queue_size=queue_size)
        self.current = Twist()

    def get_people_within_radius(self):
        # Check if any of the people in the list are within a certain radius of the drone. return a list with the people found.
        current_position = np.array([self.current.linear.x, self.current.linear.y])
        found_people = []
        for person in self.people:
            if person.found is True:
                continue
            person_position = np.array([person.point[0], person.point[1]])
            if LA.norm(current_position - person_position) < self.proximity_threshold:
                found_people.append(person)
                person.found = True
        return found_people

    def people_finder(self):
        # call get_people_within_radius if empty no people are found. If multiple people are found loop the length of the list and publish each separately.
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            detected_person = PersonIdAndPosition()
            self.people_found_listener()
            found_people = self.get_people_within_radius()
            if found_people:
                for person in found_people:
                    rospy.loginfo("%s Has found a Person! ID: %d, x: %f, y: %f, first_aid: %r", self.id, person.id, person.point[0], person.point[1], person.first_aid)
                    detected_person.id = person.id
                    detected_person.x = person.point[0]
                    detected_person.y = person.point[1]
                    detected_person.first_aid = person.first_aid
                    self.pub_found_people.publish(detected_person)
            rate.sleep()

    def people_found_listener(self):
        topics = rospy.get_published_topics()
        for topic in topics:
            if '/found_person' in topic[0] and self.id not in topic[0] and topic[0] not in self.subscribed:
                rospy.Subscriber(topic[0], PersonIdAndPosition, self.people_cb, queue_size=10)
                self.subscribed.append(topic[0])

    def people_cb(self, people_msg):
        # Receives a message containing the ID of a person that has been found. Then it changes that ID in the list of people to found by setting a boolean to true.
        for person in self.people:
            if person.id is people_msg.id and person.found is not True:
                person.found = True
                # rospy.loginfo(self.id + " Has receieved a found person and removed them from their list!")
                break

    def odom_cb(self, odom_msg):
        self.current.linear.x = odom_msg.pose.pose.position.x + self.offset_x
        self.current.linear.y = odom_msg.pose.pose.position.y + self.offset_y

if __name__ == '__main__':
    fp = FindPeople()
    try:
        fp.people_finder()
    except rospy.ROSInterruptException:
        pass