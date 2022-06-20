#!/usr/bin/env python
import rospy
import random
import airsim
import math
from multi_drone_sar.msg import People
from multi_drone_sar.msg import PersonIdAndPosition
from types import SimpleNamespace
import numpy as np

class HelperFunctions:
    def __init__(self):
        rospy.init_node('helper_functions', anonymous=False)
        self.queue_size = rospy.get_param('~queue_size', 10)
        self.people_list = [SimpleNamespace(**person) for person in rospy.get_param('people')]
        self.people_found = 0
        self.people_missing = 0
        self.subscribed = []
        self.total_missing_people = len(self.people_list)
        self.people_missing = self.total_missing_people
        self.pt = rospy.Publisher('people_and_time', People, queue_size=self.queue_size)
        self.mission_start_time = rospy.Time.now()

    def publish_time_and_people(self):
        self.people_and_time = People()
        # calculate current time and convert to minute and seconds
        elapsed_time = rospy.Time.now() - self.mission_start_time
        elapsed_time_sec = elapsed_time.to_sec()
        m, s = divmod(elapsed_time_sec, 60)
        h, m = divmod(m, 60)
        # save values to self.people_and_time
        self.people_and_time.seconds = int(s)
        self.people_and_time.minutes = int(m)
        self.people_and_time.hours = int(h)
        self.people_and_time.total_people = self.total_missing_people
        self.people_and_time.people_found = self.people_found
        self.people_and_time.people_missing = self.people_missing
        self.pt.publish(self.people_and_time)

    def people_found_listener(self):
        topics = rospy.get_published_topics()
        for topic in topics:
            if '/found_person' in topic[0] and topic[0] not in self.subscribed:
                rospy.Subscriber(topic[0], PersonIdAndPosition, self.people_counter_cb, queue_size=self.queue_size)
                self.subscribed.append(topic[0])

    def people_counter_cb(self, people_msg):
        self.people_found = self.people_found + 1
        self.people_missing = self.people_missing - 1

    def main(self):
        while not rospy.is_shutdown():
            self.people_found_listener()
            self.publish_time_and_people()

if __name__ == '__main__':
    hp = HelperFunctions()
    try:
        hp.main()
    except rospy.ROSInterruptException:
        pass