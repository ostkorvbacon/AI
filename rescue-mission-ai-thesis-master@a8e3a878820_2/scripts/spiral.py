#!/usr/bin/env python
import rospy 
import airsim
import os
import sys
import math
import time
import argparse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from multi_drone_sar.msg import PersonIdAndPosition
import numpy as np
from numpy import linalg as LA
class Spiral():
    def __init__(self):
        rospy.init_node('~spiral', anonymous=False)
        id = rospy.get_namespace()
        self.id = id
        queue_size = rospy.get_param('~queue_size', 10)
        self.radius = rospy.get_param('~radius', 2)
        self.velocity = rospy.get_param('~velocity', 5)
        self.offset_x = rospy.get_param('offset_x', 0)
        self.offset_y = rospy.get_param('offset_y', 0)
        self.spiral_size = rospy.get_param('~spiral_size', 200)
        self.spiral_gap = rospy.get_param('~spiral_gap', 5)
        self.iterations = rospy.get_param('~iterations', 1)
        self.timer_update_rate = rospy.get_param('~timer_update_rate', 0.1)
        self.plot_spiral = rospy.get_param('~plot_spiral', False)
        self.pub_heading = rospy.Publisher('~heading', Twist, queue_size=queue_size)
        self.heading = Twist()
        self.running = False
        self.current_waypoint = []
        self.counter = 0
        self.vt = [0] * self.spiral_size
        self.x = [0] * self.spiral_size
        self.y = [0] * self.spiral_size
        self.current_position = Twist()
        self.spiral_creation()
        rospy.Subscriber('/airsim_node{}odom_local_ned'.format(id), Odometry, self.odom_cb, queue_size=queue_size)
        rospy.Subscriber('{}multiple_people/person_found'.format(id), PersonIdAndPosition, self.activate_cb, queue_size=queue_size)

    def activate_cb(self, data):
        if not self.running:
            self.center = [data.x, data.y]
            self.running = True
            self.last_waypoint = [self.x[-1] + self.center[0], self.y[-1] + self.center[1]]
            self.timer = rospy.Timer(rospy.Duration(self.timer_update_rate), self.spiral)

    def spiral_creation(self):
        for i in range(self.spiral_size):
            self.vt[i] = i / 57 * math.pi
            self.x[i] = (self.vt[i] * self.spiral_gap) * math.cos(self.vt[i])
            self.y[i] = (self.vt[i] * self.spiral_gap) * math.sin(self.vt[i])
        if self.plot_spiral:
            from matplotlib import pyplot as plt
            plt.plot(self.x, self.y)
            plt.ion()
            plt.show()

    def spiral(self, event):
        # do spiral stuff
        if self.counter == 0:
            self.last_time = rospy.Time.now()
        if self.counter < self.spiral_size:
            self.current_waypoint = [self.x[self.counter]+self.center[0], self.y[self.counter]+self.center[1]]
            self.calculate_heading()
        distance = LA.norm(np.array([self.current_position.linear.x, self.current_position.linear.y]) - np.array([self.current_waypoint[0],self.current_waypoint[1]]))
        elapsed_time = (rospy.Time.now() - self.last_time).to_sec()
        if elapsed_time > 2:
            rospy.logwarn(f'Time to get to spiral wp exceeded threshold. elapsed_time:{elapsed_time}')
        if (distance < 3 and self.counter < self.spiral_size) or elapsed_time > 2:
            self.counter = self.counter + 1
            self.last_time = rospy.Time.now()
        if self.counter is self.spiral_size:
            self.running = False
            self.timer.shutdown()
            self.counter = 0
            return
        self.pub_heading.publish(self.heading)

    def calculate_heading(self):
        # calculate heading by aligning a vector between current position and desired position and setting the correct velocity
        self.heading.linear.x = self.current_waypoint[0] - self.current_position.linear.x
        self.heading.linear.y = self.current_waypoint[1] - self.current_position.linear.y
        temp_vector = np.array([self.heading.linear.x, self.heading.linear.y])
        temp_vector = temp_vector / LA.norm(temp_vector) * self.velocity
        self.heading.linear.x = temp_vector[0]
        self.heading.linear.y = temp_vector[1]

    def odom_cb(self, odom_msg):
        self.current_position.linear.x = odom_msg.pose.pose.position.x + self.offset_x
        self.current_position.linear.y = odom_msg.pose.pose.position.y + self.offset_y

if __name__ == '__main__':
    try:
        sp = Spiral()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass