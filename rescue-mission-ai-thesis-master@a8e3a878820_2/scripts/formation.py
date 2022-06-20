#!/usr/bin/env python
import rospy
import random
import math
import numpy as np
from numpy import linalg as LA
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from multi_drone_sar.msg import DroneInfoArray

class Formation:
    def __init__(self):
        rospy.init_node('formation', anonymous=False)
        id = rospy.get_namespace()
        queue_size = rospy.get_param('~queue_size', 10)
        self.id = id
        self.velocity = rospy.get_param('~velocity', 5)
        self.group_radius = rospy.get_param('~group_radius', 30)
        self.neighbours = DroneInfoArray()
        self.my_odometry = Odometry()
        self.heading = Twist()
        self.current_waypoint = None
        self.count = 0
        self.pub = rospy.Publisher('~heading', Twist, queue_size=10)
        rospy.Subscriber('neighbour_listener/neighbours', DroneInfoArray, self.neighbour_odom_cb, queue_size=queue_size)
        rospy.Subscriber(f'{self.id}navigation/current_waypoint', MoveBaseActionGoal, self.waypoint_cb, queue_size=queue_size)
        rospy.Subscriber(f'/airsim_node{self.id}odom_local_ned', Odometry, self.my_odom_cb, queue_size=queue_size)

    def neighbour_odom_cb(self, odom_msg):
        self.neighbours = odom_msg

    def waypoint_cb(self, wpid_msg):
        self.current_waypoint = [wpid_msg.goal.target_pose.pose.position.x, wpid_msg.goal.target_pose.pose.position.y]

    def my_odom_cb(self, odom_msg):
        self.my_odometry = odom_msg

    def is_same_wp_as_current(self, wp):
        if wp.goal.target_pose.pose.position.x == self.current_waypoint[0] and wp.goal.target_pose.pose.position.y == self.current_waypoint[1]:
            return True
        return False

    def separation(self):
        # steer to avoid crowding local flockmates
        self.avoidance_vector = np.zeros(3, np.float)
        counter = 0
        for neighbour in self.neighbours.info:
            distance = LA.norm(np.array([neighbour.odometry.pose.pose.position.x, neighbour.odometry.pose.pose.position.y]))
            if distance < self.group_radius and self.is_same_wp_as_current(neighbour.wp) and distance is not 0 and distance < 20:
                diff = - np.array([neighbour.odometry.pose.pose.position.x, neighbour.odometry.pose.pose.position.y, 0])
                diff /= distance
                self.avoidance_vector += diff
                counter += 1
        if counter > 0:
            self.avoidance_vector /= counter
            avoidance_vector_norm = LA.norm(self.avoidance_vector)
            if avoidance_vector_norm > 0:
                self.avoidance_vector = (self.avoidance_vector / avoidance_vector_norm) * (self.velocity)

    def alignment(self):
        #steer towards the average heading of local flockmates
        self.avg_heading = np.zeros(3,np.float)
        self.count = 0
        for neighbour in self.neighbours.info:
            distance = LA.norm(np.array([neighbour.odometry.pose.pose.position.x, neighbour.odometry.pose.pose.position.y]))
            if distance < self.group_radius and self.is_same_wp_as_current(neighbour.wp) and distance is not 0:
                self.avg_heading = self.avg_heading + np.array([neighbour.course.linear.x, neighbour.course.linear.y, 0])
                self.count += 1
        self.avg_heading = self.avg_heading + np.array([self.my_odometry.twist.twist.linear.x, self.my_odometry.twist.twist.linear.y, 0])
        self.count += 1
        if self.count > 0:
            self.avg_heading /= self.count
            avg_heading_norm = LA.norm(self.avg_heading)
            #if avg_heading_norm > 0:
            #    self.avg_heading = (self.avg_heading / avg_heading_norm) * self.velocity
            #    self.avg_heading = self.avg_heading - np.array([self.my_odometry.twist.twist.linear.x, self.my_odometry.twist.twist.linear.y, 0])
            #    avg_heading_norm = LA.norm(self.avg_heading)
            #    self.avg_heading = (self.avg_heading / avg_heading_norm) * (self.velocity)

    def cohesion(self):
        #  steer to move toward the average position of local flockmates
        self.center_of_mass = np.zeros(3,np.float)
        counter = 0
        for neighbour in self.neighbours.info:
            distance = LA.norm(np.array([neighbour.odometry.pose.pose.position.x, neighbour.odometry.pose.pose.position.y]))
            if distance < self.group_radius and self.is_same_wp_as_current(neighbour.wp) and distance is not 0:
                self.center_of_mass = self.center_of_mass + np.array([neighbour.odometry.pose.pose.position.x, neighbour.odometry.pose.pose.position.y, 0])
                counter += 1
        counter += 1
        if counter > 0:
            self.center_of_mass /= counter
            #center_of_mass_norm = LA.norm(self.center_of_mass)
            #if center_of_mass_norm > 0:
            #    self.center_of_mass = (self.center_of_mass / center_of_mass_norm) * (self.velocity)

    def formation_control(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.current_waypoint:
                current_direction = np.zeros(3,np.float)
                self.alignment()
                self.cohesion()
                self.separation()
                if self.count > 1:
                    current_heading = (self.avg_heading*1) + (self.center_of_mass*0.5) + (self.avoidance_vector*1)
                    current_heading_norm = LA.norm(current_heading)
                    if current_heading_norm > 0:
                        if current_heading_norm > self.velocity:
                            current_heading = (current_heading / current_heading_norm) * self.velocity
                        self.heading.linear.x = current_heading[0]
                        self.heading.linear.y = current_heading[1]
                        self.heading.linear.z = current_heading[2]
                        self.pub.publish(self.heading)
            rate.sleep()

if __name__ == '__main__':
    form = Formation()
    try:
        form.formation_control()
    except rospy.ROSInterruptException:
        pass
