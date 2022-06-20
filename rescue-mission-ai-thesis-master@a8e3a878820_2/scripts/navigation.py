#!/usr/bin/env python
import rospy
import random
import math
import numpy as np
from numpy import linalg as LA
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseActionGoal

class Waypoint:
    
    def __init__(self, id, x, y, z, threshold=10):
        self.id = id
        self.x = x
        self.y = y
        self.z = z
        self.threshold = threshold

class Navigation:
    def __init__(self):
        rospy.init_node('navigation', anonymous=False)
        id = rospy.get_namespace()
        self.id = id[1:-1]
        self.waypoints = [Waypoint(**wp) for wp in rospy.get_param('~waypoints', [])]
        self.velocity = rospy.get_param('velocity', 5)
        self.offset_x = rospy.get_param('offset_x', 0)
        self.offset_y = rospy.get_param('offset_y', 0)
        queue_size = rospy.get_param('~queue_size', 10)
        self.heading = Twist()
        self.current = Twist()
        if self.waypoints:
            self.desired = self.waypoints.pop(0)
        else:
            self.desired = None
        self.goal_threshold = rospy.get_param('~goal_threshold', 10)
        self.pub_heading = rospy.Publisher('~heading', Twist, queue_size=queue_size)
        self.pub_current_waypoint = rospy.Publisher('~current_waypoint', MoveBaseActionGoal, queue_size=queue_size)
        self.pub_status = rospy.Publisher('~status', GoalStatus, queue_size=queue_size)
        rospy.Subscriber('~goal', MoveBaseActionGoal, self.waypoint_cb, queue_size=queue_size)
        rospy.Subscriber('/airsim_node{}odom_local_ned'.format(id), Odometry, self.odom_cb, queue_size=queue_size)
        
        
    
    def waypoint_cb(self, waypoint_msg):
        threshold = waypoint_msg.goal.target_pose.pose.position.z
        self.desired = Waypoint(waypoint_msg.goal_id.id,
                                waypoint_msg.goal.target_pose.pose.position.x,
                                waypoint_msg.goal.target_pose.pose.position.y,
                                0)
        if threshold:
            self.desired.threshold = threshold
        rospy.loginfo_throttle_identical(99999999999999, f'{self.id} Got waypoint: {self.desired.id}, x:{self.desired.x}, y:{self.desired.y}')

    def calculate_heading(self):
        # calculate heading by aligning a vector between current position and desired position and setting the correct velocity
        self.heading.linear.x = self.desired.x - self.current.linear.x
        self.heading.linear.y = self.desired.y - self.current.linear.y
        temp_vector = np.array([self.heading.linear.x, self.heading.linear.y])
        temp_vector = temp_vector / LA.norm(temp_vector) * self.velocity
        self.heading.linear.x = temp_vector[0]
        self.heading.linear.y = temp_vector[1]

    def check_reached_goal(self):
        # is current position within threshold?
        current_position = np.array([self.current.linear.x, self.current.linear.y])
        desired_position = np.array([self.desired.x, self.desired.y])
        return LA.norm(desired_position-current_position) < self.desired.threshold
    
    def publish_status(self, goal_id, status):
        status_msg = GoalStatus()
        status_msg.goal_id.id = goal_id
        status_msg.status = status
        self.pub_status.publish(status_msg)

    def navigate(self):        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self.desired:
                rate.sleep()
                continue
            
            if self.check_reached_goal():
                self.publish_status(self.desired.id, GoalStatus.SUCCEEDED)
                rospy.loginfo(f'{self.id} Waypoint reached! {self.desired.id}, x:{self.desired.x}, y:{self.desired.y}')
                self.desired = None
                if len(self.waypoints) is not 0:
                    self.desired = self.waypoints.pop(0)
                continue
            self.publish_status(self.desired.id, GoalStatus.ACTIVE)
            self.calculate_heading()
            self.pub_heading.publish(self.heading)
            goal = MoveBaseActionGoal()
            goal.goal.target_pose.pose.position.x = self.desired.x
            goal.goal.target_pose.pose.position.y = self.desired.y
            goal.goal_id.id = self.desired.id
            self.pub_current_waypoint.publish(goal)
            rate.sleep()

    def odom_cb(self, odom_msg):
        self.current.linear.x = odom_msg.pose.pose.position.x + self.offset_x
        self.current.linear.y = odom_msg.pose.pose.position.y + self.offset_y

if __name__ == '__main__':
    navi = Navigation()
    try:
        navi.navigate()
    except rospy.ROSInterruptException:
        pass