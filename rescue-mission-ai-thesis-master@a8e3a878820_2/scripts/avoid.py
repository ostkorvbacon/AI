#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

class Avoid:
    def __init__(self):
        rospy.init_node('avoid', anonymous=False)
        id = rospy.get_namespace()
        queue_size = rospy.get_param('queue_size', 10)
        self.velocity = rospy.get_param('velocity', 5)
        self.max_threshold = 1/(rospy.get_param('~max_threshold')**2)
        self.min_threshold = 1/(rospy.get_param('~min_threshold')**2)
        self.force = np.ones(2)
        self.obstacle_distance = 1
        self.state = 0
        rospy.Subscriber('{}feelforce/force'.format(id), Twist, self.force_cb, queue_size=queue_size)
        rospy.Subscriber('{}wander/heading'.format(id), Twist, self.heading_cb, queue_size=queue_size)
        self.pub = rospy.Publisher('~heading', Twist, queue_size=queue_size)
        
    def orthogonal_force_vector(self):
        ortho = np.array([self.force[1], -self.force[0]])
        norm = np.linalg.norm(ortho)
        if norm == 0:
            return ortho
        return ortho/np.linalg.norm(ortho)

    def heading_cb(self, heading_msg):
        heading = [heading_msg.linear.x, heading_msg.linear.y]
        velocity = np.linalg.norm(heading)
        ortho_wall = self.orthogonal_force_vector()*velocity
        new_heading = Twist()
        if np.dot(self.force, heading) > 0:
            self.state = 0
        elif self.obstacle_distance > self.min_threshold:
            return
        elif self.obstacle_distance < self.max_threshold:
            self.state = 0
        elif self.state == 0 and self.obstacle_distance > self.max_threshold and self.obstacle_distance < self.min_threshold:
            if np.dot(heading, ortho_wall) > 0:
                self.state = 1
            else:
                self.state = 2
        if self.state == 1:
            new_heading.linear.x = ortho_wall[0]
            new_heading.linear.y = ortho_wall[1]
            self.pub.publish(new_heading)
        elif self.state == 2:
            new_heading.linear.x = -ortho_wall[0]
            new_heading.linear.y = -ortho_wall[1]
            self.pub.publish(new_heading)
        else:
            self.pub.publish(heading_msg)
        rospy.logdebug(f'state:{self.state}')

    def force_cb(self, force_msg):
        self.force[0] = force_msg.linear.x
        self.force[1] = force_msg.linear.y
        self.obstacle_distance = np.linalg.norm(self.force)

if __name__ == '__main__':
    try:
        avoid = Avoid()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass