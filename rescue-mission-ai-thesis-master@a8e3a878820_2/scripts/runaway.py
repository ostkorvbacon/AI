#!/usr/bin/env python
import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class Runaway:
    def __init__(self):
        rospy.init_node('runaway', anonymous=False)
        id = rospy.get_namespace()
        queue_size = rospy.get_param('queue_size', 10)
        self.threshold = 1/(rospy.get_param('~threshold', 1)**2)
        self.force_sub = rospy.Subscriber('{}feelforce/force'.format(id), Twist, self.force_cb, queue_size=queue_size)
        self.pub = rospy.Publisher('~heading', Twist, queue_size=queue_size)

    def force_cb(self, force_msg):
        force = np.array([force_msg.linear.x, force_msg.linear.y, force_msg.linear.z])
        force_mag = np.linalg.norm(force)
        if force_mag < self.threshold:
            return
        force /= force_mag
        force *= 5
        new_heading = Twist()
        new_heading.linear.x = force[0]
        new_heading.linear.y = force[1]
        new_heading.linear.z = force[2]
        self.pub.publish(new_heading)

if __name__ == '__main__':
    try:
        avoid = Runaway()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass