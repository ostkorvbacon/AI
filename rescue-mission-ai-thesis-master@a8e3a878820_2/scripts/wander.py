#!/usr/bin/env python
import rospy
import random
import math
from geometry_msgs.msg import Twist

class Wander:
    def __init__(self):
        rospy.init_node('wander', anonymous=False)
        id = rospy.get_namespace()
        self.id = id
        self.velocity = rospy.get_param('~velocity', 5)
        self.heading_update_rate = rospy.get_param('~heading_update_rate', 10)
        self.pub = rospy.Publisher('~heading', Twist, queue_size=10)
        self.random_heading(None)

    def set_heading(self, heading, velocity):
        rospy.loginfo(f'{self.id} New heading: {heading}')
        self.x = math.cos(heading)*velocity
        self.y = math.sin(heading)*velocity

    def random_heading(self, event):
        heading = math.radians(random.randint(0, 360))
        self.set_heading(heading, self.velocity)

    def run_random_walk(self):
        output = Twist()
        rate = rospy.Rate(10)
        rospy.Timer(rospy.Duration(self.heading_update_rate), self.random_heading)
        while not rospy.is_shutdown():
            output.linear.x = 0#self.x
            output.linear.y = 0#self.y
            rospy.logdebug(output)
            self.pub.publish(output)
            rate.sleep()

if __name__ == '__main__':
    wand = Wander()
    try:
        wand.run_random_walk()
    except rospy.ROSInterruptException:
        pass
