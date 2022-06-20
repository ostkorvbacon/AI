#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from multi_drone_sar.msg import DroneInfo
from multi_drone_sar.msg import DroneInfoArray
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseActionGoal
import copy

class NeighbourListener:
    def __init__(self):
        rospy.init_node('neighbour_listener', anonymous=False)
        self.id = rospy.get_namespace()
        self.queue_size = rospy.get_param('~queue_size', 10)
        self.pub = rospy.Publisher('~neighbours', DroneInfoArray, queue_size=self.queue_size)
        self.neighbours_msg = DroneInfoArray()
        self.my_odom = Odometry()
        self.subscribed = []
        self.offset_x = rospy.get_param(f'{self.id}offset_x')
        self.offset_y = rospy.get_param(f'{self.id}offset_y')
        rospy.Subscriber(f'/airsim_node{self.id}odom_local_ned', Odometry, self.my_odom_cb, queue_size=self.queue_size)

    def neighbour_odom_cb(self, odom_msg, args):
        current_neighbour_id = args[0]
        odom_msg.pose.pose.position.x = (odom_msg.pose.pose.position.x + rospy.get_param(f'/{odom_msg.header.frame_id}/offset_x')) - self.my_odom.pose.pose.position.x
        odom_msg.pose.pose.position.y = (odom_msg.pose.pose.position.y + rospy.get_param(f'/{odom_msg.header.frame_id}/offset_y')) - self.my_odom.pose.pose.position.y
        odom_msg.pose.pose.position.z -= self.my_odom.pose.pose.position.z
        if not self.update_neighbour(current_neighbour_id, odom=odom_msg):
            self.create_neighbour(current_neighbour_id, odom=odom_msg)

    def neighbour_course_cb(self, course_msg, args):
        current_neighbour_id = args[0]
        if not self.update_neighbour(current_neighbour_id, course=course_msg):
            self.create_neighbour(current_neighbour_id, course=course_msg)

    def waypoint_cb(self, wp_msg, args):
        current_neighbour_id = args[0]
        if not self.update_neighbour(current_neighbour_id, wp=wp_msg):
            self.create_neighbour(current_neighbour_id, wp=wp_msg)

    def my_odom_cb(self, odom_msg):
        odom_msg.pose.pose.position.x += self.offset_x
        odom_msg.pose.pose.position.y += self.offset_y
        self.my_odom = odom_msg

    def create_neighbour(self, id, odom=None, course=None, wp=None):
        neighbour = DroneInfo()
        neighbour.id = id
        if odom is not None:
            neighbour.odometry = odom
        if course is not None:
            neighbour.course = course
        if wp is not None:
            neighbour.wp = wp
        self.neighbours_msg.info.append(neighbour)

    def update_neighbour(self, neighbour_id, odom=None, course=None, wp=None):
        for i, neighbour in enumerate(self.neighbours_msg.info):
            if neighbour_id == neighbour.id:
                if odom is not None:
                    self.neighbours_msg.info[i].odometry = odom
                if course is not None:
                    self.neighbours_msg.info[i].course = course
                if wp is not None:
                    self.neighbours_msg.info[i].wp = wp
                return True
        return False

    def publish_neighbours(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.find_topics()
            self.pub.publish(self.neighbours_msg)
            rate.sleep()

    def find_topics(self):
        topics = rospy.get_published_topics()
        for topic in topics:
            if '/odom_local_ned' in topic[0] and self.id not in topic[0] and topic[0] not in self.subscribed:
                rospy.Subscriber(topic[0], Odometry, self.neighbour_odom_cb, queue_size=self.queue_size, callback_args=[topic[0].split('/')[2]])
                self.subscribed.append(topic[0])
            elif 'navigation/heading' in topic[0] and self.id not in topic[0] and topic[0] not in self.subscribed:
                rospy.Subscriber(topic[0], Twist, self.neighbour_course_cb, queue_size=self.queue_size, callback_args=[topic[0].split('/')[1]])
                self.subscribed.append(topic[0])
            elif 'navigation/current_waypoint' in topic[0] and self.id not in topic[0] and topic[0] not in self.subscribed:
                rospy.Subscriber(topic[0], MoveBaseActionGoal, self.waypoint_cb, queue_size=self.queue_size, callback_args=[topic[0].split('/')[1]])
                self.subscribed.append(topic[0])

if __name__ == '__main__':
    try:
        neighbours = NeighbourListener()
        neighbours.publish_neighbours()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
