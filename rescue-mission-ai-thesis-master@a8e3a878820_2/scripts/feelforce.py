#!/usr/bin/env python
import rospy
import ros_numpy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from multi_drone_sar.msg import DroneInfoArray
from tf.transformations import euler_from_quaternion, quaternion_about_axis
from visualization_msgs.msg import Marker
import copy

class Feelforce:
    def __init__(self):
        rospy.init_node('feelforce', anonymous=False)
        id = rospy.get_namespace()
        self.magnitude_threshold = rospy.get_param('magnitude_threshold', 5)
        self.max_lidar_range = rospy.get_param('max_lidar_range', 10)
        queue_size = rospy.get_param('queue_size', 10)
        self.yaw = 0
        self.my_position = [0, 0, 0]
        self.neighbours = []
        self.marker = Marker()
        self.marker.header.frame_id = id[:-1]+"/odom_local_ned"
        self.marker.type = Marker.ARROW
        self.marker.scale.x = 1
        self.marker.scale.y = 0.01
        self.marker.scale.z = 0.01
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker_pub = rospy.Publisher('~force_marker', Marker, queue_size=queue_size)
        self.pub = rospy.Publisher('~force', Twist, queue_size=queue_size)
        rospy.Subscriber(f'{id}neighbour_listener/neighbours', DroneInfoArray, self.neighbour_cb, queue_size=queue_size)
        rospy.Subscriber(f'/airsim_node{id}odom_local_ned', Odometry, self.odom_cb, queue_size=queue_size)
        rospy.Subscriber(f'/airsim_node{id}lidar/LidarCustom', PointCloud2, self.lidar_cb, queue_size=queue_size)

    def calc_average_force(self, lidar_msg):
        if len(lidar_msg.data) == 0:
            return Twist()
        data = ros_numpy.numpify(lidar_msg)
        points = np.zeros((data.shape[0], 3))
        points[:,0] = data['x']
        points[:,1] = data['y']
        points[:,2] = data['z']
        points = points[points[:,2] < 3] # Ignore floor level
        temp_points = copy.copy(points)
        points[:,0] = temp_points[:,0]*np.cos(self.yaw) - temp_points[:,1]*np.sin(self.yaw)
        points[:,1] = temp_points[:,0]*np.sin(self.yaw) + temp_points[:,1]*np.cos(self.yaw)
        if len(self.neighbours) > 0:
            np.append(points, self.neighbours*0.001, axis=0)
        elif len(points) == 0:
            return Twist()
        points[:,2] = 0
        magnitudes = np.linalg.norm(points, axis=1)[:,None]
        force_sum = np.sum(((-1)/(magnitudes**2)) * points, axis=0)/len(points)
        twist = Twist()
        twist.linear.x = force_sum[0]
        twist.linear.y = force_sum[1]
        twist.linear.z = force_sum[2]
        return twist
    
    def lidar_cb(self, lidar_msg):
        force = self.calc_average_force(lidar_msg)
        self.publish_force_marker(force)
        self.pub.publish(force)
        
    def odom_cb(self, odom_msg):
        pos = odom_msg.pose.pose.position
        self.my_position[:] = [pos.x, pos.y, pos.z]
        quart = odom_msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([quart.x, quart.y, quart.z, quart.w])
        self.yaw = yaw

    def neighbour_cb(self, neighbours):
        self.neighbours = np.array([])
        for neighbour in neighbours.info:
            np.append(self.neighbours, [neighbour.odometry.pose.pose.position.x, 
                                    neighbour.odometry.pose.pose.position.y, 
                                    neighbour.odometry.pose.pose.position.z])
    
    def publish_force_marker(self, force):
        temp_force = np.array([force.linear.x*np.cos(-self.yaw) - force.linear.y*np.sin(-self.yaw), force.linear.x*np.sin(-self.yaw) + force.linear.y*np.cos(-self.yaw)])
        mag = np.linalg.norm(temp_force)
        #rospy.loginfo(mag)
        angle = np.arctan2(temp_force[1], temp_force[0])
        quat = quaternion_about_axis(angle, (0, 0, 1))
        self.marker.pose.orientation.x  = quat[0]
        self.marker.pose.orientation.y = quat[1]
        self.marker.pose.orientation.z = quat[2]
        self.marker.pose.orientation.w = quat[3]
        self.marker.scale.x = mag
        self.marker.pose.position.x = 0#force.linear.x#self.my_position[0]
        self.marker.pose.position.y = 0#force.linear.y#self.my_position[1]
        self.marker.pose.position.z = 0#force.linear.z#self.my_position[2]
        self.marker.header.stamp = rospy.Time.now()
        self.marker_pub.publish(self.marker)

if __name__ == '__main__':
    try:
        ff = Feelforce()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
