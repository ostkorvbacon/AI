#!/usr/bin/env python
import rospy
import ros_numpy
import math
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class Explore:
    def __init__(self):
        rospy.init_node('explore', anonymous=False)
        id = rospy.get_namespace()
        self.max_lidar_range = rospy.get_param('max_lidar_range', 15)
        self.lidar_sub = rospy.Subscriber('/airsim_node{}lidar/LidarCustom'.format(id), PointCloud2, self.lidar_cb, queue_size=10)
        self.tf_sub = rospy.Subscriber('/airsim_node{}odom_local_ned'.format(id), Odometry, self.tf_cb, queue_size=10)
        self.pub = rospy.Publisher('~heading', Twist, queue_size=10)
        self.paths = np.zeros(360)
        self.tf_msg = Odometry()

    def calc_exploration_heading(self, lidar_msg):
        if len(lidar_msg.data) == 0:
            twist = Twist()
            twist.linear.x = self.max_lidar_range
            return twist
        data = ros_numpy.numpify(lidar_msg)
        points = np.zeros((data.shape[0], 3))
        points[:,0] = data['x']
        points[:,1] = data['y']
        points[:,2] = data['z']
        self.paths = np.zeros(360)
        for point in points:
            deg = round(math.degrees(math.atan2(point[1], point[0]) + 2*math.pi*(point[1]<0)))%360
            dist = np.linalg.norm(point[:2])
            if dist < 25:
                self.paths[deg] = 1
        path = self.get_open_path()
        print(path)
        if path is not None:
            output = Twist()
            output.linear.x = 5*math.cos(math.radians(path))
            output.linear.y = 5*math.sin(math.radians(path))
            self.pub.publish(output)

    def get_open_path(self):
        orient = self.tf_msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        crisp_yaw = round(yaw)
        for i in range(0, 360):
            path_found = 0
            for k in range(-10, 11):
                if self.paths[abs(crisp_yaw+k)%360] == 1:
                    path_found = 0
                    break
                path_found = 1
            if path_found:
                return crisp_yaw
            crisp_yaw += 1
        return None

    def lidar_cb(self, lidar_msg):
        self.calc_exploration_heading(lidar_msg)

    def tf_cb(self, tf_msg):
        self.tf_msg = tf_msg

if __name__ == '__main__':
    try:
        ex = Explore()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass