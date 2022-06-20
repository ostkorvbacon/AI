#!/usr/bin/env python
import rospy
import random
import airsim
import math
from multi_drone_sar.msg import People
from multi_drone_sar.srv import SetBatteryCharge, SetBatteryChargeResponse
from std_msgs.msg import Float32, Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from numpy import linalg as LA
import numpy as np
import datetime

class FakeDrone:
    def __init__(self):
        rospy.init_node('fake_drone', anonymous=False)
        id = rospy.get_namespace()
        self.id = id
        self.id_name = self.id[1:-1]
        self.host = rospy.get_param('~host')
        queue_size = rospy.get_param('~queue_size', 10)
        battery_charge_level_max = rospy.get_param('~battery_charge_level_max', 95)
        battery_charge_level_min = rospy.get_param('~battery_charge_level_min', 40)
        self.battery_full_charge_time = rospy.get_param('~battery_full_charge_time', 10)
        self.offset_x = rospy.get_param('offset_x', 0)
        self.offset_y = rospy.get_param('offset_y', 0)
        self.max_charge_dist = rospy.get_param('~max_charge_distance', 6)
        self.enable_heartbeat = True
        self.current_position = Twist()
        self.battery_charge = random.randint(battery_charge_level_min, battery_charge_level_max)
        self.battery_pub = rospy.Publisher('current_charge_level', Float32, queue_size=queue_size)
        self.heartbeat_pub = rospy.Publisher('heartbeat', Empty, queue_size=queue_size)
        self.service = rospy.Service('set_battery_level', SetBatteryCharge, self.set_battery_level_srv)
        rospy.Subscriber('/airsim_node{}odom_local_ned'.format(id), Odometry, self.odom_cb, queue_size=queue_size)
        self.client = airsim.MultirotorClient(self.host)
        self.client.confirmConnection()
        self.charging = False
        self.time_since_last_charge = 0
        self.battery_charge_publish = Float32()

    def set_battery_level_srv(self, srv):
        if srv.data >100:
            self.battery_charge = 100
        elif srv.data < 0:
            self.battery_charge = 0
        else:
            self.battery_charge = srv.data
        return SetBatteryChargeResponse(True, f'Battery charge set to {self.battery_charge}')

    def battery_drainage(self):
        self.battery_charge = self.battery_charge - 0.01
        self.battery_charge = max(self.battery_charge, 0)

    def run(self):
        rate = rospy.Rate(10)
        full_charge = 100
        time_elapsed = 0
        current_time = 0
        charing_start_time = 0
        landing = False
        rospy.sleep(20)
        while not rospy.is_shutdown():
            distance = LA.norm(np.array([self.current_position.linear.x, self.current_position.linear.y]) - np.array([self.offset_x,self.offset_y]))
            #rospy.loginfo(landed)
            if distance < self.max_charge_dist and self.current_position.linear.z > 0.55 and not self.charging and self.battery_charge < 95:
                self.charging = True
                charging_start_time = rospy.Time.now()
            if self.charging:
                current_time = rospy.Time.now()
                time_elapsed = current_time - charging_start_time
                time_elapsed = time_elapsed.to_sec()
                if self.time_since_last_charge is not 0:
                    time_elapsed = abs(current_time.to_sec() - self.time_since_last_charge)
                self.battery_charge = self.battery_charge + full_charge/(self.battery_full_charge_time/time_elapsed)
                self.time_since_last_charge = current_time.to_sec()
            if self.battery_charge >= 100:
                self.battery_charge = 100
                charging_start_time = 0
                current_time = 0
                time_elapsed = 0
                self.time_since_last_charge = 0
                self.charging = False
                landing = False
                self.enable_heartbeat = True
                rospy.loginfo(f'{self.id_name} Battery full, takes off')
            self.battery_charge_publish.data = self.battery_charge
            rospy.logdebug(self.battery_charge)
            self.battery_pub.publish(self.battery_charge_publish)
            if self.enable_heartbeat:
                self.heartbeat_pub.publish(Empty())
            #rospy.loginfo(f'time:{time_elapsed}, z_dist:{self.current_position.linear.z}, charging is: {self.charging}, batt:{self.battery_charge}')
            if not self.charging:
                self.battery_drainage()
            if self.battery_charge is 0 and not landing:
                self.enable_motor(False)
                rospy.sleep(5)
                self.client.landAsync(vehicle_name=self.id_name)
                landing = True
                self.enable_heartbeat = False
                rospy.logerr(f'{self.id_name} Battery dead, crashes')
            rate.sleep()

    def enable_motor(self, enable):
        rospy.wait_for_service('enable_motor')
        try:
            enable_motor_srv = rospy.ServiceProxy('enable_motor', SetBool)
            enable_motor_srv(enable)
        except rospy.ServiceException as e:
            rospy.logerr(f'Service enable_motor failed: {e}')
    
    def odom_cb(self, odom_msg):
        self.current_position.linear.x = odom_msg.pose.pose.position.x + self.offset_x
        self.current_position.linear.y = odom_msg.pose.pose.position.y + self.offset_y
        self.current_position.linear.z = odom_msg.pose.pose.position.z

if __name__ == '__main__':
    drone = FakeDrone()
    try:
        drone.run()
    except rospy.ROSInterruptException:
        pass
    