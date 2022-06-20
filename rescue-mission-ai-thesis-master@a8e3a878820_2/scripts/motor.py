#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from airsim_ros_pkgs.msg import VelCmd
from std_srvs.srv import SetBool, SetBoolResponse
import numpy as np
import airsim

class Motor:
    def __init__(self):
        rospy.init_node('motor', anonymous=False)
        self.id = rospy.get_namespace()
        self.id_name = self.id[1:-1]
        self.host = rospy.get_param('~host')
        self.altitude = rospy.get_param('~altitude', -11)
        self.enable = rospy.get_param('~enable', True)
        
        # Connect to AirSim API
        self.client = airsim.MultirotorClient(self.host)
        # Change trace color
        trace_color = rospy.get_param('trace_rgba', [0.1, 1.0, 0.0, 1.0])
        self.client.simSetTraceLine(trace_color, 40, self.id_name)
        rospy.loginfo("Takeoff")
        self.client.takeoffAsync(vehicle_name=self.id_name)
        rospy.loginfo("Reseting position and height...")
        self.client.moveToPositionAsync(0, 0, self.altitude, 5, vehicle_name=self.id_name).join()
        
        rospy.Service('enable_motor', SetBool, self.enable_motor_srv_cb)
        
        self.heading_sub = rospy.Subscriber('{}runaway/heading'.format(self.id), Twist, self.heading_cb, queue_size=10)
        self.pub = rospy.Publisher('/airsim_node{}vel_cmd_body_frame'.format(self.id), VelCmd, queue_size=10)

    def heading_cb(self, heading_msg):
        #velcmd = VelCmd()
        #velcmd.twist = heading_msg
        #self.pub.publish(velcmd)
        if self.enable:
            state = self.client.getMultirotorState(self.id_name)
            x = state.kinematics_estimated.position.x_val + heading_msg.linear.x
            y = state.kinematics_estimated.position.y_val + heading_msg.linear.y
            vel = np.linalg.norm([heading_msg.linear.x, heading_msg.linear.y, heading_msg.linear.z])
            self.client.moveToPositionAsync(x, y, self.altitude, vel, drivetrain=airsim.DrivetrainType.ForwardOnly, yaw_mode=airsim.YawMode(False, 0), vehicle_name=self.id_name, timeout_sec=1)
        #self.client.moveByVelocityAsync(heading_msg.linear.x, heading_msg.linear.y, 0, 0.4, drivetrain=airsim.DrivetrainType.ForwardOnly, yaw_mode=airsim.YawMode(False, 0))
    
    def enable_motor_srv_cb(self, msg):
        self.enable = msg.data
        if self.enable:
            return SetBoolResponse(True, 'Motor is enabled')
        return SetBoolResponse(True, 'Motor is disabled')

if __name__ == '__main__':
    try:
        motor = Motor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
