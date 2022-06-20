#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, Empty
from nav_msgs.msg import Odometry
from multi_drone_sar.msg import Task
from multi_drone_sar.srv import AddSingleTask, AddSingleTaskResponse
from actionlib_msgs.msg import GoalStatus
from std_srvs.srv import SetBool
import airsim

class Recharge:
    def __init__(self):
        rospy.init_node('recharge', anonymous=False)
        id = rospy.get_namespace()
        self.id = id[1:-1]
        queue_size = rospy.get_param('~queue_size', 10)
        self.battery_threshold = rospy.get_param('~battery_threshold', 15)
        self.priority = rospy.get_param('~task_priority', 1)
        offset_x = rospy.get_param('offset_x', 0)
        offset_y = rospy.get_param('offset_y', 0)
        self.charging = False
        self.home_pos = [offset_x, offset_y]
        self.inhib_pub = rospy.Publisher('~inhib', Empty, queue_size=queue_size)
        self.inhib_pub.publish(Empty())
        self.host = rospy.get_param('~host')
        self.client = airsim.MultirotorClient(self.host)
        rospy.Subscriber('current_charge_level', Float32, self.battery_status_cb, queue_size=queue_size)
        rospy.Subscriber('navigation/status', GoalStatus, self.task_status_cb, queue_size=queue_size)

    def battery_status_cb(self, status_msg):
        if status_msg.data < self.battery_threshold and not self.charging:
            rospy.wait_for_service('add_task', timeout=1000)
            task_msg = Task()
            task_msg.droneid = self.id
            task_msg.x = self.home_pos[0]
            task_msg.y = self.home_pos[1]
            task_msg.private = True
            task_msg.priority = self.priority
            task_msg.stamp = rospy.Time.now()
            task_msg.uniqueid = self.id + f'/recharge'
            try:
                add_task = rospy.ServiceProxy('add_task', AddSingleTask)
                response = add_task(task_msg, False)
                self.charging = True
            except rospy.ServiceException as e:
                rospy.logerr(f'Service add_task failed: {e}')
        elif status_msg.data >= 99:
            self.charging = False
            self.client.takeoffAsync(vehicle_name=self.id).join()
            self.call_enable_motor(True)
            self.call_enable_mission(True)
        if self.charging:
            self.inhib_pub.publish(Empty())

    def task_status_cb(self, status_msg):
        if f'{self.id}/recharge' == status_msg.goal_id.id and status_msg.status == GoalStatus.SUCCEEDED:
            response = self.call_enable_mission(False)
            if response and response.success:
                response = self.call_enable_motor(False)
                if response and response.success:
                    self.charging = True
                    rospy.sleep(2)
                    self.client.landAsync(vehicle_name=self.id).join()
    
    def call_enable_mission(self, enable):
        rospy.wait_for_service('start_mission', timeout=1000)
        try:
            start_mission = rospy.ServiceProxy('start_mission', SetBool)
            return start_mission(enable)
        except rospy.ServiceException as e:
            rospy.logerr(f'Service start_mission failed: {e}')
            return None
    
    def call_enable_motor(self, enable):
        rospy.wait_for_service('enable_motor')
        try:
            enable_motor_srv = rospy.ServiceProxy('enable_motor', SetBool)
            return enable_motor_srv(enable)
        except rospy.ServiceException as e:
            rospy.logerr(f'Service enable_motor failed: {e}')
            return None

if __name__ == '__main__':
    try:
        r = Recharge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
