#!/usr/bin/env python
import rospy
import numpy as np
import itertools
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion
from multi_drone_sar.msg import Task, TaskBid, TaskArray
from multi_drone_sar.srv import AddSingleTask, AddSingleTaskResponse
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseActionGoal
from std_srvs.srv import SetBool, SetBoolResponse
from nav_msgs.msg import Odometry
from threading import Lock

class TaskAllocation():
    def __init__(self):
        rospy.init_node('task_allocation', anonymous=False)
        id = rospy.get_namespace()
        self.queue_size = rospy.get_param('~queue_size', 10)
        self.id = id[1:-1]
        self.available_tasks = []
        self.my_tasks = []
        self.received_bids = []
        self.offset_x = rospy.get_param('offset_x', 0)
        self.offset_y = rospy.get_param('offset_y', 0)
        self.bid_tolerance = rospy.get_param('bid_tolerance', 5)
        self.bid_patience = rospy.get_param('bid_patience', 1)
        self.accept_patience = rospy.get_param('accept_patience', 1)
        self.current_position = np.zeros(2, np.float)
        self.yaw = 0
        self.subscribed = []
        self.current_task = None
        self.start_mission = False
        self.mutex = Lock()
        
        self.accept_task_pub = rospy.Publisher('~accept_task', Task, queue_size=self.queue_size)        # initialize publisher for accepting bids
        self.bid_pub = rospy.Publisher('~bid', TaskBid, queue_size=self.queue_size)        # initialize publisher for sending bids
        self.my_tasks_pub = rospy.Publisher('my_tasks', TaskArray, queue_size=self.queue_size)  
        self.current_task_pub = rospy.Publisher('navigation/goal', MoveBaseActionGoal, queue_size=self.queue_size)         # initialize publisher for sending the task further into navigation?
        self.task_complete_pub = rospy.Publisher('~task_complete', Task, queue_size=self.queue_size)
        
        rospy.Service('start_mission', SetBool, self.start_mission_srv_cb)
        rospy.Service('add_task', AddSingleTask, self.add_task_srv_cb)
        rospy.Subscriber('navigation/status', GoalStatus, self.task_status_cb, queue_size=self.queue_size)     # subscribe to a topic so we know if we are currently working on a task
        rospy.Subscriber(f'/airsim_node{id}odom_local_ned', Odometry, self.odom_cb, queue_size=self.queue_size)
        rospy.Subscriber('/task_manager/available', TaskArray, self.available_tasks_cb, queue_size=self.queue_size)        # subscribe to a topic to receive tasks
        # add publisher that notifies task manager that you want to change a task. (Should publish the task you want to change to in the available list) name: task_allocation/change_task

    def calculate_task_bid(self, task):
        # Calculate the bid based on your distance to the task
        # TODO: add the battery charge to the bid in some smart way
        x = task.x - self.current_position[0]
        y = task.y - self.current_position[1]
        dist = LA.norm([x, y])
        angle = np.degrees(np.arctan2(y, x) - self.yaw)
        if angle > 180:
            angle -= 360
        elif angle < -180:
            angle += 360
        return dist + abs(angle)*0.2

    def get_min_bid_on_task(self, ignore):
        min_task = None
        min_bid = None
        min_prio = self.available_tasks[0].priority
        for task in self.available_tasks:
            if min_prio < task.priority:
                break
            elif task.uniqueid in ignore:
                continue
            bid = self.calculate_task_bid(task)
            if min_bid is None or bid < min_bid:
                min_bid = bid
                min_task = task
        return min_task, min_bid

    def is_my_bid_min(self, task, my_bid):
        for task_bid in self.received_bids:
            if task.index == task_bid.task.index and task_bid.bid < my_bid:
                return False
        return True

    def remove_old_bids(self):
        time = rospy.Time.now()
        self.received_bids = [bid for bid in self.received_bids if bid.stamp.secs > (time.secs - self.bid_tolerance)]

    def is_task_available(self, id):
        for task in self.available_tasks:
            if task.uniqueid == id:
                return True
        return False
    
    def clear_public_tasks(self):
        rospy.wait_for_service('/clear_taken_task', timeout=1000)
        manager_clear_task = rospy.ServiceProxy('/clear_taken_task', AddSingleTask)
        for task in self.my_tasks:
            if (not self.current_task or self.current_task.uniqueid != task.uniqueid) and not task.private:
                self.my_tasks.remove(task)
                self.available_tasks.append(task)
                # Send the task to the manager and wait for response
                try:
                    response = manager_clear_task(task, False)
                except rospy.ServiceException as e:
                    rospy.logerr(f'Service add_task failed: {e}')

    # Based on https://www.mdpi.com/2218-6581/4/3/316/htm
    def allocate_tasks(self):
        rate = rospy.Rate(10)
        ignore = []
        while not rospy.is_shutdown():
            self.find_topics()
            
            if not self.start_mission:
                self.clear_public_tasks()
                rate.sleep()
                continue
            self.mutex.acquire()
            # if there are free tasks calculate which task is closest
            if self.available_tasks and (not self.my_tasks or self.available_tasks[0].priority < self.my_tasks[0].priority):
                self.remove_old_bids()
                
                task, bid = self.get_min_bid_on_task(ignore)
                if not task:
                    self.mutex.release()
                    rate.sleep()
                    continue
                # Check if I have the lowest bid from the current bids
                if not self.is_my_bid_min(task, bid):
                    ignore.append(task.uniqueid)
                    self.mutex.release()
                    rate.sleep()
                    continue
                ignore = []
                self.bid_on_task(task, bid)
                rospy.sleep(self.bid_patience) # Wait for others to bid
                if not self.is_task_available(task.uniqueid):
                    self.mutex.release()
                    rate.sleep()
                    continue
                
                if self.is_my_bid_min(task, bid):
                    self.accept_task(task)
                else:
                    rospy.logwarn(f'{self.id} Lost the bid for {task.uniqueid} with {bid}, retrying...')
                    self.mutex.release()
                    rospy.sleep(self.accept_patience) # Wait for accept signal
            elif self.my_tasks and (not self.current_task or self.my_tasks[0].uniqueid != self.current_task.uniqueid):
                self.current_task = self.my_tasks[0]
                self.clear_public_tasks()
            
            if self.current_task:
                self.publish_goal(self.current_task)
            if self.mutex.locked():
                self.mutex.release()
            self.my_tasks_pub.publish(self.my_tasks)
            rate.sleep()

    def publish_goal(self, task):
        goal = MoveBaseActionGoal()
        goal.goal.target_pose.pose.position.x = task.x
        goal.goal.target_pose.pose.position.y = task.y
        goal.goal.target_pose.pose.position.z = task.threshold
        goal.goal_id.id = task.uniqueid
        self.current_task_pub.publish(goal)
        
    def accept_task(self, task):
        task.droneid = self.id
        self.add_task(task)
        self.accept_task_pub.publish(task)
        rospy.loginfo(f'{self.id} accepted task {task.uniqueid}')

    def bid_on_task(self, task, bid):
        # publish a bid to the network
        task_bid = TaskBid()
        task_bid.bid = bid
        task_bid.task = task
        task_bid.stamp = rospy.Time.now()
        self.bid_pub.publish(task_bid)
    
    def add_task(self, task):
        task.stamp = rospy.Time.now()
        for available_task in self.available_tasks:
            if available_task.uniqueid == task.uniqueid:
                self.available_tasks.remove(available_task)
                break
        self.my_tasks.append(task)
        self.my_tasks.sort(key=lambda x: (x.priority, self.calculate_task_bid(x), x.stamp))
    
    def remove_task_by_id(self, task_id):
        for i, task in enumerate(self.my_tasks):
            if task.uniqueid == task_id:
                self.my_tasks.pop(i)
    
    def bid_cb(self, bid_msg):
        self.received_bids.append(bid_msg)

    def accept_cb(self, task_msg):
        self.available_tasks = [task for task in self.available_tasks if not task.uniqueid == task_msg.uniqueid]

    def add_task_srv_cb(self, task_msg):
        task_msg.task.droneid = self.id
        for task in itertools.chain(self.available_tasks, self.my_tasks):
            if task.uniqueid == task_msg.task.uniqueid:
                return AddSingleTaskResponse(False, 'Task already taken')
        self.accept_task(task_msg.task)
        return AddSingleTaskResponse(True, 'Task added')

    def start_mission_srv_cb(self, start):
        self.start_mission = start.data
        if self.start_mission:
            return SetBoolResponse(True, 'Mission is enabled')
        return SetBoolResponse(True, 'Mission is disabled')

    def task_status_cb(self, status_msg):
        if status_msg.status == GoalStatus.SUCCEEDED and self.current_task is not None:
            rospy.loginfo(f'{self.id} Goal {status_msg.goal_id.id} succeeded!')
            self.task_complete_pub.publish(self.current_task)
            self.mutex.acquire()
            self.remove_task_by_id(status_msg.goal_id.id)
            self.current_task = None
            self.mutex.release()

    def available_tasks_cb(self, tasks):
        self.available_tasks = tasks.task.copy()

    def odom_cb(self, odom_msg):
        self.current_position[0] = odom_msg.pose.pose.position.x + self.offset_x
        self.current_position[1] = odom_msg.pose.pose.position.y + self.offset_y
        quart = odom_msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([quart.x, quart.y, quart.z, quart.w])
        self.yaw = yaw

    def find_topics(self):
        topics = rospy.get_published_topics()
        for topic in topics:
            if 'task_allocation/accept_task' in topic[0] and self.id not in topic[0] and topic[0] not in self.subscribed:
                rospy.Subscriber(topic[0], Task, self.accept_cb, queue_size=self.queue_size)
                self.subscribed.append(topic[0])
            elif 'task_allocation/bid' in topic[0] and self.id not in topic[0] and topic[0] not in self.subscribed:
                rospy.Subscriber(topic[0], TaskBid, self.bid_cb, queue_size=self.queue_size)
                self.subscribed.append(topic[0])


if __name__ == '__main__':
    try:
        task = TaskAllocation()
        task.allocate_tasks()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass