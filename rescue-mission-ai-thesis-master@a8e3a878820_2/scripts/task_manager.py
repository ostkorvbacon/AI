#!/usr/bin/env python
import rospy
import itertools
from multi_drone_sar.msg import TaskArray, Task
from multi_drone_sar.srv import AddSingleTask, AddSingleTaskResponse, RemoveSingleTask, RemoveSingleTaskResponse, AddMultipleTasks, AddMultipleTasksResponse, SetBatteryCharge
from std_srvs.srv import Empty
from std_msgs.msg import Empty as Empty_msg
from std_srvs.srv import SetBool, SetBoolResponse

class TaskManager:
    def __init__(self):
        rospy.init_node('task_manager', anonymous=False)
        self.queue_size = rospy.get_param('~queue_size', 10)
        self.service1 = rospy.Service('remove_task', RemoveSingleTask, self.remove_task_srv_cb)       # add service call to remove task by ID
        self.service2 = rospy.Service('add_task', AddSingleTask, self.add_task_srv_cb)     # add service call to add task first or last
        self.service3 = rospy.Service('clear_list', Empty, self.clear_tasks_srv_cb)      # add service call to clear list
        self.service4 = rospy.Service('add_multiple_tasks', AddMultipleTasks, self.add_task_list_srv_cb)      # add service call to add a whole list of tasks
        self.service5 = rospy.Service('clear_taken_task', AddSingleTask, self.clear_taken_task_srv_cb)
        self.available_tasks = TaskArray()
        self.taken_tasks = TaskArray()
        self.subscribed = []
        self.drones_connected = []
        self.timer_dict = {}
        self.pub_available = rospy.Publisher('~available', TaskArray, queue_size=self.queue_size)        # publisher to publish available tasks
        self.pub_taken = rospy.Publisher('~taken', TaskArray, queue_size=self.queue_size)         # publisher to publish taken tasks with drone id for the task (this is to use for task_allocation so a drone doesnt take more then 1 task at a time)
        rospy.Service('~start_mission', SetBool, self.start_mission_srv_cb)

    def start_mission_srv_cb(self, start):
        for drone in self.drones_connected:
            srv = f'/{drone}/start_mission'
            rospy.wait_for_service(srv, 1)
            try:
                start_mission_srv = rospy.ServiceProxy(srv, SetBool)
                output = start_mission_srv(start.data)
                rospy.loginfo(output)
            except rospy.ServiceException as e:
                rospy.logerr(f'Service start_mission failed: {e}')
        return SetBoolResponse(True, f'Start mission {start.data}')

    def manage_tasks(self):
        rate = rospy.Rate(10)
        for t in rospy.get_param('~task', []):
            task = Task(**t)
            self.add_task(task)
            if task.droneid:
                service_name = f'/{task.droneid}/add_task'
                rospy.loginfo(service_name)
                rospy.wait_for_service(service_name, timeout=5000)
                try:
                    drone_add_task_srv = rospy.ServiceProxy(service_name, AddSingleTask)
                    response = drone_add_task_srv(task, False)
                    rospy.loginfo(response.message)
                except rospy.ServiceException as e:
                    rospy.logerr(f'Service add_task failed: {e}')
        while not rospy.is_shutdown():
            self.find_topics()
            self.heartbeat_timer()
            self.index_lists(self.available_tasks.task)
            self.index_lists(self.taken_tasks.task)
            self.publish_available_tasks(self.available_tasks)
            self.publish_taken_tasks(self.taken_tasks)
            rate.sleep()

    def index_lists(self, current_list):
        if len(current_list) > 0:
            for idx, val in enumerate(current_list):
                val.index = idx

    def publish_available_tasks(self, available_tasks):
        self.pub_available.publish(available_tasks)

    def publish_taken_tasks(self, taken_tasks):
        self.pub_taken.publish(taken_tasks)

    def remove_task_srv_cb(self, task_srv):
        if task_srv.index >= 0 and task_srv.index <= len(self.available_tasks.task):
            self.available_tasks.task.pop(task_srv.index)
            return RemoveSingleTaskResponse(True, f'Task has been removed, index:{task_srv.index}')
        else:
            return RemoveSingleTaskResponse(False, f'Task has not been removed, index could not be found')

    def add_task(self, task, first=False):
        for existing_task in itertools.chain(self.available_tasks.task, self.taken_tasks.task):
            if existing_task.uniqueid == task.uniqueid:
                return False
        task.stamp = rospy.Time.now()
        if task.droneid or task.private:
            if first:
                self.taken_tasks.task.insert(0, task)
            else:
                self.taken_tasks.task.append(task)
        else:
            if first:
                self.available_tasks.task.insert(0, task)
            else:
                self.available_tasks.task.append(task)
        self.available_tasks.task.sort(key=lambda x: (x.priority, x.stamp))
        return True

    def add_task_srv_cb(self, task_srv):
        task = task_srv.task
        add_success = self.add_task(task, task_srv.first)
        if not add_success:
            AddSingleTaskResponse(False, f'Task already exist id:{task.uniqueid}, x:{task.x}, y:{task.y}, first:{task_srv.first}')
        return AddSingleTaskResponse(True, f'Task has been added id:{task.uniqueid}, x:{task.x}, y:{task.y}, first:{task_srv.first}')

    def clear_tasks_srv_cb(self, msg):
        self.available_tasks.task.clear()

    def add_task_list_srv_cb(self, task_list):
        for current_task in task_list.task:
            if not self.add_task(current_task):
                return AddMultipleTasksResponse(False, f'Failed to add tasks to the list, one of the tasks already exists')
        return AddMultipleTasksResponse(True, f'Tasks have been added to the available list!')

    def heartbeat_cb(self, heartbeat, args):
        current_drone_id = args[0]
        self.timer_dict[current_drone_id] = rospy.Time.now()

    def heartbeat_timer(self):
        for drone_id, last_heartbeat in self.timer_dict.items():
            time_since_last = (rospy.Time.now().to_sec() - last_heartbeat.to_sec())
            if time_since_last > 10:
                for val in self.taken_tasks.task:
                    uniqueid = val.uniqueid
                    if val.droneid == drone_id and not val.private:
                        self.move_task(self.taken_tasks.task, self.available_tasks.task, uniqueid, "")
                    elif val.droneid == drone_id and val.private:
                        self.remove_task(self.taken_tasks.task, uniqueid)

    def accept_task_cb(self, chosen_task, args):
        if chosen_task.private:
            return
        current_drone_id = args[0]
        did_move = self.move_task(self.available_tasks.task, self.taken_tasks.task, chosen_task.uniqueid, current_drone_id)
        if not did_move:
            rospy.logerr(f'Failed to move task {chosen_task.uniqueid} from available to taken for drone {current_drone_id}. Is the task still available?')

    def task_complete_cb(self, chosen_task):
        self.remove_task(self.taken_tasks.task, chosen_task.uniqueid)
        
    def drone_tasks_cb(self, tasks):
        for drone_task in tasks.task:
            self.add_task(drone_task)

    def remove_task(self, task_list, uniqueid):
        for idx, val in enumerate(task_list):
            if val.uniqueid == uniqueid:
                task_list.pop(idx)

    def clear_taken_task_srv_cb(self, task_srv):
        if not self.move_task(self.taken_tasks.task, self.available_tasks.task, task_srv.task.uniqueid, ""):
            rospy.logerr(f'Failed to move task {task_srv.task.uniqueid} from taken to avaible for drone {task_srv.task.droneid}')
            return AddSingleTaskResponse(False, f'Failed to clear task {task_srv.task.uniqueid}')
        return AddSingleTaskResponse(True, f'Succedded to clear task {task_srv.task.uniqueid}')

    def move_task(self, from_list, to_list, uniqueid, droneid):
        for idx, val in enumerate(from_list):
            if val.uniqueid == uniqueid:
                val.droneid = droneid # depending on from what list we move the task this can be different. Avail -> Taken droneid: name of drone taking task. Taken -> Avail droneid: ""
                to_list.append(from_list.pop(idx))
                return True
        return False

    def find_topics(self):
        topics = rospy.get_published_topics()
        for topic in topics:
            name = topic[0].split('/')[1]
            if '/heartbeat' in topic[0] and topic[0] not in self.subscribed:         # listen to drones heartbeats
                rospy.Subscriber(topic[0], Empty_msg, self.heartbeat_cb, queue_size=self.queue_size, callback_args=[name])
                self.subscribed.append(topic[0])
            elif 'task_allocation/accept_task' in topic[0] and topic[0] not in self.subscribed:         # add subscribers to accept messages from all drones to know when a drone takes a task (to move it from available to taken)
                rospy.Subscriber(topic[0], Task, self.accept_task_cb, queue_size=self.queue_size, callback_args=[name])
                self.subscribed.append(topic[0])
            elif 'task_allocation/task_complete' in topic[0] and topic[0] not in self.subscribed:         # listen to drones when they have completed a task
                rospy.Subscriber(topic[0], Task, self.task_complete_cb, queue_size=self.queue_size)
                self.subscribed.append(topic[0])
            elif 'my_tasks' in topic[0] and topic[0] not in self.subscribed:         # listen to drones when they have completed a task
                rospy.Subscriber(topic[0], TaskArray, self.drone_tasks_cb, queue_size=self.queue_size)
                self.subscribed.append(topic[0])
            if 'drone_' in name and name not in self.drones_connected:
                self.drones_connected.append(name)


if __name__ == '__main__':
    task_manager = TaskManager()
    try:
        task_manager.manage_tasks()
    except rospy.ROSInterruptException:
        pass