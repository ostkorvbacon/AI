#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
from multi_drone_sar.msg import Task, PersonIdAndPosition
from multi_drone_sar.srv import AddSingleTask, AddSingleTaskResponse
from actionlib_msgs.msg import GoalStatus

class FirstAid:
    def __init__(self):
        rospy.init_node('first_aid', anonymous=False)
        id = rospy.get_namespace()
        self.queue_size = rospy.get_param('~queue_size', 10)
        self.id = id[1:-1]
        self.subscribed = []
        self.name = rospy.get_param('~first_aid_drone_name', 'drone_4')
        self.priority = rospy.get_param('~first_aid_drone_priority', 2)
        self.person_in_need = PersonIdAndPosition()
        self.uniqueid_counter = 0
        rospy.Subscriber('navigation/status', GoalStatus, self.task_status_cb, queue_size=self.queue_size)
        self.inhib_pub = rospy.Publisher('~inhib', Empty, queue_size=self.queue_size)
        self.inhib_pub.publish(Empty())
            
    def help_person(self, msg):
        task_msg = self.create_task_msg(msg)
        rospy.wait_for_service('add_task', timeout=1000)
        try:
            add_first_aid_task = rospy.ServiceProxy('add_task', AddSingleTask)
            response = add_first_aid_task(task_msg, False)
            if not response.success:
                return AddSingleTaskResponse(False, response.message)
        except rospy.ServiceException as e:
            rospy.logerr(f'Service add_task failed: {e}')
        return AddSingleTaskResponse(True, 'First Aid task added')

    def create_task_msg(self, msg):
        temp_task_msg = Task()
        temp_task_msg.droneid = self.name
        temp_task_msg.x = msg.x
        temp_task_msg.y = msg.y
        temp_task_msg.private = True
        temp_task_msg.priority = self.priority
        temp_task_msg.stamp = rospy.Time.now()
        temp_task_msg.uniqueid = self.name + f'/first_aid:{self.uniqueid_counter}'
        self.uniqueid_counter += 1
        return temp_task_msg

    def found_people_cb(self, msg):
        if msg.first_aid == True and self.id == self.name:
            self.help_person(msg)
            rospy.loginfo(f'{self.id} is delivering first aid. x:{msg.x}, y:{msg.y}, id:{msg.id}')

    def people_found_listener(self):
        topics = rospy.get_published_topics()
        for topic in topics:
            if '/found_person' in topic[0] and topic[0] not in self.subscribed:
                rospy.Subscriber(topic[0], PersonIdAndPosition, self.found_people_cb, queue_size=self.queue_size)
                self.subscribed.append(topic[0])
    
    def task_status_cb(self, status_msg):
        if f'{self.id}/first_aid:' in status_msg.goal_id.id and status_msg.status != GoalStatus.SUCCEEDED:
            self.inhib_pub.publish(Empty())

    def main(self):
        while not rospy.is_shutdown():
            self.people_found_listener()

if __name__ == '__main__':
    fa = FirstAid()
    try:
        fa.main()
    except rospy.ROSInterruptException:
        pass