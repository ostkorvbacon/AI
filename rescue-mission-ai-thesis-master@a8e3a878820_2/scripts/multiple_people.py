#!/usr/bin/env python
import rospy
from multi_drone_sar.msg import PersonIdAndPosition
class MultiplePeople:
    def __init__(self):
        rospy.init_node('~multiple_people', anonymous=False)
        id = rospy.get_namespace()
        self.id = id
        self.time_threshold = rospy.get_param('~time_threshold', 7)
        self.start_time = rospy.Time.now()
        queue_size = rospy.get_param('~queue_size', 10)
        self.latest_msg_time = 0
        self.activation_msg = PersonIdAndPosition()
        self.people_counter = 0
        self.pub_activate = rospy.Publisher('~person_found', PersonIdAndPosition, queue_size=queue_size)
        rospy.Subscriber('{}find_people/found_person'.format(id), PersonIdAndPosition, self.time_cb, queue_size=queue_size)

    def time_cb(self, data):
        time_difference = 0
        self.message_received = rospy.Time.now()
        if self.latest_msg_time is 0: # first time finding a person
            self.latest_msg_time = self.message_received
            self.people_counter = self.people_counter + 1
        else:
            time_difference = self.message_received - self.latest_msg_time # measures the time since last message has been received
            self.latest_msg_time = self.message_received # save the newest received message for comparision next time
            time_difference = time_difference.to_sec() # convert the difference to seconds
            if time_difference < self.time_threshold: # compare the difference to the threshold to see if the person found was found close enough
                self.people_counter = self.people_counter + 1
            else:
                self.people_counter = 0 # the time difference was to large and we reset the counter
            if self.people_counter > 2: #if the time between several people are within the threshold we start a spiral
                self.activation_msg = data
                self.pub_activate.publish(self.activation_msg)
                rospy.loginfo(f"{self.id} has found MULTIPLE PEOPLE!: {self.people_counter}")
                self.people_counter = 0

if __name__ == '__main__':
    try:
        mp = MultiplePeople()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass