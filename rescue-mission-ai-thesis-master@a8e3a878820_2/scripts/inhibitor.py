#!/usr/bin/env python
import rospy
import rostopic

class Inhibitor:
    def __init__(self):
        rospy.init_node('~suppressor', anonymous=True)
        queue_size = rospy.get_param('~queue_size', 10)
        self.time_suppressed = rospy.get_param('~time_suppressed')
        upper_topic = rospy.get_param('~upper_topic')
        lower_topic = rospy.get_param('~lower_topic')
        
        lower_topic_type = self.wait_for_topic(rospy.get_namespace() + lower_topic)
        if lower_topic_type is None:
            rospy.logerr(f'Topic {lower_topic_type} not found')
            return
        
        upper_topic_type = self.wait_for_topic(rospy.get_namespace() + upper_topic)
        if upper_topic_type is None:
            rospy.logerr(f'Topic {upper_topic_type} not found')
            return
        
        self.pub = rospy.Publisher('~output', lower_topic_type, queue_size=queue_size)
        self.start_time = rospy.Time()
        rospy.Subscriber(upper_topic, upper_topic_type, self.upper_cb, queue_size=queue_size)
        rospy.Subscriber(lower_topic, lower_topic_type, self.lower_cb, queue_size=queue_size)

    def upper_cb(self, msg):
        self.start_time = rospy.Time.now()

    def lower_cb(self, msg):
        current_time = rospy.Time.now()
        if (current_time - self.start_time).secs > self.time_suppressed/1000:
            self.pub.publish(msg)

    def wait_for_topic(self, name):
        """Wait for the topic to be avaible, None if early exit.
        Args:
            name (string): Name of the topic to wait for.
        Returns:
            class or None: The class of the topic or none if early exit.
        """
        topic_type = rostopic.get_topic_class(name)[0]
        rospy.loginfo(f"Searching for topic: {name}")
        rate = rospy.Rate(10)
        while topic_type is None and not rospy.is_shutdown():
            topic_type = rostopic.get_topic_class(name)[0]
            rate.sleep()
        rospy.loginfo(f'Found topic: {name}')
        return topic_type

if __name__ == '__main__':
    try:
        inhib = Inhibitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
