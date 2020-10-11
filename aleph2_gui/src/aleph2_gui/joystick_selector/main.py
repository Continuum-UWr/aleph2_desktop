import os
import rospy
import rosgraph

from input_msgs.msg import InputMessage

class JoystickSelector():

    def __init__(self, callback):
        self.client_callback = callback
        self.current_device = 0
        self.master = rosgraph.Master('/rostopic/')

        self.sub_input = 0
        rate = rospy.Rate(30)
        self.current_topic = "<None>"

    def Switch(self):
        topics = [topic[0][7:] for topic in self.master.getTopicTypes() \
                if topic[0].startswith('/input')]
        topics.append("<None>")

        if self.sub_input != 0:
            self.sub_input.unregister()
            self.sub_input = 0

        new_index = (topics.index(self.current_topic) + 1) % len(topics)
        self.current_topic = topics[new_index]

        if self.current_topic != "<None>":
            self.sub_input = rospy.Subscriber(
                "/input/" + self.current_topic, 
                InputMessage,
                self.input_callback
            )

        return self.current_topic

    def input_callback(self, data):
        self.client_callback(data)

    def shutdown(self):
        if self.sub_input != 0:
            self.sub_input.unregister()
