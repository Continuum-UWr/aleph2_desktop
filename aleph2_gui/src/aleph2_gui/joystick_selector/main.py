import os
import rospy
import rosgraph

from input_msgs.msg import InputMessage
from input_msgs.msg import DevicesListMessage

class JoystickSelector():

    def __init__(self, callback):
        self.client_callback = callback
        self.current_device = 0
        #self.master = rosgraph.Master('/rostopic/')
        self.devices = {}
        self.devices_sub = rospy.Subscriber("/input/devices_list", DevicesListMessage, self.devices_callback)

        self.sub_input = 0
        rate = rospy.Rate(30)
        self.current_topic = "<None>"
        self.last_index = 0
        self.topics = ["<None>"]

    def devices_callback(self, data):
        name = data.node_name
        devs = data.devices_list
        self.devices[name] = devs
        self.topics = ["<None>"]
        for name, dev_list in self.devices.items():
            self.topics += dev_list
        if self.current_topic not in self.topics:
            self.current_topic = "<None>"
            self.last_index = 0

    def Switch(self):
        topics = ["<None>"]
        for name, dev_list in self.devices.items():
            topics += dev_list
        #topics.append("<None>")

        if self.sub_input != 0:
            self.sub_input.unregister()
            self.sub_input = 0

        try:
            new_index = (topics.index(self.current_topic) + 1) % len(topics)
            self.last_index = new_index-1
            self.current_topic = topics[new_index]
        except ValueError as e:
            new_index = last_index
            last_index -= 1
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
