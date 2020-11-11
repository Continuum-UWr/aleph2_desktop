import sys
import rospy
import os

from geometry_msgs.msg import Twist


class SteeringModule:
    def __init__(self):
        self.axis_f = 1
        self.axis_d = 3
        self.pub_cmd  = rospy.Publisher('aleph2/drivetrain/controllers/diff_drive/cmd_vel', Twist, queue_size=10)
        self.pub_mux = rospy.Publisher('joy_vel', Twist, queue_size=10)
        self.mux_mode = False

    def Update(self, axes):

        steer_analog = Twist();

        steer_analog.linear.x = axes[self.axis_f] / -0.9
        steer_analog.angular.z = axes[self.axis_d] / -0.45

        if self.mux_mode:
            self.pub_mux.publish(steer_analog)
        else:
            self.pub_cmd.publish(steer_analog)

    def shutdown(self):
        self.pub_cmd.unregister()
        self.pub_mux.unregister()
