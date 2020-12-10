import math
import os
import sys

import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from PyQt5.QtCore import pyqtSlot
from PyQt5.QtGui import *

import aleph2_gui.resources.ta
from aleph2_gui.joystick_selector import JoystickSelector

from std_msgs.msg import Float64


class Aleph2ManipController(Plugin):
    SENSITIVITY_STEP = 1.5
    BTN_SENS_UP = 7
    BTN_SENS_DOWN = 6

    def InputPanel(self, name):
        return self._widget.PIN.findChild(QWidget, name)

    def __init__(self, context):
        super(Aleph2ManipController, self).__init__(context)
        self.setObjectName('Aleph2ManipController')

        self._widget = QWidget()

        ui_file = os.path.join(
            rospkg.RosPack().get_path("aleph2_gui"),
            "resources/ui/aleph2_manip_controller.ui"
        )

        loadUi(ui_file, self._widget)
        self._widget.setObjectName('ManipulatorUi')

        if context.serial_number() > 1:
            self._widget.setWindowTitle("{} ({})".format(
                self._widget.windowTitle(), context.serial_number()))
        context.add_widget(self._widget)

        self.sensitivity = 0
        self.sensitivity_value = 1
        self.SensitivityChanged()

        self.effortController = EffortController()

        self.selector = JoystickSelector(self.input_callback)

        self.InputPanel("BTNController").clicked.connect(
            self.BTNControllerClicked)
        self.InputPanel("SBSENS").valueChanged.connect(
            self.SensitivityChanged)
        self.SensitivityChanged()

    @pyqtSlot()
    def SensitivityChanged(self):
        self.sensitivity = self.InputPanel("SBSENS").value()
        self.sensitivity_value = math.pow(
            self.SENSITIVITY_STEP, self.sensitivity)

    @pyqtSlot()
    def BTNControllerClicked(self):
        self.InputPanel("BTNController").setText(self.selector.Switch())

    def input_callback(self, data):  # noqa
        axes = []
        for axis in data.axes:
            axes.append(axis * self.sensitivity_value)

        for i in data.buttons_pressed:
            if i == self.BTN_SENS_UP and self.sensitivity < 7:
                self.sensitivity += 1
                self.InputPanel("SBSENS").setValue(self.sensitivity)
            if i == self.BTN_SENS_DOWN and self.sensitivity > -1:
                self.sensitivity -= 1
                self.InputPanel("SBSENS").setValue(self.sensitivity)

        self.effortController.input_callback(data, self.sensitivity_value)

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass


class EffortController:
    JOINT_NAMES = [
        "base",
        "elbow",
        "gripper",
        "shoulder",
        "wrist_roll",
        "wrist_tilt",
    ]
    JOINT_AXES = {
        "base": [0],
        "elbow": [1],
        "shoulder": [2, 5],
        "wrist_roll": [3],
        "wrist_tilt": [4],
    }
    JOINT_KEYS = {
        "gripper": ([0], [1])
    }
    BASE_MULTPLIER = 50.0
    MULTPLIER = {
        "base": 1.0,
        "elbow": 1.0,
        "gripper": 1.0,
        "shoulder": 1.0,
        "wrist_roll": 1.0,
        "wrist_tilt": 1.0,
    }

    def __init__(self):
        self.pubs = {}
        for name in self.JOINT_NAMES:
            self.pubs[name] = rospy.Publisher(
                "/aleph2/manip/controllers/effort/{}/command".format(name), Float64, queue_size=1)

    def input_callback(self, data, sens_mult):
        axes = list(data.axes)
        axes[2] = (axes[2]+1)/2
        axes[5] = -(axes[5]+1)/2

        for axisName in self.JOINT_AXES:
            mult = sens_mult*self.BASE_MULTPLIER*self.MULTPLIER[axisName]
            value = sum([axes[i] for i in self.JOINT_AXES[axisName]])

            value = mult * value
            self.pubs[axisName].publish(Float64(value))

        for axisName in self.JOINT_KEYS:
            mult = sens_mult*self.BASE_MULTPLIER*self.MULTPLIER[axisName]
            value = 1*sum([data.buttons[i] for i in self.JOINT_KEYS[axisName][0]]) + \
                -1*sum([data.buttons[i] for i in self.JOINT_KEYS[axisName][1]])

            value = mult * value
            print(value)
            self.pubs[axisName].publish(Float64(value))
