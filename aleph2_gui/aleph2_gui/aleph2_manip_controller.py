import os

from rclpy.node import Node
from rclpy.impl.rcutils_logger import RcutilsLogger
from rqt_gui.ros2_plugin_context import Ros2PluginContext
from ament_index_python import get_resource

from qt_gui.plugin import Plugin

os.environ["QT_API"] = "pyqt5"

from qtpy.uic import loadUi
from qtpy.QtWidgets import QWidget
from qtpy.QtCore import pyqtSlot, pyqtSignal

import aleph2_gui.resources.ta
from aleph2_gui.joystick_selector import JoystickSelector

from std_msgs.msg import Float64
from input_manager.msg import Input


class Aleph2ManipController(Plugin):
    refresh_signal = pyqtSignal()

    BTN_SENS_UP = 7
    BTN_SENS_DOWN = 6

    SENSITIVITY_STEP = 1.5

    def __init__(self, context: Ros2PluginContext):
        super(Aleph2ManipController, self).__init__(context)
        self.setObjectName("Aleph2ManipController")

        self._node: Node = context.node
        self._logger = self._node.get_logger().get_child("Aleph2ManipController")
        self._widget = QWidget()

        _, package_path = get_resource("packages", "aleph2_gui")

        ui_file = os.path.join(
            package_path,
            "share/aleph2_gui/ui/aleph2_manip_controller.ui",
        )

        loadUi(ui_file, self._widget)
        self._widget.setObjectName("ManipulatorUi")

        context.add_widget(self._widget)

        self.sensitivity = 1.0
        self.sensitivity_level = 0

        self.effortController = EffortController(self._node, self._logger)

        self.selector = JoystickSelector(
            self._node,
            self._logger,
            self.input_callback,
            self.InputPanel("ControllersList"),
        )

        self.setup_signals()

    def InputPanel(self, name):
        return self._widget.PIN.findChild(QWidget, name)

    def setup_signals(self):
        self.refresh_signal.connect(self.slot_refresh_gui)
        self.InputPanel("SBSENS").valueChanged.connect(self.slot_change_sensitivty)
        self.selector.controllerChanged.connect(self.slot_reset_sensitivity)

    @pyqtSlot()
    def slot_reset_sensitivity(self):
        self.InputPanel("SBSENS").setValue(0)

    @pyqtSlot(int)
    def slot_change_sensitivty(self, value):
        self.sensitivity = self.SENSITIVITY_STEP ** value
        self.sensitivity_level = value

    @pyqtSlot()
    def slot_refresh_gui(self):
        self.InputPanel("SBSENS").setValue(self.sensitivity_level)

    def input_callback(self, data: Input):
        try:
            for i in data.buttons_pressed:
                if i == self.BTN_SENS_UP and self.sensitivity_level < 7:
                    self.sensitivity_level += 1
                if i == self.BTN_SENS_DOWN and self.sensitivity_level > -1:
                    self.sensitivity_level -= 1
        except IndexError:
            self._logger.error(
                "Received input from an unsupported device: not enough buttons",
                throttle_duration_sec=3.0,
            )

        if len(data.buttons_pressed) > 0:
            self.refresh_signal.emit()

        self.effortController.input_callback(data, self.sensitivity)

    def shutdown_plugin(self):
        self.selector.shutdown()
        self.effortController.shutdown()


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
    JOINT_KEYS = {"gripper": [0, 1]}
    BASE_MULTPLIER = 50.0
    MULTPLIER = {
        "base": 1.0,
        "elbow": 1.0,
        "gripper": 1.0,
        "shoulder": 1.0,
        "wrist_roll": 1.0,
        "wrist_tilt": 1.0,
    }

    def __init__(self, node: Node, logger: RcutilsLogger):
        self._node = node
        self._logger = logger.get_child("EffortController")

        self.pubs = {}
        for name in self.JOINT_NAMES:
            self.pubs[name] = self._node.create_publisher(
                Float64, f"aleph2/manip/controllers/effort/{name}/command", 1
            )

    def input_callback(self, data: Input, sens_mult: float):
        axes = data.axes
        buttons = data.buttons
        try:
            mult = sens_mult * self.BASE_MULTPLIER
            for joint_name in self.JOINT_AXES:
                joint_axes = self.JOINT_AXES[joint_name]
                value = 0.0
                if len(joint_axes) > 1:
                    value = (axes[joint_axes[0]] - axes[joint_axes[1]]) / 2.0
                else:
                    value = axes[joint_axes[0]]

                joint_mult = mult * self.MULTPLIER[joint_name]
                value = joint_mult * value
                self.pubs[joint_name].publish(Float64(data=value))

            for joint_name in self.JOINT_KEYS:
                # fmt: off
                value = (
                    float(buttons[self.JOINT_KEYS[joint_name][0]]) - 
                    float(buttons[self.JOINT_KEYS[joint_name][1]])
                )
                # fmt: on

                joint_mult = mult * self.MULTPLIER[joint_name]
                value = joint_mult * value
                self.pubs[joint_name].publish(Float64(data=value))
        except IndexError:
            self._logger.error(
                "Received input from an unsupported device: not enough axes",
                throttle_duration_sec=3.0,
            )

    def shutdown(self):
        for pub in self.pubs.items():
            self._node.destroy_publisher(pub)
