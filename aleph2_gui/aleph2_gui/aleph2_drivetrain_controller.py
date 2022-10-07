import os
import time
import threading

from rclpy.node import Node
from rqt_gui.ros2_plugin_context import Ros2PluginContext
from ament_index_python import get_resource

from rqt_gui_py.plugin import Plugin
from qtpy import loadUi
from qtpy.QtWidgets import QWidget
from qtpy.QtCore import pyqtSlot, pyqtSignal

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32
from nav_msgs.msg import Odometry

import aleph2_gui.resources.ta
from aleph2_gui.joystick_selector import JoystickSelector


class Aleph2DrivetrainController(Plugin):
    refresh_signal = pyqtSignal()
    odom_refresh_signal = pyqtSignal()

    DRIVER_TIMEOUT = 1.0  # sec

    BTN_SENS_UP = 7
    BTN_SENS_DOWN = 6
    BTN_PWR_ON = 0
    BTN_PWR_OFF = 1
    BTN_BRK_ON = 4
    BTN_BRK_OFF = 2
    BTN_TEMP_ACTIVE = 5

    AXIS_LINEAR = 1
    AXIS_ANGULAR = 3

    SENSITIVITY_STEP = 1.33
    LINEAR_MULTIPLIER = -1.11
    ANGULAR_MULTIPLIER = -2.22

    def __init__(self, context: Ros2PluginContext):
        super(Aleph2DrivetrainController, self).__init__(context)

        self.setObjectName("DrivetrainController")

        self._node: Node = context.node
        self._logger = self._node.get_logger().get_child("Aleph2DrivetrainController")
        self._widget = QWidget()

        _, package_path = get_resource("packages", "aleph2_gui")

        ui_file = os.path.join(
            package_path,
            "share/aleph2_gui/ui/aleph2_drivetrain_controller.ui",
        )

        loadUi(ui_file, self._widget)
        self._widget.setObjectName("DrivetrainUi")

        context.add_widget(self._widget)

        self.odometry = Odometry()

        self.power = [False] * 4
        self.brake = [False] * 4

        self.sensitivity = 1
        self.sensitivity_level = 0

        self.input_active = False
        self.input_temp_active = False
        self.mux_mode = False
        self.ignore_planner = False
        self.reconfigure_broken = False

        self.usage_label = "USAGE:"

        self.last_update = time.time()
        self.driver_online = threading.Event()

        self.pub_ign_planner = self._node.create_publisher(Bool, "ign_planner", 1)
        self.pub_cmd_vel = self._node.create_publisher(Twist, "aleph2/cmd_vel", 1)
        self.pub_joy_vel = self._node.create_publisher(Twist, "joy_vel", 1)

        self.sub_odom = self._node.create_subscription(
            Odometry,
            "aleph2/drivetrain/controllers/diff_drive/odom",
            self.odom_callback,
            10,
        )

        self.sub_usage = self._node.create_subscription(
            Float32,
            "aleph2/drivetrain_control_loop/usage",
            self.usage_callback,
            10,
        )

        self.selector = JoystickSelector(
            self._node,
            self._logger,
            self.input_callback,
            self.InputPanel("ControllersList"),
        )

        self.setup_signals()
        self.refresh_signal.emit()

        self.ign_planner_timer = self._node.create_timer(
            0.1, self.ign_planner_timer_callback
        )

        self.check_driver_thread = threading.Thread(
            target=self.check_driver_loop, daemon=True
        )
        self.check_driver_thread.start()

    def DriverPanel(self, name):
        return self._widget.PDRIVER.findChild(QWidget, name)

    def InputPanel(self, name):
        return self._widget.PIN.findChild(QWidget, name)

    def OdomPanel(self, name):
        return self._widget.PODOM.findChild(QWidget, name)

    def PowerPanel(self, name):
        return self._widget.PPOWER.findChild(QWidget, name)

    @pyqtSlot(int)
    def slot_change_sensitivity(self, value):
        self.sensitivity = self.SENSITIVITY_STEP ** value
        self.sensitivity_level = value

    @pyqtSlot()
    def slot_reset_sensitivity(self):
        self.InputPanel("ISENSITIVITY").setValue(0)

    @pyqtSlot(bool)
    def slot_set_input_active(self, checked):
        self.input_active = checked

    @pyqtSlot(bool)
    def slot_toggle_mux_mode(self, checked):
        if not checked:
            self.InputPanel("CBIGNORE").setChecked(False)
        self.InputPanel("CBIGNORE").setEnabled(checked)
        self.mux_mode = checked

    @pyqtSlot(bool)
    def slot_toggle_ign_planner(self, checked):
        self.ignore_planner = checked

    @pyqtSlot()
    def slot_refresh_odom(self):
        self.OdomPanel("PBF").setValue(abs(self.odometry.twist.twist.linear.x * 15))
        if self.odometry.twist.twist.angular.z > 0:
            self.OdomPanel("PBDR").setValue(self.odometry.twist.twist.angular.z * 20)
            self.OdomPanel("PBDL").setValue(0)
        else:
            self.OdomPanel("PBDR").setValue(0)
            self.OdomPanel("PBDL").setValue(-self.odometry.twist.twist.angular.z * 20)

    @pyqtSlot()
    def slot_refresh_gui(self):
        self.DriverPanel("LUSAGE").setText(self.usage_label)

        if self.driver_online.is_set():
            self.DriverPanel("CBDRV").setChecked(True)
        else:
            self.DriverPanel("CBDRV").setChecked(False)

        self.InputPanel("ISENSITIVITY").setValue(self.sensitivity_level)

        self.PowerPanel("PFL").setChecked(self.power[0])
        self.PowerPanel("PRL").setChecked(self.power[1])
        self.PowerPanel("PFR").setChecked(self.power[2])
        self.PowerPanel("PRR").setChecked(self.power[3])

        self.PowerPanel("BFL").setChecked(self.brake[0])
        self.PowerPanel("BRL").setChecked(self.brake[1])
        self.PowerPanel("BFR").setChecked(self.brake[2])
        self.PowerPanel("BRR").setChecked(self.brake[3])

    @pyqtSlot()
    def slot_FL_config_changed(self):
        self.power[0] = self.PowerPanel("PFL").isChecked()
        self.brake[0] = self.PowerPanel("BFL").isChecked()

        # self.FLConfigClient.update_configuration(
        #     {"power": self.power[0], "brake": self.brake[0]}
        # )

    @pyqtSlot()
    def slot_RL_config_changed(self):
        self.power[1] = self.PowerPanel("PRL").isChecked()
        self.brake[1] = self.PowerPanel("BRL").isChecked()

        # self.BLConfigClient.update_configuration(
        #     {"power": self.power[1], "brake": self.brake[1]}
        # )

    @pyqtSlot()
    def slot_FR_config_changed(self):
        self.power[2] = self.PowerPanel("PFR").isChecked()
        self.brake[2] = self.PowerPanel("BFR").isChecked()

        # self.FRConfigClient.update_configuration(
        #     {"power": self.power[2], "brake": self.brake[2]}
        # )

    @pyqtSlot()
    def slot_RR_config_changed(self):
        self.power[3] = self.PowerPanel("PRR").isChecked()
        self.brake[3] = self.PowerPanel("BRR").isChecked()

        # self.BRConfigClient.update_configuration(
        #     {"power": self.power[3], "brake": self.brake[3]}
        # )

    # def get_config_callback(self, wheel):
    #     def ConfigCallback(config):
    #         self.power[wheel] = config.power
    #         self.brake[wheel] = config.brake
    #         self.refresh_signal.emit()

    #     return ConfigCallback

    def setup_signals(self):
        self.refresh_signal.connect(self.slot_refresh_gui)
        self.odom_refresh_signal.connect(self.slot_refresh_odom)

        self.InputPanel("ISENSITIVITY").valueChanged.connect(
            self.slot_change_sensitivity
        )
        self.InputPanel("CBACTIVE").toggled.connect(self.slot_set_input_active)
        self.InputPanel("CBMUX").toggled.connect(self.slot_toggle_mux_mode)
        self.InputPanel("CBIGNORE").toggled.connect(self.slot_toggle_ign_planner)

        self.PowerPanel("PFL").toggled.connect(self.slot_FL_config_changed)
        self.PowerPanel("BFL").toggled.connect(self.slot_FL_config_changed)
        self.PowerPanel("PRL").toggled.connect(self.slot_RL_config_changed)
        self.PowerPanel("BRL").toggled.connect(self.slot_RL_config_changed)
        self.PowerPanel("PFR").toggled.connect(self.slot_FR_config_changed)
        self.PowerPanel("BFR").toggled.connect(self.slot_FR_config_changed)
        self.PowerPanel("PRR").toggled.connect(self.slot_RR_config_changed)
        self.PowerPanel("BRR").toggled.connect(self.slot_RR_config_changed)

        self.selector.controllerChanged.connect(self.slot_reset_sensitivity)

    def input_callback(self, data):
        try:
            for i in data.buttons_pressed:
                if i == self.BTN_SENS_UP and self.sensitivity_level < 5:
                    self.sensitivity_level += 1
                if i == self.BTN_SENS_DOWN and self.sensitivity_level > -5:
                    self.sensitivity_level -= 1

                if i == self.BTN_PWR_ON:
                    self.power = [True] * 4
                if i == self.BTN_PWR_OFF:
                    self.power = [False] * 4

                if i == self.BTN_BRK_ON:
                    self.brake = [True] * 4

                if i == self.BTN_BRK_OFF:
                    self.brake = [False] * 4

            self.input_temp_active = data.buttons[self.BTN_TEMP_ACTIVE]
        except IndexError:
            self._logger.error(
                "Received input from an unsupported device: not enough buttons",
                throttle_duration_sec=3.0,
            )

        if len(data.buttons_pressed) > 0:
            self.refresh_signal.emit()

        try:
            if self.input_active or self.input_temp_active:
                linear = (
                    data.axes[self.AXIS_LINEAR]
                    * self.sensitivity
                    * self.LINEAR_MULTIPLIER
                )
                angular = (
                    data.axes[self.AXIS_ANGULAR]
                    * self.sensitivity
                    * self.ANGULAR_MULTIPLIER
                )
                self.publish_command(linear, angular)
        except IndexError:
            self._logger.error(
                "Received input from an unsupported device: not enough axes",
                throttle_duration_sec=3.0,
            )

    def publish_command(self, linear, angular):
        cmd_vel = Twist()

        cmd_vel.linear.x = linear
        cmd_vel.angular.z = angular

        if self.mux_mode:
            self.pub_joy_vel.publish(cmd_vel)
        else:
            self.pub_cmd_vel.publish(cmd_vel)

    def usage_callback(self, msg):
        self.usage_label = f"USAGE: {msg.data:.2f}%"
        self.last_update = time.time()
        self.driver_online.set()
        self.refresh_signal.emit()

    def odom_callback(self, data):
        self.odometry = data
        self.odom_refresh_signal.emit()

    def check_driver_loop(self):
        while True:
            self.driver_online.wait()

            while time.time() < self.last_update + self.DRIVER_TIMEOUT:
                time.sleep(self.last_update - time.time() + self.DRIVER_TIMEOUT)

            self.driver_online.clear()
            self.refresh_signal.emit()

    def ign_planner_timer_callback(self):
        if self.mux_mode:
            lock = Bool()
            lock.data = self.ignore_planner
            self.pub_ign_planner.publish(lock)

    def shutdown_plugin(self):
        self._node.destroy_subscription(self.sub_odom)
        self._node.destroy_subscription(self.sub_usage)
        self._node.destroy_publisher(self.pub_ign_planner)
        self._node.destroy_publisher(self.pub_cmd_vel)
        self._node.destroy_publisher(self.pub_joy_vel)
        self.selector.shutdown()
