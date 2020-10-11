import os
import sys
import time
import threading

import rospy
import rospkg
import dynamic_reconfigure.client

from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from PyQt5.QtCore import pyqtSlot, pyqtSignal
from PyQt5.QtGui import *

# add resources folder to python load path
sys.path.append(os.path.join(rospkg.RosPack().get_path("aleph2_gui"), "resources"))

from resources_rc import *

from std_msgs.msg import Bool, Float32
from nav_msgs.msg import Odometry

from aleph2_gui.joystick_selector import JoystickSelector
from .steering_module import SteeringModule


class Aleph2DrivetrainController(Plugin):
    refresh_signal = pyqtSignal()
    odom_refresh_signal = pyqtSignal()
    
    DRIVER_TIMEOUT = 1.0 # sec

    def DriverPanel(self, name):
        return self._widget.PDRIVER.findChild(QWidget, name)

    def InputPanel(self, name):
        return self._widget.PIN.findChild(QWidget, name)

    def OdomPanel(self, name):
        return self._widget.PODOM.findChild(QWidget, name)

    def PowerPanel(self, name):
        return self._widget.PPOWER.findChild(QWidget, name)

    @pyqtSlot(int)
    def ISensChanged(self, value):
        self.sensitivity = 1
        self.sensitivity_level = value
        for _ in range(value, 0):
            self.sensitivity /= 1.33
        for _ in range(0, value):
            self.sensitivity *= 1.33

    @pyqtSlot(bool)
    def CBACTIVEToggled(self, checked):
        self.input_active = checked

    @pyqtSlot(bool)
    def CBMUXToggled(self, checked):
        if not checked:
            self.InputPanel("CBIGNORE").setChecked(False)
        self.InputPanel("CBIGNORE").setEnabled(checked)
        self.mux_mode = checked
        self.steerer.mux_mode = checked

    @pyqtSlot(bool)
    def CBIGNOREToggled(self, checked):
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
        self.PowerPanel("PBL").setChecked(self.power[1])
        self.PowerPanel("PFR").setChecked(self.power[2])
        self.PowerPanel("PBR").setChecked(self.power[3])

        self.PowerPanel("BFL").setChecked(self.brake[0])
        self.PowerPanel("BBL").setChecked(self.brake[1])
        self.PowerPanel("BFR").setChecked(self.brake[2])
        self.PowerPanel("BBR").setChecked(self.brake[3])

    @pyqtSlot()
    def FLConfigChanged(self):
        self.power[0] = self.PowerPanel("PFL").isChecked()
        self.brake[0] = self.PowerPanel("BFL").isChecked()

        self.FLConfigClient.update_configuration({
            "power": self.power[0],
            "brake": self.brake[0]
        })

    @pyqtSlot()
    def BLConfigChanged(self):
        self.power[1] = self.PowerPanel("PBL").isChecked()
        self.brake[1] = self.PowerPanel("BBL").isChecked()

        self.BLConfigClient.update_configuration({
            "power": self.power[1],
            "brake": self.brake[1]
        })

    @pyqtSlot()
    def FRConfigChanged(self):
        self.power[2] = self.PowerPanel("PFR").isChecked()
        self.brake[2] = self.PowerPanel("BFR").isChecked()

        self.FRConfigClient.update_configuration({
            "power": self.power[2],
            "brake": self.brake[2]
        })

    @pyqtSlot()
    def BRConfigChanged(self):
        self.power[3] = self.PowerPanel("PBR").isChecked()
        self.brake[3] = self.PowerPanel("BBR").isChecked()

        self.BRConfigClient.update_configuration({
            "power": self.power[3],
            "brake": self.brake[3]
        })

    def GetConfigCallback(self, wheel):
        def ConfigCallback(config):
            self.power[wheel] = config.power
            self.brake[wheel] = config.brake
            self.refresh_signal.emit()
        return ConfigCallback

    def __init__(self, context):
        super(Aleph2DrivetrainController, self).__init__(context)

        self.setObjectName('DrivetrainController')

        self._widget = QWidget()

        ui_file = os.path.join(
            rospkg.RosPack().get_path("aleph2_gui"), 
            "resources/ui/aleph2_drivetrain_controller.ui"
        )

        loadUi(ui_file, self._widget)
        self._widget.setObjectName('DrivetrainUi')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        context.add_widget(self._widget)

        self.odometry = Odometry()

        self.power = [False] * 4
        self.brake = [False] * 4

        self.sensitivity = 1
        self.sensitivity_level = 0
        self.btn_sens_up = 7
        self.btn_sens_down = 6
        self.btn_pwr_on = 0
        self.btn_pwr_off = 1
        self.btn_brk_on = 4
        self.btn_brk_off = 2

        self.input_active = True
        self.mux_mode = False
        self.ignore_planner = False
        self.reconfigure_broken = False

        self.usage_label = "USAGE:"

        self.last_update = time.time()
        self.driver_online = threading.Event()

        self.pub_ign_planner = rospy.Publisher(
            "ign_planner",
            Bool,
            queue_size=1
        )

        try:
            self.FLConfigClient = dynamic_reconfigure.client.Client(
                "aleph2/drivetrain/joints/wheel_FL_joint", 
                timeout=1, 
                config_callback=self.GetConfigCallback(0)
            )

            self.BLConfigClient = dynamic_reconfigure.client.Client(
                "aleph2/drivetrain/joints/wheel_RL_joint", 
                timeout=1, 
                config_callback=self.GetConfigCallback(1)
            )

            self.FRConfigClient = dynamic_reconfigure.client.Client(
                "aleph2/drivetrain/joints/wheel_FR_joint", 
                timeout=1, 
                config_callback=self.GetConfigCallback(2)
            )

            self.BRConfigClient = dynamic_reconfigure.client.Client(
                "aleph2/drivetrain/joints/wheel_RR_joint", 
                timeout=1, 
                config_callback=self.GetConfigCallback(3)
            )
        except rospy.ROSException:
            rospy.logerr("Could not initialize dynamic reconfigure clients for wheel joints")
            self.reconfigure_broken = True

        self.setup_signals()

        self.sub_odom = rospy.Subscriber(
            "aleph2/drivetrain/controllers/diff_drive/odom",
            Odometry,  
            self.odom_callback
        )

        self.sub_usage = rospy.Subscriber(
            "aleph2/drivetrain_control_loop/usage",
            Float32,
            self.usage_callback
        )

        self.selector = JoystickSelector(self.input_callback)
        self.steerer = SteeringModule()

        self.refresh_signal.emit()
        
        self.check_driver_thread = threading.Thread(target=self.check_driver_loop)
        self.check_driver_thread.daemon = True
        self.check_driver_thread.start()

        self.ignore_planner_thread = threading.Thread(target=self.ignore_planner_loop)
        self.ignore_planner_thread.daemon = True
        self.ignore_planner_thread.start()

    @pyqtSlot()
    def BTNControllerClicked(self):
        self.InputPanel("BTNController").setText(self.selector.Switch())

    def setup_signals(self):
        self.refresh_signal.connect(self.slot_refresh_gui)
        self.odom_refresh_signal.connect(self.slot_refresh_odom)

        self.InputPanel("BTNController").clicked.connect(self.BTNControllerClicked)
        self.InputPanel("ISENSITIVITY").valueChanged.connect(self.ISensChanged)
        self.InputPanel("CBACTIVE").toggled.connect(self.CBACTIVEToggled)
        self.InputPanel("CBMUX").toggled.connect(self.CBMUXToggled)
        self.InputPanel("CBIGNORE").toggled.connect(self.CBIGNOREToggled)

        if not self.reconfigure_broken:
            self.PowerPanel("PFL").toggled.connect(self.FLConfigChanged)
            self.PowerPanel("BFL").toggled.connect(self.FLConfigChanged)
            self.PowerPanel("PBL").toggled.connect(self.BLConfigChanged)
            self.PowerPanel("BBL").toggled.connect(self.BLConfigChanged)
            self.PowerPanel("PFR").toggled.connect(self.FRConfigChanged)
            self.PowerPanel("BFR").toggled.connect(self.FRConfigChanged)
            self.PowerPanel("PBR").toggled.connect(self.BRConfigChanged)
            self.PowerPanel("BBR").toggled.connect(self.BRConfigChanged)

    def input_callback(self, data):

        for i in data.buttons_pressed:
            if i == self.btn_sens_up and self.sensitivity_level < 5:
                self.sensitivity_level += 1
            if i == self.btn_sens_down and self.sensitivity_level > -5:
                self.sensitivity_level -= 1

            if i == self.btn_pwr_on:
                self.power = [True] * 4
            if i == self.btn_pwr_off:
                self.power = [False] * 4

            if i == self.btn_brk_on:
                self.brake = [True] * 4
            if i == self.btn_brk_off:
                self.brake = [False] * 4
                
        if len(data.buttons_pressed) > 0:
            self.refresh_signal.emit()

        if self.input_active:
            axes = []
            for axis in data.axes:
                axes.append(axis * self.sensitivity)
            self.steerer.Update(axes)

    def usage_callback(self, msg):
        self.usage_label = "USAGE: {:.2f}%".format(msg.data)
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

    def ignore_planner_loop(self):
        rate = rospy.Rate(10)
        while True:
            if self.mux_mode:
                lock = Bool(self.ignore_planner)
                self.pub_ign_planner.publish(lock)
            rate.sleep()

    def shutdown_plugin(self):
        self.sub_odom.unregister()
        self.pub_ign_planner.unregister()
        self.selector.shutdown()
        self.steerer.shutdown()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    # def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
