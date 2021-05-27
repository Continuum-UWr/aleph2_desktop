import os
import rospy
import rosgraph

from input_manager.msg import Input
from input_manager.msg import DeviceList
from python_qt_binding.QtCore import pyqtSlot, pyqtSignal, QObject
from python_qt_binding.QtGui import QColor


COLOR_BLUE = QColor("#3a3a99")
COLOR_RED = QColor("#bf2626")
COLOR_BLACK = QColor(0, 0, 0, 0)
NONE_DEV = "<None>"


class JoystickSelector(QObject):
    controllerChanged = pyqtSignal()
    devicesUpdated = pyqtSignal()

    def __init__(self, callback, widget):
        super(JoystickSelector, self).__init__()
        self.client_callback = callback
        self.widget = widget

        self.devices_dict = {}
        self.selected_dev = NONE_DEV
        self.devices = [NONE_DEV]
        self.active_dict = {}
        self.active = [False]

        self.sub_input = 0
        self.widget.itemClicked.connect(self.device_selected)
        self.devicesUpdated.connect(self.build_list)
        self.controllerChanged.connect(self.build_list)
        self.devices_sub = rospy.Subscriber(
            "input/devices_list", DeviceList, self.devices_callback
        )

        self.devicesUpdated.emit()

    def devices_callback(self, data):
        name = data.node_name
        self.devices_dict[name] = data.devices_list
        self.devices = [NONE_DEV]
        self.active_dict[name] = data.active
        self.active = [False]

        for k in self.devices_dict.keys():
            self.devices += self.devices_dict[k]
            self.active += self.active_dict[k]

        if self.selected_dev not in self.devices:
            self.sub_input.unregister()
            self.sub_input = 0
            self.selected_dev = NONE_DEV
            self.controllerChanged.emit()

        self.devicesUpdated.emit()

    @pyqtSlot()
    def build_list(self):
        self.widget.clear()
        self.widget.addItems(self.devices)
        for i in range(len(self.active)):
            if self.widget.item(i).text() == self.selected_dev:
                self.widget.item(i).setBackground(COLOR_BLUE)
                continue

            if self.active[i]:
                self.widget.item(i).setBackground(COLOR_RED)
            else:
                self.widget.item(i).setBackground(COLOR_BLACK)

    @pyqtSlot()
    def device_selected(self):
        if self.widget.currentItem().text() == self.selected_dev:
            return

        if self.selected_dev != NONE_DEV:
            self.sub_input.unregister()
            self.sub_input = 0

        self.selected_dev = self.widget.currentItem().text()
        self.controllerChanged.emit()
        for node in self.devices_dict.keys():
            if self.selected_dev in self.devices_dict[node]:
                self.sub_input = rospy.Subscriber(
                    node + "/" + self.selected_dev, Input, self.client_callback
                )

                break

    def shutdown(self):
        if self.sub_input != 0:
            self.sub_input.unregister()
        self.widget.clear()
        self.devices_sub.unregister()
