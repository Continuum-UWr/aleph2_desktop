from rclpy.node import Node
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from input_manager.msg import Input
from qtpy.QtCore import pyqtSlot, pyqtSignal, QObject
from qtpy.QtGui import QColor
from qtpy.QtWidgets import QListWidget

COLOR_BLUE = QColor("#3a3a99")
COLOR_RED = QColor("#bf2626")
COLOR_BLACK = QColor(0, 0, 0, 0)
NONE_DEV = "<None>"


class JoystickSelector(QObject):
    controllerChanged = pyqtSignal()
    devicesUpdated = pyqtSignal()

    def __init__(self, node: Node, logger: RcutilsLogger, callback, widget: QWidget):
        super(JoystickSelector, self).__init__()
        self._node = node
        self._widget = widget
        self._callback = callback
        self._logger = logger.get_child("JoystickSelector")

        self.devices_dict = {}
        self.selected_dev = NONE_DEV
        self.devices = [NONE_DEV]
        self.active_dict = {}
        self.active = [False]
        self.selected_object = None

        self.sub_input = None
        self._widget.itemClicked.connect(self.slot_select_device)
        self.devicesUpdated.connect(self.slot_rebuild_list)
        self.controllerChanged.connect(self.slot_rebuild_list)

        devices_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.sub_devices = self._node.create_subscription(
            DeviceList, "input/devices", self.devices_callback, devices_qos
        )

        self.devicesUpdated.emit()

    def devices_callback(self, data: DeviceList):
        name = data.node_name
        self.devices_dict[name] = data.devices
        self.devices = [NONE_DEV]
        self.active_dict[name] = data.active
        self.active = [False]

        for k in self.devices_dict.keys():
            self.devices += self.devices_dict[k]
            self.active += self.active_dict[k]

        if self.selected_dev not in self.devices:
            self._node.destroy_subscription(self.sub_input)
            self.sub_input = None
            self.selected_dev = NONE_DEV
            self.controllerChanged.emit()

        self.devicesUpdated.emit()

    @pyqtSlot()
    def slot_rebuild_list(self):
        self._widget.clear()
        self._widget.addItems(self.devices)
        for i in range(len(self.active)):
            if self._widget.item(i).text() == self.selected_dev:
                self.selected_object = self._widget.item(i)
                self.selected_object.setBackground(COLOR_BLUE)
                continue

            if self.active[i]:
                self._widget.item(i).setBackground(COLOR_RED)
            else:
                self._widget.item(i).setBackground(COLOR_BLACK)

    @pyqtSlot()
    def slot_select_device(self):
        text = self._widget.currentItem().text()
        clicked_index = -1
        try:
            clicked_index = self.devices.index(text)
        except ValueError:
            self._logger.error(
                f'The selected device "{text}" was not on the device list (possible race condition)'
            )
            return

        if self.active[clicked_index]:
            self._widget.setCurrentItem(self.selected_object)
            self.controllerChanged.emit()
            self._logger.warn(f'The selected device "{text}" is already in use')
            return

        if text == self.selected_dev:
            return

        if self.selected_dev != NONE_DEV:
            self._node.destroy_subscription(self.sub_input)
            self.sub_input = None

        self.selected_dev = text
        self.controllerChanged.emit()
        for node in self.devices_dict.keys():
            if self.selected_dev in self.devices_dict[node]:
                self.sub_input = self._node.create_subscription(
                    Input, "input/data/" + self.selected_dev, self._callback, 10
                )
                break

    def shutdown(self):
        if self.sub_input is not None:
            self._node.destroy_subscription(self.sub_input)
        self._node.destroy_subscription(self.sub_devices)