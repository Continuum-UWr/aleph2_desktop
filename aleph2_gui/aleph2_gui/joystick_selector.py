from collections.abc import Callable

from rclpy.node import Node
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from input_manager.msg import Input, DeviceList, DeviceInfo
from qtpy.QtCore import Slot, Signal, QObject, Qt
from qtpy.QtGui import QColor
from qtpy.QtWidgets import QListWidget, QListWidgetItem

# COLOR_BLUE = QColor("#3a3a99")
COLOR_RED = QColor("#bf2626")
COLOR_BLACK = QColor(0, 0, 0, 255)
NONE_DEV = "<None>"


class JoystickSelector(QObject):
    controllerChanged = Signal()
    devicesUpdated = Signal()

    devices_dict: dict[str, list[DeviceInfo]]
    selected_dev: str
    devices: list[str]
    active_dict: dict[str, list[bool]]
    active: list[bool]
    # selected_object: QListWidgetItem

    def __init__(
        self,
        node: Node,
        logger: RcutilsLogger,
        callback: Callable[[Input], None],
        widget: QListWidget,
    ):
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
        # self.selected_object = None

        self.sub_input = None
        self._widget.itemPressed.connect(self.slot_select_device)
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

        for node_name in self.devices_dict.keys():
            for device in self.devices_dict[node_name]:
                self.devices.append(device.name)
            self.active += self.active_dict[node_name]

        if self.selected_dev not in self.devices:
            self._node.destroy_subscription(self.sub_input)
            self.sub_input = None
            self.selected_dev = NONE_DEV
            self.controllerChanged.emit()

        self.devicesUpdated.emit()

    @Slot()
    def slot_rebuild_list(self):
        self._widget.clear()
        for device, active in zip(self.devices, self.active):
            item = QListWidgetItem(device)

            self._widget.addItem(item)

            if device == self.selected_dev:
                item.setSelected(True)
                continue

            if active:
                item.setBackground(COLOR_RED)
                item.setFlags(Qt.ItemFlag.NoItemFlags)

    @Slot(QListWidgetItem)
    def slot_select_device(self, item):
        text = item.text()

        if text == self.selected_dev:
            return

        if self.selected_dev != NONE_DEV:
            self._node.destroy_subscription(self.sub_input)
            self.sub_input = None

        self.selected_dev = text
        self.controllerChanged.emit()

        if self.selected_dev != NONE_DEV:
            self.sub_input = self._node.create_subscription(
                Input, "input/data/" + self.selected_dev, self._callback, 10
            )

    def shutdown(self):
        if self.sub_input is not None:
            self._node.destroy_subscription(self.sub_input)
        self._node.destroy_subscription(self.sub_devices)
