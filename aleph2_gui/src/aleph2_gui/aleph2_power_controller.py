import os

import rospy
import rospkg

from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import (
    QWidget,
    QGroupBox,
    QProgressBar,
    QLCDNumber,
    QHBoxLayout,
)
from python_qt_binding.QtCore import pyqtSlot, pyqtSignal

import aleph2_gui.resources.ta

from rubi_server.msg import RubiFloat, RubiBool
from aleph2_msgs.msg import PowerStatus


LIPO_CELL_COUNT = 6

HC_BOARD_CHANNEL_COUNT = 8
HC_BOARD_CHANNEL_NAMES = [
    "NANOTEC 1",
    "NANOTEC 2",
    "NANOTEC 3",
    "NANOTEC 4",
    "N/C",
    "N/C",
    "N/C",
    "LC BOARD",
]
HC_BOARD_SOFT_FUSE_LEVELS = [5, 5, 5, 5, 0, 0, 0, 5]
HC_BOARD_SOFT_FUSE_TRIPPED = [False, False, False, False, False, False, False, False]

LC_BOARD_CHANNEL_COUNT = 12
LC_BOARD_OUTPUT_NAMES = {
    "5v": ["OUTPUT {}".format(i) for i in range(1, 13)],
    "12v": ["OUTPUT {}".format(i) for i in range(1, 13)],
}

JETSON_RAILS_COUNT = 12


class Aleph2PowerController(Plugin):

    signal_lipo_cells_update = pyqtSignal()
    signal_hc_current_update = pyqtSignal()
    signal_lc_outputs_update = pyqtSignal()
    signal_power_status_update = pyqtSignal()

    def __init__(self, context):
        super(Aleph2PowerController, self).__init__(context)
        self.setObjectName("Aleph2PowerController")

        self._widget = QWidget()

        ui_file = os.path.join(
            rospkg.RosPack().get_path("aleph2_gui"),
            "resources/ui/aleph2_power_controller.ui",
        )
        self.ui_file_hc_delegate = os.path.join(
            rospkg.RosPack().get_path("aleph2_gui"),
            "resources/ui/aleph2_power_controller_hc_delegate.ui",
        )
        self.ui_file_lc_delegate = os.path.join(
            rospkg.RosPack().get_path("aleph2_gui"),
            "resources/ui/aleph2_power_controller_lc_delegate.ui",
        )

        self.ui_file_jetson_delegate = os.path.join(
            rospkg.RosPack().get_path("aleph2_gui"),
            "resources/ui/aleph2_power_controller_jetson_delegate.ui",
        )

        self.ui = loadUi(ui_file, self._widget)

        self._widget.setObjectName("hc_controller_ui")
        context.add_widget(self._widget)

        self.create_UI()

        self.lc_output_states = {
            "5v": [False] * LC_BOARD_CHANNEL_COUNT,
            "12v": [False] * LC_BOARD_CHANNEL_COUNT,
        }
        self.hc_output_states = [False] * HC_BOARD_CHANNEL_COUNT
        self.lipo_cells_voltage = [0.0] * LIPO_CELL_COUNT
        self.HC_board_current = [0.0] * 8
        self.hc_board_power_acc = [0.0] * 8

        self.power_status_rails = [""] * 12
        self.power_status_current = [0.0] * 12
        self.power_status_voltage = [0.0] * 12

        # Publishers
        self.lc_board_outputs_publishers = {"5v": [], "12v": []}
        for i in range(LC_BOARD_CHANNEL_COUNT):
            for v in "5v", "12v":
                self.lc_board_outputs_publishers[v].append(
                    rospy.Publisher(
                        "rubi/boards/kurwotron/fields_to_board/V{}_{}".format(
                            v[:-1], i + 1
                        ),
                        RubiBool,
                        queue_size=1,
                    )
                )

        self.hc_board_outputs_publisher = rospy.Publisher(
            "rubi/boards/HighPowerBoard/fields_to_board/power_cutoff",
            RubiBool,
            queue_size=1,
        )

        self.setup_signals()

        # Subscribers
        self.lc_board_outputs_subscribers = {"5v": [], "12v": []}
        for i in range(LC_BOARD_CHANNEL_COUNT):
            for v in "5v", "12v":
                self.lc_board_outputs_subscribers[v].append(
                    rospy.Subscriber(
                        "rubi/boards/kurwotron/fields_from_board/V{}_{}".format(
                            v[:-1], i + 1
                        ),
                        RubiBool,
                        self.get_lc_output_callback(v, i),
                    )
                )

        self.lipo_cells_subscriber = rospy.Subscriber(
            "rubi/boards/HighPowerBoard/fields_from_board/lipo_cells_voltage",
            RubiFloat,
            self.lipo_cells_voltage_callback,
        )

        self.HC_board_out_subscriber = rospy.Subscriber(
            "/rubi/boards/HighPowerBoard/fields_from_board/current",
            RubiFloat,
            self.HC_board_out_subscriber,
        )

        self.power_status_subscriber = rospy.Subscriber(
            "/power_status", PowerStatus, self.power_status_callback
        )

    def create_UI(self):

        # LIPO VOLTAGE WIDGET

        self.lipo_cells_subcontainers = []
        self.lipo_cells_volage_bars = []
        self.lipo_cells_volage_displays = []

        for i in range(LIPO_CELL_COUNT):
            cell = QGroupBox(self.ui.battery_level_container)
            self.ui.battery_level_layout.addWidget(cell)
            cell.setTitle("LIPO CELL {}".format(i + 1))
            cell.cell_layout = QHBoxLayout(cell)

            bar = QProgressBar(cell)
            bar.setRange(int(1000 * 3.0), int(1000 * 4.2))
            cell.cell_layout.addWidget(bar)
            self.lipo_cells_volage_bars.append(bar)

            num = QLCDNumber(cell)
            cell.cell_layout.addWidget(num)
            self.lipo_cells_volage_displays.append(num)
            self.lipo_cells_subcontainers.append(cell)

        # HC BOARD WIDGET

        self.hc_board_channel_subcontainers = []

        for i in range(HC_BOARD_CHANNEL_COUNT):
            subcontainer = QGroupBox(self.ui.hc_board_channel_container)
            subcontainer.setTitle(HC_BOARD_CHANNEL_NAMES[i])
            subcontainer.setCheckable(True)
            subcontainer.ui = loadUi(self.ui_file_hc_delegate, subcontainer)
            self.ui.hc_board_channel_layout.addWidget(subcontainer)
            self.hc_board_channel_subcontainers.append(subcontainer)

        # LC BOARD WIDGET

        self.lc_board_subcontainers = {"5v": [], "12v": []}

        for i in range(LC_BOARD_CHANNEL_COUNT):
            for v, container, layout in (
                ("5v", self.ui.lc_board5v_container, self.ui.lc_board5v_layout),
                ("12v", self.ui.lc_board12v_container, self.ui.lc_board12v_layout),
            ):
                subcontainer = QGroupBox(container)
                subcontainer.setTitle(LC_BOARD_OUTPUT_NAMES[v][i])
                subcontainer.setCheckable(True)
                subcontainer.ui = loadUi(self.ui_file_lc_delegate, subcontainer)
                layout.addWidget(subcontainer)
                self.lc_board_subcontainers[v].append(subcontainer)

        # JETSON POWER MONITOR WIDGET

        self.jetson_power_monitor_subcontainers = []

        for i in range(JETSON_RAILS_COUNT):
            subcontainer = QGroupBox(self.ui.jetson_channel_container)
            subcontainer.setTitle("a")
            subcontainer.ui = loadUi(self.ui_file_jetson_delegate, subcontainer)
            self.ui.jetson_power_monitor_layout.addWidget(subcontainer)
            self.jetson_power_monitor_subcontainers.append(subcontainer)

    def get_lc_output_state_changed_slot(self, voltage, output):
        @pyqtSlot(bool)
        def slot_lc_output_state_changed(state):
            self.lc_output_states[voltage][output] = not state
            msg = RubiBool([not state])
            self.lc_board_outputs_publishers[voltage][output].publish(msg)

        return slot_lc_output_state_changed

    def get_hc_output_state_changed_slot(self, output):
        @pyqtSlot(bool)
        def slot_hc_output_state_changed(state):
            self.hc_output_states[output] = state
            msg = RubiBool(self.hc_output_states)
            self.hc_board_outputs_publisher.publish(msg)

        return slot_hc_output_state_changed

    def get_lc_output_callback(self, voltage, output):
        def lc_output_callback(msg):
            if len(msg.data) > 0:
                self.lc_output_states[voltage][output] = msg.data[0]
            self.signal_lc_outputs_update.emit()

        return lc_output_callback

    def lipo_cells_voltage_callback(self, msg):
        self.lipo_cells_voltage = msg.data
        self.signal_lipo_cells_update.emit()

    def HC_board_out_subscriber(self, msg):
        self.hc_board_current = msg.data
        self.signal_hc_current_update.emit()

    def power_status_callback(self, msg):
        if "" in self.power_status_rails:
            self.power_status_rails = msg.rail
        self.power_status_current = msg.current
        self.power_status_voltage = msg.voltage
        self.signal_power_status_update.emit()

    @pyqtSlot()
    def slot_refresh_cells_voltage(self):
        total_voltage = sum(self.lipo_cells_voltage[:-1])
        self.ui.lipo_group_voltage.display(total_voltage)
        self.ui.lipo_group_voltage_progress_bar.setValue(
            min(int(1000 * total_voltage), 25200)
        )
        for cell_it, voltage in enumerate(self.lipo_cells_voltage[:-1]):
            self.lipo_cells_volage_displays[cell_it].display(voltage)
            self.lipo_cells_volage_bars[cell_it].setValue(
                min(int(1000 * voltage), 4200)
            )

    @pyqtSlot()
    def slot_refresh_hc_current(self):
        U = sum(self.lipo_cells_voltage[:-1])
        for i in range(8):
            self.hc_board_channel_subcontainers[i].ui.current_draw_LCD.display(
                self.hc_board_current[i]
            )
            self.hc_board_channel_subcontainers[i].ui.current_draw_bar.setValue(
                max(min(int(1000 * self.hc_board_current[i]), 5000), 0)
            )
            # assume, that the updates come once in a second.
            # calcullate the power, using W = U*I*T
            self.hc_board_power_acc[i] += (
                1.0 / 3600.0 * max(self.hc_board_current[i], 0.0) * U
            )
            self.hc_board_channel_subcontainers[i].ui.power_stat_LCD.display(
                self.hc_board_power_acc[i]
            )

    @pyqtSlot()
    def slot_refresh_power_monitor(self):
        for i in range(JETSON_RAILS_COUNT):
            self.jetson_power_monitor_subcontainers[i].ui.Current_LCD.display(
                self.power_status_current[i]
            )
            self.jetson_power_monitor_subcontainers[i].ui.Voltage_LCD.display(
                self.power_status_voltage[i]
            )
            self.jetson_power_monitor_subcontainers[i].setTitle(
                self.power_status_rails[i]
            )

    @pyqtSlot()
    def slot_refresh_lc_outputs(self):
        for i in range(LC_BOARD_CHANNEL_COUNT):
            for v in "5v", "12v":
                self.lc_board_subcontainers[v][i].setChecked(
                    not self.lc_output_states[v][i]
                )

    def setup_signals(self):
        self.signal_lipo_cells_update.connect(self.slot_refresh_cells_voltage)
        self.signal_hc_current_update.connect(self.slot_refresh_hc_current)
        self.signal_power_status_update.connect(self.slot_refresh_power_monitor)
        self.signal_lc_outputs_update.connect(self.slot_refresh_lc_outputs)
        for i in range(HC_BOARD_CHANNEL_COUNT):
            self.hc_board_channel_subcontainers[i].toggled.connect(
                self.get_hc_output_state_changed_slot(i)
            )
        for i in range(LC_BOARD_CHANNEL_COUNT):
            for v in "5v", "12v":
                self.lc_board_subcontainers[v][i].toggled.connect(
                    self.get_lc_output_state_changed_slot(v, i)
                )

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
