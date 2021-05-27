# todos
# when board removed -- clear_gui + refresh_boards
# better new window showing
# rubi_board.py?
# BoardAnnounce -- but problem with start (distrituted approach)
# maybe only one RC (return from _widget2)?
# 17 boards -> scroll disapears, tf?;
# all magic values to other file
# include rubi_autodefs? regex
# RubiController scrollarea resize right
# todo publishers at end?
# chb -> is_online, cb2 -> is_wake
# connect buttions when rubi_controller out
import os
import sys
import copy

from math import ceil
from functools import partial
from collections import OrderedDict

import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QLabel, QCheckBox, QPushButton
from python_qt_binding.QtCore import pyqtSlot, pyqtSignal, Qt
from python_qt_binding.QtGui import QPixmap

from std_msgs.msg import Empty, Float32MultiArray
from rubi_server.msg import RubiInt, RubiUnsignedInt, RubiFloat, RubiString, RubiBool
from rubi_server.srv import (
    ShowBoards,
    BoardDescriptor,
    BoardInstances,
    FieldDescriptor,
    FuncDescriptor,
    CansNames,
    BoardOnline,
    BoardWake,
)

import aleph2_gui.resources.ta
from .rubi_interface_builder import RubiInterfaceBuilder
from .rubi_board import QTextEditEventFilter, QSpinBoxEventFilter


class RubiController(Plugin):
    (CFL, CSF, CTC, CIN, COT, CFN, CAN, CTI, CTO) = (
        "fields",
        "subfields",
        "typecode",
        "in",
        "out",
        "functions",
        "arg_names",
        "type_in",
        "type_out",
    )
    PUB_QSIZE = 10
    UINT_EXP = [0, 0, 31, 15, 7, 32, 16, 8]
    FLOAT_EXP = 15
    FLOAT_PREC = 5
    refresh_boards_signal = pyqtSignal()
    safe_update_signal = pyqtSignal(type(lambda: ()))

    TYPECODE_TO_TYPE = {
        1: None,
        2: RubiInt,
        3: RubiInt,
        4: RubiInt,
        5: RubiUnsignedInt,
        6: RubiUnsignedInt,
        7: RubiUnsignedInt,
        8: RubiFloat,
        9: RubiString,
        10: RubiString,
        11: RubiBool,
    }

    def rubi_controller_panel(self, name):
        return self._widget.findChild(QWidget, name)

    def board_controller_panel(self, name):
        return self._widget2.findChild(QWidget, name)

    def subfields_val_cb(self, info, len, data):
        for num in range(0, len):
            if isinstance(data.data[0], float):
                info["val" + str(num)].setText("{:.2f}".format(data.data[num]))
            else:
                info["val" + str(num)].setText(str(data.data[num]))

    def subfields_bool_cb(self, info, len, data):
        for num in range(0, len):
            info["chb" + str(num)].setChecked(data.data[num])

    @pyqtSlot(type(lambda: ()))
    def safe_execute(self, func):
        func()

    def str_cb(self, string, data):
        string.setText(str(data.data[0]))

    def bool_cb(self, bool, data):
        bool.setChecked(data.data[0])

    def load_cb(self, data):
        for (can_name, can_load) in zip(self.cans, data.data):
            lbl_load = self.rubi_controller_panel("load_" + can_name)
            lbl_load.setText(str(int(ceil(can_load / 1000.0))) + "k")

    @pyqtSlot()
    def state_clicked(self, pub):
        msg = Empty()
        pub.publish(msg)

    @pyqtSlot()
    def panic_clicked(self):
        msg = Empty()
        self.panic_pub.publish(msg)

    @pyqtSlot()
    def subfields_spn_changed(self, info, len, pub, typecode):
        val = 0
        if typecode == 8:
            val = RubiFloat()
        elif 2 <= typecode <= 4:
            val = RubiInt()
        else:
            val = RubiUnsignedInt()
        for num in range(0, len):
            val.data += [info["spn" + str(num)].value()]
        pub.publish(val)

    @pyqtSlot()
    def subfields_chb_changed(self, info, len, pub):
        val = RubiBool()
        for num in range(0, len):
            val.data += [info["chb" + str(num)].isChecked()]
        pub.publish(val)

    def val_cb(self, val, data):
        # rospy.loginfo('val %s, currTh %s, as int %s' % (val, QThread.currentThread(), int(QThread.currentThreadId())))
        val.setText(str(data.data[0]))

    @pyqtSlot()
    def spn_changed(self, spn, pub, typecode):
        val = 0
        if typecode == 8:
            val = RubiFloat()
        elif 2 <= typecode <= 4:
            val = RubiInt()
        else:
            val = RubiUnsignedInt()
        val.data = [spn.value()]
        pub.publish(val)

    @pyqtSlot()
    def chb_changed(self, chb, pub):
        val = RubiBool()
        val.data = [chb.isChecked()]
        pub.publish(val)

    @pyqtSlot()
    def show_board_window(self, board):
        self.context.remove_widget(self._widget)
        self.context.add_widget(self._widget2)
        # self._widget2.setVisible(True)
        # rospy.loginfo('RC: GUI for board %s showed\n%s', board, self.boards[board])

        pub_reboot = rospy.Publisher(
            "/rubi/boards/%s/reboot" % board, Empty, queue_size=self.PUB_QSIZE
        )
        pub_sleep = rospy.Publisher(
            "/rubi/boards/%s/sleep" % board, Empty, queue_size=self.PUB_QSIZE
        )
        pub_wake = rospy.Publisher(
            "/rubi/boards/%s/wake" % board, Empty, queue_size=self.PUB_QSIZE
        )
        self.board_controller_panel("lbl_reboot").clicked.connect(
            partial(self.state_clicked, pub_reboot)
        )
        self.board_controller_panel("lbl_sleep").clicked.connect(
            partial(self.state_clicked, pub_sleep)
        )
        self.board_controller_panel("lbl_wake").clicked.connect(
            partial(self.state_clicked, pub_wake)
        )
        self.board_controller_panel("lbl_board_name").setText(board)
        base_next_y = 5
        base_move_y = 30
        base_geo_spacing = 15
        base_geo_spn_y = 10
        base_geo_str_y = 35
        base_geo_str_spacing = -10
        base_geo_tln_next = 115
        base_alb_x = 5  # label, checkbox -- alone field + subfields, bool
        base_alb_dim = (240, 41)
        base_olb_x = 5  # label -- overfield
        base_olb_dim = (330, 41)
        base_vlb_x = 265  # label -- read only val
        base_vlb_dim = (70, 41)
        base_spn_x = 270  # spin, double spin
        base_spn_dim = (62, 22)
        base_tsh_x = 80  # text -- short
        base_tsh_dim = (261, 30)
        base_tln_x = 80  # text -- long
        base_tln_dim = (261, 140)
        wid = self.board_controller_panel("bScrArWC")

        builder = RubiInterfaceBuilder()

        for i, field in enumerate(self.boards[board]["fields"]):
            read_topic_name = "/rubi/boards/" + board + "/fields_to_board/" + field
            write_topic_name = "/rubi/boards/" + board + "/fields_from_board/" + field
            infovc = self.boards[board]["valueChanged"]
            infosc = self.boards[board]["stateChanged"]
            info = self.boards[board]["fields"][field]

            ros_pub_handler = None

            if info["in"]:
                read_topic = rospy.Publisher(
                    read_topic_name,
                    self.TYPECODE_TO_TYPE[info["typecode"]],
                    queue_size=self.PUB_QSIZE,
                )

                def ros_pub_handler(
                    fields, read_topic=read_topic, info=info, field=field
                ):
                    return read_topic.publish(
                        self.TYPECODE_TO_TYPE[info["typecode"]](fields)
                    )

            write_handler = builder.build_field(
                wid,
                field,
                info["typecode"],
                ros_pub_handler,
                info["out"],
                info["subfields"],
            )

            if info["out"]:
                print(write_handler, info["out"])
                ros_sub_handler = (
                    lambda data, h=write_handler: self.safe_update_signal.emit(
                        lambda: h(data.data)
                    )
                )
                sub = rospy.Subscriber(
                    write_topic_name,
                    self.TYPECODE_TO_TYPE[info["typecode"]],
                    ros_sub_handler,
                )

        # sleep/wake part
        nope = QLabel(wid)
        pixmap = QPixmap(":/ta-skin/sleep_nope.png")
        nope.setPixmap(pixmap)
        nope.setScaledContents(True)
        nope.setFixedWidth(base_olb_dim[0])
        nope.setFixedHeight(base_next_y)
        nope.move(base_olb_x, base_geo_spn_y)
        nope.setAttribute(Qt.WA_TranslucentBackground, True)
        th = None
        try:
            th = self.boards[board]["srv_wake"]().wake
        except Exception as e:
            rospy.logwarn(
                "RC: show_board_window(): exception during srv_wake for board %s, e=%s",
                board,
                e,
            )
            th = False
        # todo why no thread crash?
        # rospy.logerr("th %s", th)
        nope.setVisible(not th)
        self.boards[board]["nope"] = nope

        wid.setMinimumHeight(base_next_y)

        self.timers += [
            rospy.Timer(rospy.Duration(2), partial(self.watch_board_cb, board))
        ]

    @pyqtSlot()
    def clear_gui(self):
        pass
        # rospy.loginfo("RC: GUI gewascht :3 nanaa~ todo motzno")
        # wid = self.BoardsPanel("bScrArWC")
        # for board in self.curr_gui_boards:

    @pyqtSlot()
    def refresh_boards(self):
        # rospy.loginfo("RC: GUI wird neugeladet! :3")
        self.clear_gui()

        # cans part
        base_can_move = 30
        base_can_y = 50
        base_can_name_x = 20
        base_can_name_dim = (91, 41)
        base_can_load_x = 110
        base_can_load_dim = (61, 41)
        info_panel = self.rubi_controller_panel("INFO")
        can_load = "/rubi/cans_load/"
        # todo cans could change in runtime (after driver reset)
        if not self.cans:
            self.cans = self.get_cans_names()
        for can in self.cans:
            name = QLabel(info_panel)
            name.setObjectName("name_" + can)
            name.setFixedWidth(base_can_name_dim[0])
            name.setFixedHeight(base_can_name_dim[1])
            name.setText(can)
            name.move(base_can_name_x, base_can_y)
            name.setVisible(True)
            load = QLabel(info_panel)
            load.setObjectName("load_" + can)
            load.setFixedWidth(base_can_load_dim[0])
            load.setFixedHeight(base_can_load_dim[1])
            load.setText("")
            load.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            load.move(base_can_load_x, base_can_y)
            load.setVisible(True)
            base_can_y += base_can_move

        sub = rospy.Subscriber(can_load, Float32MultiArray, self.load_cb)
        self.subscribers += [sub]
        info_panel.setMinimumHeight(111 + base_can_move * len(self.cans))

        self.rubi_controller_panel("btn_panic").move(
            base_can_move, base_can_move * (len(self.cans) + 2)
        )
        self.rubi_controller_panel("btn_panic").clicked.connect(self.panic_clicked)

        # boards part
        base_geo_move = 30
        base_chb_xy = [8, 4]  # checkbox -- online
        base_chb_dim = (36, 41)
        base_cb2_xy = [38, 4]  # checkbox2 -- wake
        base_cb2_dim = (36, 41)
        base_lbl_xy = [68, 4]  # label
        base_lbl_dim = (161, 41)
        base_pb_xy = [230, 10]  # pushbutton
        base_pb_dim = (91, 31)
        wid = self.rubi_controller_panel("bScrArWC")
        for board in self.boards:
            if board in self.curr_gui_boards:
                # rospy.loginfo("%s moved to %s", board, base_chb_xy[1])
                self.boards[board]["chb"].move(base_chb_xy[0], base_chb_xy[1])
                self.boards[board]["cb2"].move(base_cb2_xy[0], base_cb2_xy[1])
                self.boards[board]["lbl"].move(base_lbl_xy[0], base_lbl_xy[1])
                self.boards[board]["pb"].move(base_pb_xy[0], base_pb_xy[1])
            else:
                # rospy.loginfo("%s new at %s", board, base_chb_xy[1])
                self.boards[board]["chb"] = QCheckBox(wid)
                self.boards[board]["chb"].setObjectName("chb_" + board)
                self.boards[board]["chb"].setFixedWidth(base_chb_dim[0])
                self.boards[board]["chb"].setFixedHeight(base_chb_dim[1])
                self.boards[board]["chb"].move(base_chb_xy[0], base_chb_xy[1])
                self.boards[board]["chb"].setCheckable(True)
                self.boards[board]["chb"].setChecked(False)
                self.boards[board]["chb"].setEnabled(False)
                self.boards[board]["chb"].setVisible(True)
                self.boards[board]["srv_online"] = rospy.ServiceProxy(
                    "/rubi/boards/" + board + "/is_online", BoardOnline
                )
                self.boards[board]["cb2"] = QCheckBox(wid)
                self.boards[board]["cb2"].setObjectName("cb2_" + board)
                self.boards[board]["cb2"].setFixedWidth(base_cb2_dim[0])
                self.boards[board]["cb2"].setFixedHeight(base_cb2_dim[1])
                self.boards[board]["cb2"].move(base_cb2_xy[0], base_cb2_xy[1])
                self.boards[board]["cb2"].setCheckable(True)
                self.boards[board]["cb2"].setChecked(False)
                self.boards[board]["cb2"].setEnabled(False)
                self.boards[board]["cb2"].setVisible(True)
                self.boards[board]["srv_wake"] = rospy.ServiceProxy(
                    "/rubi/boards/" + board + "/is_wake", BoardWake
                )

                self.boards[board]["lbl"] = QLabel(wid)
                self.boards[board]["lbl"].setObjectName("lbl_" + board)
                self.boards[board]["lbl"].setFixedWidth(base_lbl_dim[0])
                self.boards[board]["lbl"].setFixedHeight(base_lbl_dim[1])
                self.boards[board]["lbl"].move(base_lbl_xy[0], base_lbl_xy[1])
                self.boards[board]["lbl"].setText(board)
                self.boards[board]["lbl"].setVisible(True)
                self.boards[board]["pb"] = QPushButton(wid)
                self.boards[board]["pb"].setObjectName("pb_" + board)
                self.boards[board]["pb"].setFixedWidth(base_pb_dim[0])
                self.boards[board]["pb"].setFixedHeight(base_pb_dim[1])
                self.boards[board]["pb"].move(base_pb_xy[0], base_pb_xy[1])
                self.boards[board]["pb"].setText("CONNECT")
                self.boards[board]["pb"].clicked.connect(
                    partial(self.show_board_window, board)
                )
                self.boards[board]["pb"].setVisible(True)

                self.curr_gui_boards += [board]

                min_heig = base_lbl_xy[1] + base_lbl_dim[1]
                wid.setMinimumHeight(min_heig)

            base_chb_xy[1] += base_geo_move
            base_cb2_xy[1] += base_geo_move
            base_lbl_xy[1] += base_geo_move
            base_pb_xy[1] += base_geo_move

            # rospy.loginfo("board a out %s", self.boards['a']['pb'])
        # rospy.loginfo("board a out %s", self.curr_gui_boards)

        # if (base_cb_xy[1] > ..):
        # resize scr_area
        # rospy.loginfo("RC: boards panel: %s", self._widget.BOARDS)
        # self.InputPanel("ISENSITIVITY").setValue(self.sensitivity_level)
        # self.MainPanel("LUPS").setText(str(self.status.ups))
        # self.Widget2("BBR").setChecked(False)
        # wid = self.rubi_controller_panel("pb_FakeBoard1")
        # wid2 = self.rubi_controller_panel("pb_FakeBoard2")
        # rospy.loginfo("pb fbs: %s -- %s", wid, wid2)

    @pyqtSlot()
    def check_boards(self):
        # self.RubiControllerPanel("chb_online").setChecked(False)
        try:
            rospy.wait_for_service("/rubi/show_boards", timeout=1)
            show_boards = rospy.ServiceProxy("/rubi/show_boards", ShowBoards)
            boards_names = show_boards().boards_names

            rospy.wait_for_service("/rubi/get_board_instances", timeout=1)
            board_instances = rospy.ServiceProxy(
                "/rubi/get_board_instances", BoardInstances
            )

            resp = []
            for bn in boards_names:
                for id in board_instances(bn).ids:
                    if id == "":
                        instance_name = bn
                    else:
                        instance_name = bn + "/" + id

                    name = {"instance_name": instance_name, "descriptor_name": bn}
                    resp.append(name)

            self.rubi_controller_panel("chb_online").setChecked(True)
            return resp
        except rospy.ROSException:
            self.rubi_controller_panel("chb_online").setChecked(False)
        except rospy.ServiceException as e:
            rospy.logwarn("RC: ros services call failed: %s", e)

    @pyqtSlot()
    def get_board_info(self, board_name):
        rospy.loginfo("RC: trying get info about %s", board_name)
        try:
            info = OrderedDict([(self.CFL, OrderedDict()), (self.CFN, OrderedDict())])
            board_info = rospy.ServiceProxy(
                "/rubi/get_board_descriptor", BoardDescriptor
            )
            bi = board_info(board_name)
            info["description"] = bi.description
            info["version"] = bi.version
            info["driver"] = bi.driver
            for field in bi.fields:
                field_info = rospy.ServiceProxy(
                    "/rubi/get_field_descriptor", FieldDescriptor
                )
                fi = field_info(board_name, field)
                info[self.CFL][field] = {
                    self.CSF: fi.subfields,
                    self.CTC: fi.typecode,
                    self.CIN: fi.input,
                    self.COT: fi.output,
                }
            for func in bi.functions:
                func_info = rospy.ServiceProxy(
                    "/rubi/get_func_descriptor", FuncDescriptor
                )
                fi = func_info(board_name, func)
                info[self.CFN][func] = {
                    self.CAN: fi.arg_names,
                    self.CTI: fi.type_in,
                    self.CTO: fi.type_out,
                }
            info["valueChanged"] = []
            info["stateChanged"] = []
            return info
        except rospy.ServiceException as e:
            rospy.logwarn("RC: some /rubi/get_descriptor call failed: %s", e)

    @pyqtSlot()
    def get_cans_names(self):
        rospy.loginfo("RC: trying get names of cans")
        try:
            names = rospy.ServiceProxy("/rubi/get_cans_names", CansNames)
            return names().names
        except rospy.ServiceException as e:
            rospy.logwarn("RC: some /rubi/get_cans_names call failed: %s", e)

    # def board_announ_cb(self, msg):
    #     rospy.loginfo("RC: new board announced: %s", msg)
    #     self.refresh_boards_signal.emit()

    def check_boards_cb(self, event):
        # rospy.loginfo("RC: periodic boards check")
        for board in self.boards:
            if "chb" in self.boards[board] and "cb2" in self.boards[board]:
                try:
                    th = self.boards[board]["srv_online"]().online
                    self.boards[board]["chb"].setChecked(th)
                except rospy.ServiceException:
                    self.boards[board]["chb"].setChecked(False)
                try:
                    # todo why no thread crash?
                    th = self.boards[board]["srv_wake"]().wake
                    # rospy.logerr("th cb %s", th)
                    self.boards[board]["cb2"].setChecked(th)
                except rospy.ServiceException:
                    self.boards[board]["cb2"].setChecked(False)

        boards_names = self.check_boards()
        if boards_names:
            changed = 0
            for board in boards_names:
                board_name = board["instance_name"]
                if board_name not in self.boards:
                    self.boards[board_name] = self.get_board_info(
                        board["descriptor_name"]
                    )
                    changed += 1
            if changed > 0:
                self.refresh_boards_signal.emit()
        else:
            for board in self.boards:
                if "nope" in self.boards[board]:
                    self.boards[board]["nope"].setVisible(True)

    def watch_board_cb(self, board, event):
        rospy.loginfo("RC: periodic %s watch" % board)
        curr_watch = False
        try:
            curr_watch = self.boards[board]["srv_online"]().online
            self.boards[board]["nope"].setVisible(
                not self.boards[board]["srv_wake"]().wake
            )
        except rospy.ServiceException:
            curr_watch = False
        # todo
        # if not self.last_watch and curr_watch:
        # for vc in self.boards[board]['valueChanged']:
        # QMetaObject.invokeMethod(vc, "valueChanged", Qt.QueuedConnection, Q_ARG(double, 0.))
        # QMetaObject.invokeMethod(vc, "valueChanged", Qt.QueuedConnection, Q_ARG(int, 0))
        # for sc in self.boards[board]['stateChanged']:
        # QMetaObject.invokeMethod(sc, "stateChanged", Qt.QueuedConnection, Q_ARG(int, 0))
        self.last_watch = curr_watch

    def __init__(self, context):
        super(RubiController, self).__init__(context)
        self.boards = OrderedDict()
        self.curr_gui_boards = []
        self.timers = []
        self.subscribers = []
        self.srv_online_check = []
        self.context = context
        self.cans = []
        self.last_watch = True
        self.panic_pub = rospy.Publisher(
            "/rubi/panic", Empty, queue_size=self.PUB_QSIZE
        )
        self.qet_ef = QTextEditEventFilter()
        self.qsb_ef = QSpinBoxEventFilter()

        self.setObjectName("RubiController")

        self._widget = QWidget()
        ui_file = os.path.join(
            rospkg.RosPack().get_path("aleph2_gui"), "resources/ui/rubi_controller.ui"
        )
        loadUi(ui_file, self._widget)
        # self._widget.setObjectName('RubiControllerUi')

        self._widget2 = QWidget()
        ui_file2 = os.path.join(
            rospkg.RosPack().get_path("aleph2_gui"),
            "resources/ui/rubi_controller_board.ui",
        )
        loadUi(ui_file2, self._widget2)
        # self._widget2.setObjectName('BoardControllerUi')

        self.refresh_boards_signal.connect(self.refresh_boards)
        self.safe_update_signal.connect(self.safe_execute)

        # self.subscribers += [rospy.Subscriber("/rubi/new_boards",
        #                       BoardAnnounce, self.board_announ_cb)]

        self.timers += [rospy.Timer(rospy.Duration(1), self.check_boards_cb)]

        if self.context.serial_number() > 1:
            self._widget.setWindowTitle(
                "{} ({})".format(
                    self._widget.windowTitle(), self.context.serial_number()
                )
            )
            self._widget2.setWindowTitle(
                "{} ({})".format(
                    self._widget2.windowTitle(), self.context.serial_number()
                )
            )
        self.context.add_widget(self._widget)
        # self.context.add_widget(self._widget2)
        # self._widget2.setVisible(False)

    def shutdown_plugin(self):
        for subscriber in self.subscribers:
            subscriber.unregister()
        for timer in self.timers:
            timer.shutdown()
