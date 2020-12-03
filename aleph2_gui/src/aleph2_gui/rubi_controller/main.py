import os
import sys
from math import ceil
from functools import partial
from collections import OrderedDict

import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QCheckBox, QLabel, \
    QPushButton, QSpinBox, QDoubleSpinBox, QTextEdit
from python_qt_binding.QtCore import pyqtSlot, pyqtSignal, QEvent, QMetaObject
from python_qt_binding.QtGui import QPixmap

from std_msgs.msg import Empty, Float32MultiArray
from rubi_server.msg import RubiInt, RubiFloat, RubiString, RubiBool
from rubi_server.srv import BoardOnline, BoardWake
from rubi_server.srv import ShowBoards, BoardDescriptor, \
    BoardInstances, FieldDescriptor, FuncDescriptor, CansNames

import aleph2_gui.resources.ta
from .rubi_board import QTextEditEventFilter, QSpinBoxEventFilter


class RubiController(Plugin):
    (CFL, CSF, CTC, CIN, COT,
     CFN, CAN, CTI, CTO) = (
        'fields', 'subfields', 'typecode', 'in', 'out',
        'functions', 'arg_names', 'type_in', 'type_out'
    )
    PUB_QSIZE = 10
    UINT_EXP = [0, 0, 31, 15, 7, 32, 16, 8]
    FLOAT_EXP = 15
    FLOAT_PREC = 5
    refresh_boards_signal = pyqtSignal()

    def RubiControllerPanel(self, name):
        return self._widget.findChild(QWidget, name)

    def BoardControllerPanel(self, name):
        return self._widget2.findChild(QWidget, name)

    def subfields_val_cb(self, info, len, data):
        for num in range(0, len):
            if isinstance(data.data[0], float):
                info['val' + str(num)].setText("{:.2f}".format(data.data[num]))
            else:
                info['val' + str(num)].setText(str(data.data[num]))

    def subfields_bool_cb(self, info, len, data):
        for num in range(0, len):
            info['chb' + str(num)].setChecked(data.data[num])

    def str_cb(self, str, data):
        str.setText(str(data.data[0]))

    def bool_cb(self, bool, data):
        bool.setChecked(data.data[0])

    def load_cb(self, data):
        for (can_name, can_load) in zip(self.cans, data.data):
            lbl_load = self.RubiControllerPanel("load_" + can_name)
            lbl_load.setText(str(int(ceil(can_load / 1000.))) + 'k')

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
        else:
            val = RubiInt()
        for num in range(0, len):
            val.data += [info['spn' + str(num)].value()]
        pub.publish(val)

    @pyqtSlot()
    def subfields_chb_changed(self, info, len, pub):
        val = RubiBool()
        for num in range(0, len):
            val.data += [info['chb' + str(num)].isChecked()]
        pub.publish(val)

    def val_cb(self, val, data):
        val.setText(str(data.data[0]))

    @pyqtSlot()
    def spn_changed(self, spn, pub, typecode):
        val = 0
        if typecode == 8:
            val = RubiFloat()
        else:
            val = RubiInt()
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

        pub_reboot = rospy.Publisher('/rubi/boards/%s/reboot' % board, Empty, queue_size=self.PUB_QSIZE)
        pub_sleep = rospy.Publisher('/rubi/boards/%s/sleep' % board, Empty, queue_size=self.PUB_QSIZE)
        pub_wake = rospy.Publisher('/rubi/boards/%s/wake' % board, Empty, queue_size=self.PUB_QSIZE)
        self.BoardControllerPanel("lbl_reboot").clicked.connect(partial(self.state_clicked, pub_reboot))
        self.BoardControllerPanel("lbl_sleep").clicked.connect(partial(self.state_clicked, pub_sleep))
        self.BoardControllerPanel("lbl_wake").clicked.connect(partial(self.state_clicked, pub_wake))
        self.BoardControllerPanel("lbl_board_name").setText(board)
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
        wid = self.BoardControllerPanel("bScrArWC")
        for field in self.boards[board]['fields']:
            rubi_in = '/rubi/boards/' + board + '/fields_to_board/' + field
            rubi_out = '/rubi/boards/' + board + '/fields_from_board/' + field
            infovc = self.boards[board]['valueChanged']
            infosc = self.boards[board]['stateChanged']
            info = self.boards[board]['fields'][field]
            # rospy.loginfo(">%s", info)
            subfields_len = len(info['subfields'])

            # subfields part
            if subfields_len > 0:
                info['olb'] = QLabel(wid)
                info['olb'].setObjectName("olb_" + field)
                info['olb'].setFixedWidth(base_olb_dim[0])
                info['olb'].setFixedHeight(base_olb_dim[1])
                info['olb'].move(base_olb_x, base_next_y)
                info['olb'].setText(field)
                info['olb'].setVisible(True)
                info['sub'] = info['pub'] = 0
                base_next_y += base_move_y
                if 2 <= info['typecode'] <= 8:
                    # RUBI_WRITEONLY
                    if not info['in']:
                        for (num, subfield) in enumerate(info['subfields']):
                            lbl = 'lbl' + str(num)
                            val = 'val' + str(num)
                            info[lbl] = QLabel(wid)
                            info[lbl].setObjectName("lbl_" + field + "_" + str(num))
                            info[lbl].setFixedWidth(base_alb_dim[0])
                            info[lbl].setFixedHeight(base_alb_dim[1])
                            info[lbl].move(base_alb_x, base_next_y)
                            info[lbl].setText(subfield)
                            info[lbl].setVisible(True)
                            info[val] = QLabel(wid)
                            info[val].setObjectName("val_" + field + "_" + str(num))
                            info[val].setFixedWidth(base_vlb_dim[0])
                            info[val].setFixedHeight(base_vlb_dim[1])
                            info[val].move(base_vlb_x, base_next_y)
                            info[val].setText('out')
                            info[val].setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                            info[val].setVisible(True)
                            base_next_y += base_move_y
                        base_next_y -= base_move_y
                    # RUBI_READONLY or RUBI_READWRITE
                    else:
                        for (num, subfield) in enumerate(info['subfields']):
                            lbl = 'lbl' + str(num)
                            spn = 'spn' + str(num)
                            info[lbl] = QLabel(wid)
                            info[lbl].setObjectName("lbl_" + field + "_" + str(num))
                            info[lbl].setFixedWidth(base_alb_dim[0])
                            info[lbl].setFixedHeight(base_alb_dim[1])
                            info[lbl].move(base_alb_x, base_next_y)
                            info[lbl].setText(subfield)
                            info[lbl].setVisible(True)
                            info[spn] = 0
                            # float field
                            if info['typecode'] == 8:
                                info[spn] = QDoubleSpinBox(wid)
                                info[spn].setRange(-2**self.FLOAT_EXP, 2**self.FLOAT_EXP-1)
                                info[spn].setDecimals(self.FLOAT_PREC)
                            # (u)int field
                            else:
                                info[spn] = QSpinBox(wid)
                                tc = info['typecode']
                                # int field
                                if 2 <= tc <= 4:
                                    info[spn].setRange(-2**self.UINT_EXP[tc], 2**self.UINT_EXP[tc]-1)
                                # uint field
                                else:
                                    info[spn].setRange(0, 2**self.UINT_EXP[tc])
                            info[spn].setObjectName("spn_" + field + "_" + str(num))
                            info[spn].setFixedWidth(base_spn_dim[0])
                            info[spn].setFixedHeight(base_spn_dim[1])
                            info[spn].move(base_spn_x, base_next_y + base_geo_spn_y)
                            info[spn].setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                            info[spn].setKeyboardTracking(False)
                            info[spn].setVisible(True)
                            base_next_y += base_move_y
                        base_next_y -= base_move_y
                    if not info['in']:
                        if info['typecode'] == 8:
                            info['sub'] = rospy.Subscriber(rubi_out, RubiFloat, partial(self.subfields_val_cb, info, subfields_len))
                        else:
                            info['sub'] = rospy.Subscriber(rubi_out, RubiInt, partial(self.subfields_val_cb, info, subfields_len))
                        self.subscribers += [info['sub']]
                    else:
                        if info['typecode'] == 8:
                            info['pub'] = rospy.Publisher(rubi_in, RubiFloat, queue_size=self.PUB_QSIZE)
                        else:
                            info['pub'] = rospy.Publisher(rubi_in, RubiInt, queue_size=self.PUB_QSIZE)
                        for num in range(0, subfields_len):
                            info['spn' + str(num)].valueChanged.connect(partial(self.subfields_spn_changed, info, subfields_len, info['pub'], info['typecode']))
                            infovc += [info['spn' + str(num)]]
                elif info['typecode'] == 11:
                    for (num, subfield) in enumerate(info['subfields']):
                        chb = 'chb' + str(num)
                        info[chb] = QCheckBox(wid)
                        info[chb].setObjectName("chb_" + field + "_" + str(num))
                        info[chb].setFixedWidth(base_alb_dim[0])
                        info[chb].setFixedHeight(base_alb_dim[1])
                        info[chb].move(base_alb_x, base_next_y)
                        info[chb].setText(subfield)
                        info[chb].setCheckable(True)
                        info[chb].setEnabled(False)
                        info[chb].setVisible(True)
                        if info['in']:
                            info[chb].setEnabled(True)
                            info['pub'] = rospy.Publisher(rubi_in, RubiBool, queue_size=self.PUB_QSIZE)
                            info[chb].stateChanged.connect(partial(self.subfields_chb_changed, info, subfields_len, info['pub']))
                            infosc += [info[chb]]
                        base_next_y += base_move_y
                    if not info['in']:
                        info['sub'] = rospy.Subscriber(rubi_out, RubiBool, partial(self.subfields_bool_cb, info, subfields_len))
                        self.subscribers += [info['sub']]
                    base_next_y -= base_move_y
            # normal field part
            # (u)int or float field
            elif 2 <= info['typecode'] <= 8:
                # RUBI_WRITEONLY
                if not info['in']:
                    info['lbl'] = QLabel(wid)
                    info['lbl'].setObjectName("lbl_" + field)
                    info['lbl'].setFixedWidth(base_alb_dim[0])
                    info['lbl'].setFixedHeight(base_alb_dim[1])
                    info['lbl'].move(base_alb_x, base_next_y)
                    info['lbl'].setText(field)
                    info['lbl'].setVisible(True)
                    info['val'] = QLabel(wid)
                    info['val'].setObjectName("val_" + field)
                    info['val'].setFixedWidth(base_vlb_dim[0])
                    info['val'].setFixedHeight(base_vlb_dim[1])
                    info['val'].move(base_vlb_x, base_next_y)
                    info['val'].setText('out')
                    info['val'].setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                    info['val'].setVisible(True)
                    info['sub'] = 0
                    if info['typecode'] == 8:
                        info['sub'] = rospy.Subscriber(rubi_out, RubiFloat, partial(self.val_cb, info['val']))
                    else:
                        info['sub'] = rospy.Subscriber(rubi_out, RubiInt, partial(self.val_cb, info['val']))
                    self.subscribers += [info['sub']]
                # RUBI_READONLY or RUBI_READWRITE
                else:
                    info['lbl'] = QLabel(wid)
                    info['lbl'].setObjectName("lbl_" + field)
                    info['lbl'].setFixedWidth(base_alb_dim[0])
                    info['lbl'].setFixedHeight(base_alb_dim[1])
                    info['lbl'].move(base_alb_x, base_next_y)
                    info['lbl'].setText(field)
                    info['lbl'].setVisible(True)
                    info['spn'] = info['pub'] = 0
                    # float field
                    if info['typecode'] == 8:
                        info['spn'] = QDoubleSpinBox(wid)
                        info['spn'].setRange(-2**self.FLOAT_EXP, 2**self.FLOAT_EXP-1)
                        info['spn'].setDecimals(self.FLOAT_PREC)
                        info['pub'] = rospy.Publisher(rubi_in, RubiFloat, queue_size=self.PUB_QSIZE)
                    # (u)int field
                    else:
                        info['spn'] = QSpinBox(wid)
                        tc = info['typecode']
                        # int field
                        if 2 <= tc <= 4:
                            info['spn'].setRange(-2**self.UINT_EXP[tc], 2**self.UINT_EXP[tc]-1)
                        # uint field
                        else:
                            info['spn'].setRange(0, 2**self.UINT_EXP[tc])
                        info['pub'] = rospy.Publisher(rubi_in, RubiInt, queue_size=self.PUB_QSIZE)
                    info['spn'].setObjectName("spn_" + field)
                    info['spn'].setFixedWidth(base_spn_dim[0])
                    info['spn'].setFixedHeight(base_spn_dim[1])
                    info['spn'].move(base_spn_x, base_next_y + base_geo_spn_y)
                    info['spn'].setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                    info['spn'].setKeyboardTracking(False)
                    info['spn'].setVisible(True)
                    info['spn'].valueChanged.connect(partial(self.spn_changed, info['spn'], info['pub'], info['typecode']))
                    infovc += [info['spn']]
            # string field
            elif 9 <= info['typecode'] <= 10:
                info['lbl'] = QLabel(wid)
                info['lbl'].setObjectName("lbl_" + field)
                info['lbl'].setFixedWidth(base_alb_dim[0])
                info['lbl'].setFixedHeight(base_alb_dim[1])
                info['lbl'].move(base_alb_x, base_next_y)
                info['lbl'].setText(field)
                info['lbl'].setVisible(True)
                info['pub'] = info['sub'] = 0
                info['str'] = QTextEdit(wid)
                # short field
                if info['typecode'] == 9:
                    info['str'].setObjectName("tsh_" + field)
                    info['str'].setFixedWidth(base_tsh_dim[0])
                    info['str'].setFixedHeight(base_tsh_dim[1])
                    info['str'].move(base_tsh_x, base_next_y + base_geo_str_y)
                    info['str'].setText('short string 01234567890')
                # long field
                else:
                    info['str'].setObjectName("tln_" + field)
                    info['str'].setFixedWidth(base_tln_dim[0])
                    info['str'].setFixedHeight(base_tln_dim[1])
                    info['str'].move(base_tln_x, base_next_y + base_geo_str_y)
                    info['str'].setText('long string 123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123')
                    base_next_y += base_geo_tln_next
                info['str'].setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                info['str'].setStyleSheet("font-family:'Sans Serif'; font-size:9pt;")
                info['str'].setVisible(True)
                if not info['in']:
                    info['str'].setReadOnly(True)
                    info['sub'] = rospy.Subscriber(rubi_out, RubiString, partial(self.str_cb, info['str']))
                    self.subscribers += [info['sub']]
                else:
                    info['pub'] = rospy.Publisher(rubi_in, RubiString, queue_size=self.PUB_QSIZE)
                    info['str'].installEventFilter(self.qet_ef)
                base_next_y += base_geo_str_spacing
                base_next_y += base_geo_str_y
            # bool field
            elif info['typecode'] == 11:
                info['chb'] = QCheckBox(wid)
                info['chb'].setObjectName("chb_" + field)
                info['chb'].setFixedWidth(base_alb_dim[0])
                info['chb'].setFixedHeight(base_alb_dim[1])
                info['chb'].move(base_alb_x, base_next_y)
                info['chb'].setText(field)
                info['chb'].setCheckable(True)
                info['chb'].setVisible(True)
                info['pub'] = info['sub'] = 0
                if not info['in']:
                    info['chb'].setEnabled(False)
                    info['sub'] = rospy.Subscriber(rubi_out, RubiBool, partial(self.bool_cb, info['chb']))
                    self.subscribers += [info['sub']]
                else:
                    info['chb'].setEnabled(True)
                    info['pub'] = rospy.Publisher(rubi_in, RubiBool, queue_size=self.PUB_QSIZE)
                    info['chb'].stateChanged.connect(partial(self.chb_changed, info['chb'], info['pub']))
                    infosc += [info['chb']]
            else:
                rospy.logwarn("RC: show_board_window(): board %s, field %s with typecode %s", board, field, info['typecode'])

            base_next_y += base_move_y
            base_next_y += base_geo_spacing

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
            th = self.boards[board]['srv_wake']().wake
        except Exception as e:
            rospy.logwarn("RC: show_board_window(): exception during srv_wake for board %s, e=%s",
                           board, e)
            th = False
        # todo why no thread crash?
        # rospy.logerr("th %s", th)
        nope.setVisible(not th)
        self.boards[board]['nope'] = nope

        wid.setMinimumHeight(base_next_y)

        self.timers += [rospy.Timer(rospy.Duration(2), partial(self.watch_board_cb, board))]

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
        info_panel = self.RubiControllerPanel("INFO")
        can_load = '/rubi/cans_load/'
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

        self.RubiControllerPanel("btn_panic").move(base_can_move, base_can_move * (len(self.cans) + 2))
        self.RubiControllerPanel("btn_panic").clicked.connect(self.panic_clicked)

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
        wid = self.RubiControllerPanel("bScrArWC")
        for board in self.boards:
            if board in self.curr_gui_boards:
                # rospy.loginfo("%s moved to %s", board, base_chb_xy[1])
                self.boards[board]['chb'].move(base_chb_xy[0], base_chb_xy[1])
                self.boards[board]['cb2'].move(base_cb2_xy[0], base_cb2_xy[1])
                self.boards[board]['lbl'].move(base_lbl_xy[0], base_lbl_xy[1])
                self.boards[board]['pb'].move(base_pb_xy[0], base_pb_xy[1])
            else:
                # rospy.loginfo("%s new at %s", board, base_chb_xy[1])
                self.boards[board]['chb'] = QCheckBox(wid)
                self.boards[board]['chb'].setObjectName("chb_" + board)
                self.boards[board]['chb'].setFixedWidth(base_chb_dim[0])
                self.boards[board]['chb'].setFixedHeight(base_chb_dim[1])
                self.boards[board]['chb'].move(base_chb_xy[0], base_chb_xy[1])
                self.boards[board]['chb'].setCheckable(True)
                self.boards[board]['chb'].setChecked(False)
                self.boards[board]['chb'].setEnabled(False)
                self.boards[board]['chb'].setVisible(True)
                self.boards[board]['srv_online'] = rospy.ServiceProxy('/rubi/boards/' + board + "/is_online", BoardOnline)
                self.boards[board]['cb2'] = QCheckBox(wid)
                self.boards[board]['cb2'].setObjectName("cb2_" + board)
                self.boards[board]['cb2'].setFixedWidth(base_cb2_dim[0])
                self.boards[board]['cb2'].setFixedHeight(base_cb2_dim[1])
                self.boards[board]['cb2'].move(base_cb2_xy[0], base_cb2_xy[1])
                self.boards[board]['cb2'].setCheckable(True)
                self.boards[board]['cb2'].setChecked(False)
                self.boards[board]['cb2'].setEnabled(False)
                self.boards[board]['cb2'].setVisible(True)
                self.boards[board]['srv_wake'] = rospy.ServiceProxy('/rubi/boards/' + board + "/is_wake", BoardWake)

                self.boards[board]['lbl'] = QLabel(wid)
                self.boards[board]['lbl'].setObjectName("lbl_" + board)
                self.boards[board]['lbl'].setFixedWidth(base_lbl_dim[0])
                self.boards[board]['lbl'].setFixedHeight(base_lbl_dim[1])
                self.boards[board]['lbl'].move(base_lbl_xy[0], base_lbl_xy[1])
                self.boards[board]['lbl'].setText(board)
                self.boards[board]['lbl'].setVisible(True)
                self.boards[board]['pb'] = QPushButton(wid)
                self.boards[board]['pb'].setObjectName("pb_" + board)
                self.boards[board]['pb'].setFixedWidth(base_pb_dim[0])
                self.boards[board]['pb'].setFixedHeight(base_pb_dim[1])
                self.boards[board]['pb'].move(base_pb_xy[0], base_pb_xy[1])
                self.boards[board]['pb'].setText("CONNECT")
                self.boards[board]['pb'].clicked.connect(partial(self.show_board_window, board))
                self.boards[board]['pb'].setVisible(True)

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
        # wid = self.RubiControllerPanel("pb_FakeBoard1")
        # wid2 = self.RubiControllerPanel("pb_FakeBoard2")
        # rospy.loginfo("pb fbs: %s -- %s", wid, wid2)

    @pyqtSlot()
    # todo boards dissapear
    def check_boards(self):
        # self.RubiControllerPanel("chb_online").setChecked(False)
        try:
            rospy.wait_for_service('/rubi/show_boards', timeout=1)
            show_boards = rospy.ServiceProxy('/rubi/show_boards', ShowBoards)
            boards_names = show_boards().boards_names

            rospy.wait_for_service('/rubi/get_board_instances', timeout=1)
            board_instances = rospy.ServiceProxy('/rubi/get_board_instances', BoardInstances)

            resp = []
            for bn in boards_names:
                for id in board_instances(bn).ids:
                    if id == "":
                        instance_name = bn
                    else:
                        instance_name = bn + "/" + id

                    name = {"instance_name" : instance_name, "descriptor_name" : bn}
                    resp.append(name)

            self.RubiControllerPanel("chb_online").setChecked(True)
            return resp
        except rospy.ROSException:
            self.RubiControllerPanel("chb_online").setChecked(False)
        except rospy.ServiceException as e:
            rospy.logwarn("RC: ros services call failed: %s", e)


    @pyqtSlot()
    def get_board_info(self, board_name):
        rospy.loginfo("RC: trying get info about %s", board_name)
        try:
            info = OrderedDict([(self.CFL, OrderedDict()), (self.CFN, OrderedDict())])
            board_info = rospy.ServiceProxy(
                '/rubi/get_board_descriptor', BoardDescriptor
            )
            bi = board_info(board_name)
            info['description'] = bi.description
            info['version'] = bi.version
            info['driver'] = bi.driver
            for field in bi.fields:
                field_info = rospy.ServiceProxy(
                    '/rubi/get_field_descriptor', FieldDescriptor
                )
                fi = field_info(board_name, field)
                info[self.CFL][field] = {
                    self.CSF: fi.subfields, self.CTC: fi.typecode,
                    self.CIN: fi.input, self.COT: fi.output
                }
            for func in bi.functions:
                func_info = rospy.ServiceProxy(
                    '/rubi/get_func_descriptor', FuncDescriptor
                )
                fi = func_info(board_name, func)
                info[self.CFN][func] = {
                    self.CAN: fi.arg_names,
                    self.CTI: fi.type_in, self.CTO: fi.type_out
                }
            info['valueChanged'] = []
            info['stateChanged'] = []
            return info
        except rospy.ServiceException as e:
            rospy.logwarn("RC: some /rubi/get_descriptor call failed: %s", e)

    @pyqtSlot()
    def get_cans_names(self):
        rospy.loginfo("RC: trying get names of cans")
        try:
            names = rospy.ServiceProxy(
                '/rubi/get_cans_names', CansNames
            )
            return names().names
        except rospy.ServiceException as e:
            rospy.logwarn("RC: some /rubi/get_cans_names call failed: %s", e)

    # def board_announ_cb(self, msg):
    #     rospy.loginfo("RC: new board announced: %s", msg)
    #     self.refresh_boards_signal.emit()

    def check_boards_cb(self, event):
        # rospy.loginfo("RC: periodic boards check")
        for board in self.boards:
            if 'chb' in self.boards[board] and 'cb2' in self.boards[board]:
                try:
                    th = self.boards[board]['srv_online']().online
                    self.boards[board]['chb'].setChecked(th)
                except rospy.ServiceException:
                    self.boards[board]['chb'].setChecked(False)
                try:
                    # todo why no thread crash?
                    th = self.boards[board]['srv_wake']().wake
                    # rospy.logerr("th cb %s", th)
                    self.boards[board]['cb2'].setChecked(th)
                except rospy.ServiceException:
                    self.boards[board]['cb2'].setChecked(False)

        boards_names = self.check_boards()

        if boards_names:
            changed = 0
            for board in boards_names:
                board_name = board["instance_name"]
                if board_name not in self.boards:
                    self.boards[board_name] = self.get_board_info(board["descriptor_name"])
                    changed += 1
            if changed > 0:
                self.refresh_boards_signal.emit()
        else:
            for board in self.boards:
                if 'nope' in self.boards[board]:
                    self.boards[board]['nope'].setVisible(True)

    def watch_board_cb(self, board, event):
        rospy.loginfo("RC: periodic %s watch" % board)
        curr_watch = False
        try:
            curr_watch = self.boards[board]['srv_online']().online
            self.boards[board]['nope'].setVisible(not self.boards[board]['srv_wake']().wake)
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
        self.panic_pub = rospy.Publisher('/rubi/panic', Empty, queue_size=self.PUB_QSIZE)
        self.qet_ef = QTextEditEventFilter()
        self.qsb_ef = QSpinBoxEventFilter()

        self.setObjectName('RubiController')

        self._widget = QWidget()
        ui_file = os.path.join(
            rospkg.RosPack().get_path("aleph2_gui"),
            "resources/ui/rubi.ui"
        )
        loadUi(ui_file, self._widget)
        # self._widget.setObjectName('RubiControllerUi')

        self._widget2 = QWidget()
        ui_file2 = os.path.join(
            rospkg.RosPack().get_path("aleph2_gui"),
            "resources/ui/rubi-board.ui"
        )
        loadUi(ui_file2, self._widget2)
        # self._widget2.setObjectName('BoardControllerUi')
        self.refresh_boards_signal.connect(self.refresh_boards)

        # self.subscribers += [rospy.Subscriber("/rubi/new_boards",
        #                       BoardAnnounce, self.board_announ_cb)]

        self.timers += [rospy.Timer(rospy.Duration(1), self.check_boards_cb)]

        if self.context.serial_number() > 1:
            self._widget.setWindowTitle("{} ({})".format(
                self._widget.windowTitle(), self.context.serial_number()))
            self._widget2.setWindowTitle("{} ({})".format(
                self._widget2.windowTitle(), self.context.serial_number()))
        self.context.add_widget(self._widget)
        # self.context.add_widget(self._widget2)
        # self._widget2.setVisible(False)

    def shutdown_plugin(self):
        for subscriber in self.subscribers:
            subscriber.unregister()
        for timer in self.timers:
            timer.shutdown()
        # print(">> %s\n" % self.boards['FakeBoard2']['fields'])
        # print(">> %s\n" % self.boards['FakeBoard2']['fields']['IntTest']['sub'])

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
