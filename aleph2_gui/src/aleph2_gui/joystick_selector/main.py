import os
import rospy
import rosgraph

from input_msgs.msg import InputMessage
from input_msgs.msg import DevicesListMessage
from python_qt_binding.QtCore import pyqtSlot, pyqtSignal, QObject
from python_qt_binding.QtGui import QColor
#from python_qt_binding.QtWidgets import QWidget

class JoystickSelector(QObject):
    dupa_signal = pyqtSignal()

    def __init__(self, callback, widget, signal):
        super(JoystickSelector, self).__init__()
        self.client_callback = callback
        self.widget = widget
        self.signal = signal
        
        self.devices_dict = {}
        self.selected_dev = "<None>"
        self.devices = ["<None>"]
        
        self.sub_input = 0
        self.widget.itemClicked.connect(self.device_selected)
        self.devices_sub = rospy.Subscriber("/input/devices_list", DevicesListMessage, self.devices_callback)
        rate = rospy.Rate(30)
        
        self.build_list(self.devices)
        #self.widget.item(0).setBackground(QColor("blue"))
        self.widget.item(0).setSelected(True)

    def devices_callback(self, data):
        name = data.node_name
        devs = data.devices_list
        self.devices_dict[name] = devs
        self.devices = ["<None>"]
        for name, dev_list in self.devices_dict.items():
            self.devices += dev_list

        if self.selected_dev not in self.devices:
            self.sub_input.unregister()
            self.sub_input = 0
            self.selected_dev = "<None>"
            self.signal.emit()


        self.build_list(self.devices)

    def build_list(self, devices):
        self.widget.clear()
        self.widget.addItems(devices)
        self.widget.item(0).setSelected(True)
        #for i in range(self.widget.count()):
        #    self.widget.item(i).setForeground(QColor("#ffffff"))#QColor("white")
            #self.widget.item(i).setSelected(True)          

    @pyqtSlot()
    def dupa(self):
        self.widget.setText("<None>")

    @pyqtSlot()
    def help(self):
        self.widget.setText(self.Switch()) 

    @pyqtSlot()
    def device_selected(self):        
        if self.widget.currentItem().text() == self.selected_dev:
            return
        
        if self.selected_dev != "<None>":
            self.sub_input.unregister()
            self.sub_input = 0

        self.selected_dev = self.widget.currentItem().text()
        self.signal.emit()
        for k in self.devices_dict.keys():
            if self.selected_dev in self.devices_dict[k]:
                self.sub_input = rospy.Subscriber("/" + k + "/" + self.selected_dev, InputMessage, self.client_callback)
                #print("/" + k + "/" + self.selected_dev)
                break

    def shutdown(self):
        if self.sub_input != 0:
            self.sub_input.unregister()
        self.widget.clear()
        #self.devices_sub.unregister()
        
