# todos
# AttributeError: 'module' object has no attribute 'warninfo' -> QTextEditEventFilter on close
# and TypeError: invalid result from QTextEditEventFilter.eventFilter(), an integer is required la
# and maybe QObject.eventHandler in else:?
# and maybe cursor at end?
# SPEF: some number restrictions?
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, \
    QTextEdit, QSpinBox, QDoubleSpinBox

from PyQt5.QtCore import *
from PyQt5.QtCore import pyqtSlot, pyqtSignal, QEvent

from rubi_server.srv import ShowBoards, BoardDescriptor, \
    FieldDescriptor, FuncDescriptor


class RubiQTextEdit(QTextEdit):
    def keyPressEvent(self, ev):
        rospy.loginfo("inside kpe")
        text = unicode(ev.text())
        key = ev.key()
        if key == Qt.Key_Enter:
            rospy.loginfo("inside kpe %s", self.toPlainText())
        elif text:
            rospy.loginfo("inside kpe")
            # self.text_typed(text)
        else:
            QTextEdit.keyPressEvent(self, ev)

    def __init__(self, parent=None):
        QTextEdit.__init__(self)


class QTextEditEventFilter(QObject):
    def eventFilter(self, obj, event):
        if not isinstance(obj, QTextEdit):
            # rospy.logwarn("RB: QTextEditEventFilter() installed to %s instead of QTextEdit: something bad will happen soon!", obj)
            return False
        else:
            if event.type() == QEvent.KeyPress and event.key() == Qt.Key_Return:
                max_len = 31
                (type, field) = obj.objectName().split('_')

                if type == 'tln':
                    max_len = 255
                text = obj.toPlainText()
                rospy.loginfo(text)
                if len(text) > max_len:
                    rospy.logwarn("RC: text too long (>%s), thus has been truncated", max_len)
                    text = text[:max_len]
                obj.setPlainText(text)
                obj.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

                pw = obj.parentWidget()
                ch = pw.findChild(QWidget, "pub")
                return True
            # elif event.type() == QEvent.KeyPress:
            #     QTextEdit.keyPressEvent(obj, event)
            #     return True
            else:
                return False


class QSpinBoxEventFilter(QObject):
    def eventFilter(self, obj, event):
        if not (isinstance(obj, QSpinBox) or isinstance(obj, QDoubleSpinBox)):
            # rospy.logwarn("RB: QTextEditEventFilter() installed to %s instead of QTextEdit: something bad will happen soon!", obj)
            return False
        else:
            # rospy.loginfo("**%s %s", event.type(), obj)
            if event.type() == QEvent.KeyPress and event.key() == Qt.Key_Return:
                # rospy.loginfo("**%s %s return", event.type(), obj)
                obj.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                return True
            else:
                return False
            # if event.type() == QEvent.KeyPress and event.key() == Qt.Key_Return:
            #     max_len = 31
            #     (type, field, board) = obj.objectName().split('_')
            #     if type == 'tln':
            #         max_len = 255
            #     text = obj.toPlainText()
            #     rospy.loginfo(text)
            #     if len(text) > max_len:
            #         rospy.logwarn("RC: text too long (>%s), thus has been truncated", max_len)
            #         text = text[:max_len]
            #     obj.setPlainText(text)
            #     obj.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            #     return True
            # elif event.type() == QEvent.KeyPress:
            #     QTextEdit.keyPressEvent(obj, event)
            #     return True
            # else:
            #     return False


class RubiBoard(Plugin):
    def __init__(self, context):
        super(RubiBoard, self).__init__(context)

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass

    # def trigger_configuration(self):
    #     pass
