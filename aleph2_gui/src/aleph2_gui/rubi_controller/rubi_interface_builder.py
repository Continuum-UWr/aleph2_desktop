import sys
from functools import partial

from python_qt_binding.QtWidgets import (
    QWidget, QCheckBox, QLabel, QPushButton, QSpinBox, QDoubleSpinBox, QTextEdit)

from python_qt_binding.QtCore import *
from python_qt_binding.QtGui import QPixmap


class RubiInterfaceBuilder:

    TYPECODE_void = 1
    TYPECODE_int32_t = 2
    TYPECODE_int16_t = 3
    TYPECODE_int8_t = 4
    TYPECODE_uint32_t = 5
    TYPECODE_uint16_t = 6
    TYPECODE_uint8_t = 7
    TYPECODE_float = 8
    TYPECODE_shortstring = 9
    TYPECODE_longstring = 10
    TYPECODE_bool_t = 11
    TYPECODE_RUBI_ENUM1 = 12

    VERTICAL_STEP = 34
    VERTICAL_HALFSTEP = 15
    VERTICAL_TITLE_DIM = (330, 41)
    TITLE_MARGIN = 5
    TITLE_HEIGHT = 41

    HORIZONTAL_SIZE = 340
    HORIZONTAL_PIVOT = 0.65 * 350
    HORIZONTAL_DOUBLEMARGINS = 15

    GEOMETRY = {
        # (typecode, read) : (columns, height, horizontal_offset, vertical_boost)
        (TYPECODE_int32_t, False): (0, 41, 8, 0),
        (TYPECODE_int32_t, True): (0, 24, 8, 8)
    }

    def __init__(self):

        self.WIDGET_BUILDERS = {
            self.TYPECODE_int32_t: self.build_int_widget
        }

        self.vertial_cursor = 5
        self.name_counter = 0

    def build_int_widget(self, container, typecode, read, write):
        if read:
            ret = QSpinBox(container)
            ret.setObjectName(self.make_name())
            ret.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            ret.setKeyboardTracking(False)
            ret.setVisible(True)

            def connect_read_handler(handler):
                return ret.valueChanged.connect(handler)

            def get_value_handler():
                return ret.value()

            if write:
                return (ret, (connect_read_handler, get_value_handler), self.write_handler_int_spinbox)
            else:
                return (ret, (connect_read_handler, get_value_handler), None)
        else:
            ret = QLabel(container)
            ret.setObjectName(self.make_name())
            ret.setText('out')
            ret.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            ret.setVisible(True)
            return (ret, None, self.write_handler_int_label)

    def make_name(self):
        self.name_counter += 1
        return "windget" + str(self.name_counter)

    def write_handler_bool(widgets, datas):
        for widget, data in zip(widgets, datas.data):
            widget.setChecked(data)

    def write_handler_string(widgets, datas):
        for widget, data in zip(widgets, datas.data):
            widget.setText(data)

    def write_handler_int_spinbox(widgets, datas):
        for widget, data in zip(widgets, datas.data):
            widget.setValue(data)

    def write_handler_int_label(self, widgets, datas):
        for widget, data in zip(widgets, datas):
            widget.setText(str(data))

    def write_handler_float_label(widgets, datas):
        for widget, data in zip(widgets, datas.data):
            widget.setText(str(data))

    def build_field(self, container, name, typecode, read_handler, write_handler, subfields):
        if len(subfields) > 0:
            return self.build_field_with_subfields(container, name, typecode, read_handler, write_handler, subfields)
        else:
            return self.build_field_no_subfields(container, name, typecode, read_handler, write_handler)

    def make_label(self, container, x, dim, text):
        field_label = QLabel(container)
        field_label.setObjectName(self.make_name())
        field_label.setFixedWidth(dim)
        field_label.setFixedHeight(41)
        field_label.move(x, self.vertical_cursor)
        field_label.setText(text)
        field_label.setVisible(True)

    def build_field_with_subfields(self, container, name, typecode, toplevel_read_handler, write, subfields):
        self.make_label(container, self.TITLE_MARGIN,
                        self.horizontal_size, name)
        self.vertical_cursor += self.VERTICAL_STEP

        widgets = []
        write_handler = None
        read = toplevel_read_handler is not None

        read_stuffs = []

        for subfield in subfields:
            (widget, read_stuff, write_handler) = \
                self.WIDGET_BUILDERS[typecode](
                    container, typecode, read, write)

            (columns, height, margin,
             step_boost) = self.GEOMETRY[(typecode, read)]

            self.vertical_cursor += step_boost

            self.make_label(container, self.TITLE_MARGIN,
                            self.horizontal_pivot, subfield)

            if columns == 0:
                widget.setFixedWidth(
                    self.horizontal_size - self.horizontal_pivot)
                widget.setFixedHeight(height)
                widget.move(self.horizontal_pivot + margin,
                            self.vertical_cursor + step_boost)

                self.vertical_cursor += self.VERTICAL_STEP
            else:
                assert(0)

            widgets += [widget]
            read_stuffs += [read_stuff]

        if read:
            def handler(): return toplevel_read_handler(
                [get_value_handler() for (_, get_value_handler) in read_stuffs])
            for connect_read_handler, _ in read_stuffs:
                connect_read_handler(handler)

        self.vertical_cursor += self.VERTICAL_HALFSTEP

        if write:
            return partial(write_handler, widgets)
        else:
            print("ZWRACAM KURWE", write)
            return "kurwa"

    def build_field_no_subfields(self, container, name, typecode, read_handler, write_handler):
        assert(False)
