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

    FLOAT_EXP = 15
    FLOAT_PREC = 5
    FLOAT_STEP = 0.01

    VERTICAL_STEP = 34
    VERTICAL_HALFSTEP = 15
    VERTICAL_TITLE_DIM = (330, 41)
    TITLE_MARGIN = 5
    TITLE_HEIGHT = 41

    HORIZONTAL_SIZE = 340
    HORIZONTAL_PIVOT = int(0.65 * HORIZONTAL_SIZE)
    HORIZONTAL_DOUBLEMARGINS = 15

    INT_RANGES = {
        #int type: (min, max)
        TYPECODE_int32_t: (-2147483648, 2147483647), 
        TYPECODE_int16_t: (-32768, 32767),
        TYPECODE_int8_t: (-128, 127),
        TYPECODE_uint32_t: (0, 4294967295), 
        TYPECODE_uint16_t: (0, 65535),
        TYPECODE_uint8_t: (0, 255)
    }

    GEOMETRY = {
        # (typecode, read) : (columns, height, horizontal_offset, vertical_boost)
        (TYPECODE_int32_t, False): (0, 41, 8, 0),
        (TYPECODE_int32_t, True): (0, 24, 8, 8),
        (TYPECODE_int16_t, False): (0, 41, 8, 0),
        (TYPECODE_int16_t, True): (0, 24, 8, 8),
        (TYPECODE_int8_t, False): (0, 41, 8, 0),
        (TYPECODE_int8_t, True): (0, 24, 8, 8),
        (TYPECODE_uint32_t, False): (0, 41, 8, 0),
        (TYPECODE_uint32_t, True): (0, 24, 8, 8),
        (TYPECODE_uint16_t, False): (0, 41, 8, 0),
        (TYPECODE_uint16_t, True): (0, 24, 8, 8),
        (TYPECODE_uint8_t, False): (0, 41, 8, 0),
        (TYPECODE_uint8_t, True): (0, 24, 8, 8),
        (TYPECODE_bool_t, False): (0, 36, 8, 0),
        (TYPECODE_bool_t, True): (0, 36, 8, 0),
        (TYPECODE_float, False): (0, 41, 8, 0),
        (TYPECODE_float, True): (0, 24, 8, 8)
    }

    def __init__(self):

        self.WIDGET_BUILDERS = {
            self.TYPECODE_int32_t: self.build_int_widget,
            self.TYPECODE_int16_t: self.build_int_widget,
            self.TYPECODE_int8_t: self.build_int_widget,
            self.TYPECODE_uint32_t: self.build_int_widget,
            self.TYPECODE_uint16_t: self.build_int_widget,
            self.TYPECODE_uint8_t: self.build_int_widget,
            self.TYPECODE_bool_t: self.build_bool_widget,
            self.TYPECODE_float: self.build_float_widget
        }

        self.vertical_cursor = 5
        self.name_counter = 0

    def build_int_widget(self, container, typecode, read, write):
        if read:
            ret = QSpinBox(container)
            ret.setObjectName(self.make_name())
            ret.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            ret.setKeyboardTracking(False)
            ret.setRange(self.INT_RANGES[typecode][0], self.INT_RANGES[typecode][1])
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

    def build_bool_widget(self, container, typecode, read, write):
        ret = QCheckBox(container)
        ret.setObjectName(self.make_name())
        
        if read:
            ret.setCheckable(True)
            ret.setEnabled(True)  
            ret.setVisible(True)

            def connect_read_handler(handler):
                return ret.stateChanged.connect(handler)

            def get_value_handler():
                return ret.checkState();

            if write:
                return (ret, (connect_read_handler, get_value_handler), self.write_handler_bool)
            else:
                return (ret, (connect_read_handler, get_value_handler), None)
        else:
            ret.setChecked(False)
            ret.setEnabled(False)
            ret.setVisible(True)
            return (ret, None, self.write_handler_bool)

    def build_float_widget(self, container, typecode, read, write):
        if read:
            ret = QDoubleSpinBox(container)
            ret.setObjectName(self.make_name())
            ret.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            ret.setKeyboardTracking(False)
            ret.setRange(-2**self.FLOAT_EXP, 2**self.FLOAT_EXP-1)
            ret.setDecimals(self.FLOAT_PREC)
            ret.setSingleStep(self.FLOAT_STEP)
            

            def connect_read_handler(handler):
                return ret.valueChanged.connect(handler)

            def get_value_handler():
                return ret.value()

            if write:
                return (ret, (connect_read_handler, get_value_handler), self.write_handler_float_spinbox)
            else:
                return (ret, (connect_read_handler, get_value_handler), None)
        else:
            ret = QLabel(container)
            ret.setObjectName(self.make_name())
            ret.setText('out')
            ret.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            ret.setVisible(True)
            return (ret, None, self.write_handler_float_label)

    def make_name(self):
        self.name_counter += 1
        return "windget" + str(self.name_counter)

    def write_handler_bool(self, widgets, datas):
        for widget, data in zip(widgets, datas):
            widget.setChecked(data)

    def write_handler_string(self, widgets, datas):
        for widget, data in zip(widgets, datas):
            widget.setText(data)

    def write_handler_int_spinbox(self, widgets, datas):
        for widget, data in zip(widgets, datas):
            widget.setValue(data)

    def write_handler_int_label(self, widgets, datas):
        for widget, data in zip(widgets, datas):
            widget.setText(str(data))

    def write_handler_float_label(self, widgets, datas):
        for widget, data in zip(widgets, datas):
            rndData = round(data, self.FLOAT_PREC+1)
            widget.setText("{value:.{prec}f}".format(value=rndData,prec=self.FLOAT_PREC))
    
    def write_handler_float_spinbox(self, widgets, datas):
        for widget, data in zip(widgets, datas):
            widget.setValue(data)

    def build_field(self, container, name, typecode, read_handler, write, subfields):
        if len(subfields) > 0:
            return self.build_field_with_subfields(container, name, typecode, read_handler, write, subfields)
        else:
            return self.build_field_no_subfields(container, name, typecode, read_handler, write)

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
                        self.HORIZONTAL_SIZE, name)
        self.vertical_cursor += self.VERTICAL_STEP

        widgets = []
        write_handler = None
        read = toplevel_read_handler is not None

        read_stuffs = []

        for subfield in subfields:
            widget, read_stuff, write_handler = self.WIDGET_BUILDERS[typecode](
                container, typecode, read, write)

            columns, height, margin, step_boost = self.GEOMETRY[(
                typecode, read)]

            self.vertical_cursor += step_boost

            self.make_label(container, self.TITLE_MARGIN,
                            self.HORIZONTAL_PIVOT, subfield)

            if columns == 0:
                widget.setFixedWidth(
                    self.HORIZONTAL_SIZE - self.HORIZONTAL_PIVOT)
                widget.setFixedHeight(height)
                widget.move(self.HORIZONTAL_PIVOT + margin,
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

        return None

    def build_field_no_subfields(self, container, name, typecode, read_handler, write):
        write_handler = None
        read = read_handler is not None

        widget, read_stuff, write_handler = self.WIDGET_BUILDERS[typecode](
            container, typecode, read, write)

        columns, height, margin, step_boost = self.GEOMETRY[(typecode, read)]

        self.vertical_cursor += step_boost

        self.make_label(container, self.TITLE_MARGIN,
                        self.HORIZONTAL_PIVOT, name)

        if columns == 0:
            widget.setFixedWidth(
                self.HORIZONTAL_SIZE - self.HORIZONTAL_PIVOT)
            widget.setFixedHeight(height)
            widget.move(self.HORIZONTAL_PIVOT + margin,
                        self.vertical_cursor + step_boost)

            self.vertical_cursor += self.VERTICAL_STEP
        else:
            assert(0)

        if read:
            def handler(): return read_handler(
                [get_value_handler() for (_, get_value_handler) in [read_stuff]])
            for connect_read_handler, _ in [read_stuff]:
                connect_read_handler(handler)

        self.vertical_cursor += self.VERTICAL_HALFSTEP

        if write:
            return partial(write_handler, [widget])

        return None
