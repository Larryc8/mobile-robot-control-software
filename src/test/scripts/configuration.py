from typing import List
import numpy as np
import time

import sys
import rospy
from PyQt5.QtWidgets import (
    QMainWindow,
    QApplication,
    QPushButton,
    QWidget,
    QTabWidget,
    QVBoxLayout,
    QLayout,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QSlider,
    QGroupBox,
    QTabWidget,
    QStyle,
    QLineEdit,
)
from PyQt5.QtCore import Qt, pyqtSlot, pyqtSignal
from config_model import ConfigModel, NodesManager, StaticParamsConfigLoader

inputsHasChanged = False

from styles.buttons import (
    border_button_style,
    border_button_style_danger,
    primary_button_style,
    secondary_button_style,
    colored_button_style,
    tertiary_button_style,
    toggle_button_style,
    minimal_button_style,
    patrol_checkbox_style,
)

from styles.labels import inactive_label_style, minimal_label_style

class ConfigPanel(QWidget):
    query_param = pyqtSignal(str)
    def __init__(self, nodes_manager) -> None:
        super().__init__()
        # self.inputsHasChanged = False
        self.config_mapping = ConfigModel(workspace="/turtlebot3_slam_gmapping/")
        self.config_dwa_planner = ConfigModel(
            param_file="./navigation_dwaplanner_params.yml",
            workspace="/move_base/DWAPlannerROS/",
        )
        self.config_acml = ConfigModel(
            param_file="./navigation_acml_params.yml", workspace="/acml/"
        )
        self.configs = {
            "Mapeo": self.config_mapping,
            "Localizacion": self.config_acml,
            "Planeacion": self.config_dwa_planner,
        }

        StaticParamsConfigLoader(
            "./navigation_move_base_static_params.yml", "/move_base/"
        )

        self.nodes_manager = nodes_manager
        self.nodes = [
            {
                "package": "gmapping",
                "exec": "slam_gmapping",
                "name": "turtlebot3_slam_gmapping",
            }
        ]

        self.layout = QVBoxLayout()
        self.buttons_layout = QHBoxLayout()
        self.tabs = QTabWidget()
        self.filter_text = QLineEdit()
        self.filter_text.setPlaceholderText('Busca los parametros por nombre')
        self.filter_text.returnPressed.connect(self.handleLineEditEnter)


        self.panels = [
            {
                "Mapeo": Panel(
                    self.config_mapping.get_params(), self.config_mapping.get_ranges()
                )
            },
            {
                "Localizacion": Panel(
                    self.config_acml.get_params(), self.config_acml.get_ranges()
                )
            },
            {
                "Planeacion": Panel(
                    self.config_dwa_planner.get_params(),
                    self.config_dwa_planner.get_ranges(),
                )
            },
        ]

        for panel in self.panels:
            [(name, tab)] = panel.items()
            self.tabs.addTab(tab, name)
            self.query_param.connect(tab.update_panel)


        self.save_config_button = QPushButton("Guardar Configuracion")
        self.apply_config_button = QPushButton("Aplicar Cambios")
        self.reset_config_btn = QPushButton("Restaurar valores")
        self.buttons_layout.addWidget(self.save_config_button, 2)
        self.buttons_layout.addWidget(self.reset_config_btn, 1 )
        self.buttons_layout.addWidget(self.apply_config_button, 1)


        self.save_config_button.clicked.connect(self.saveClickHandler)
        self.apply_config_button.clicked.connect(self.applyClickHandler)
        self.save_config_button.setStyleSheet(colored_button_style)
        self.apply_config_button.setStyleSheet(primary_button_style)
        self.reset_config_btn.setStyleSheet(secondary_button_style)
        # self.start_nodes_button.clicked.connect(self.startNodesClickHandler)
        # btn.setIcon(QApplication.style().standardIcon()) SP_BrowserReload

        icon = QApplication.style().standardIcon(QStyle.SP_DialogSaveButton)
        self.save_config_button.setIcon(icon)
        icon = QApplication.style().standardIcon(QStyle.SP_DialogApplyButton)
        self.apply_config_button.setIcon(icon)
        icon = QApplication.style().standardIcon(QStyle.SP_BrowserReload)
        self.reset_config_btn.setIcon(icon)

        self.layout.addWidget(self.filter_text)
        self.layout.addWidget(self.tabs)
        # self.layout.addWidget(self.reset_config_btn)
        self.layout.addLayout(self.buttons_layout)

        self.setLayout(self.layout)

    @pyqtSlot()
    def saveClickHandler(self) -> None:
        for panel in self.panels:
            [(name, tab)] = panel.items()
            self.configs.get(name).set_params(tab.getInputsValue())

    @pyqtSlot()
    def applyClickHandler(self) -> None:
        self.nodes_manager.stopNodes(["turtlebot3_slam_gmapping"])
        self.nodes_manager.bringUpStop()
        time.sleep(2)
        print("RE-START NODES")
        self.nodes_manager.bringUpStart()
        self.nodes_manager.startNodes(self.nodes_manager.initNodes(self.nodes))

    @pyqtSlot()
    def startNodesClickHandler(self) -> None:
        self.nodes_manager.bringUpStart()
        self.nodes_manager.startNodes(self.nodes_manager.initNodes(self.nodes))

    @pyqtSlot()
    def handleLineEditEnter(self):
        self.query_param.emit(self.filter_text.text())


class Panel(QWidget):
    def __init__(self, configs: dict, ranges: dict = {}) -> None:
        super().__init__()
        self.MAX_ELEMENT_BY_COLUMN = 6
        self.BLACK_LIST = ["map_frame"]
        self.layout = QGridLayout()
        self.input_values = configs.copy()
        self.filter_text = QLineEdit()
        self.ranges = ranges

        self.generateConfigInputs()

        for index, input in enumerate(self.config_inputs):
            row = index % self.MAX_ELEMENT_BY_COLUMN
            column = index // self.MAX_ELEMENT_BY_COLUMN
            self.layout.addWidget(input, row, column, alignment=Qt.AlignTop)

        self.setLayout(self.layout)

    def generateConfigInputs(self):
        self.config_inputs = []
        for name, value in self.input_values.items():
            if (
                not isinstance(value, str)
                and not isinstance(value, bool)
                and value != 0
            ):
                slider_range = self.ranges.get(name)
                self.config_inputs.append(
                    ConfigInput(
                        name,
                        default_value=value,
                        _range=[
                            slider_range - slider_range,
                            slider_range + slider_range,
                        ],
                    )
                )




    def getInputs(self) -> list:
        return self.config_inputs

    def getInputsValue(self) -> dict:
        for input in self.config_inputs:
            self.input_values.update(input.value())
        return self.input_values

    def update_panel(self, text):
        # if text:
        print('filter from panel', text)
        for index  in range(len(self.config_inputs)):
            row = index % self.MAX_ELEMENT_BY_COLUMN
            column = index // self.MAX_ELEMENT_BY_COLUMN
            item = self.layout.itemAtPosition(row, column)
            if item:
                widget = item.widget()
                if widget:
                    widget.deleteLater()

        self.generateConfigInputs()

        for index, input in enumerate([item for item in self.config_inputs if text in item.title()]):
            row = index % self.MAX_ELEMENT_BY_COLUMN
            column = index // self.MAX_ELEMENT_BY_COLUMN
            self.layout.addWidget(input, row, column, alignment=Qt.AlignTop)



class ConfigInput(QGroupBox):
    def __init__(
        self, label_text: str, default_value: int = 0, _range: List[float] = [1, 2]
    ) -> None:
        super().__init__(label_text)
        self.setFixedHeight(115)
        self.setFixedWidth(300)
        # self.setStyleSheet("background-color: navy;")
        self.range = _range
        self.name = label_text

        step = (max(self.range) - min(self.range)) / 100
        if min(self.range) < 0:
            slider_range = range(100, 0, -1)
            step = -step
        else:
            slider_range = range(0, 100, 1)
        self.param_values_range = [round(value * step, 5) for value in slider_range]

        self.layout = QVBoxLayout()
        self.label = QLabel()
        self.label.setText(self.name)
        self.value_lebel = QLabel()
        self.line_edit = QLineEdit()
        self.current_value = {0: 2.34}

        self.value_lebel.setText(str(default_value))
        self.line_edit.setText(str(default_value))

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setValue(self.mapValueToRange(default_value))
        self.slider.setTickInterval(20)
        self.slider.setTickPosition(QSlider.TicksBothSides)
        # self.slider.setRange(min(range), max(range))

        self.slider.valueChanged.connect(self.changeEventHandler)
        self.line_edit.returnPressed.connect(self.handleLineEditChange)

        [
            self.layout.addWidget(element)
            for element in (  self.line_edit, self.slider)
        ]
        self.setLayout(self.layout)

    @pyqtSlot(int)
    def changeEventHandler(self, slider_value):
        # range_index = [value * step for value in _range]
        global inputsHasChanged
        inputsHasChanged = True
        value = self.mapRangeToValue(slider_value)
        self.value_lebel.setText(str(value))
        self.line_edit.setText(str(value))

    def mapRangeToValue(self, index: int) -> float:
        return self.param_values_range[index]

    def mapValueToRange(self, value) -> int:
        try:
            index = self.param_values_range.index(value)
            print(f"Found at index {index}")
        except ValueError:
            index = -1
            print("Not found")
        return index

    def value(self) -> dict:
        return {self.label.text(): self.mapRangeToValue(self.slider.value())}

    def handleLineEditChange(self):
        try:
            x = self.line_edit.text()
            value =  float(x)
            range_values_float = self.param_values_range.copy()
            range_values_float.pop()
            range_values_float.append(value)
            range_values_float.sort()
            index = range_values_float.index(value)
            # index = self.mapValueToRange(float(x))
            self.slider.setValue(index)
        except ValueError as e:
            print('config Panel-Input', e)
        # print(x, index)
        # if index > 0:


if __name__ == "__main__":
    app = QApplication([])
    ex = ConfigPanel()
    ex.show()
    app.exec()
