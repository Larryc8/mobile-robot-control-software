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
)
from PyQt5.QtCore import Qt, pyqtSlot
from config_model import ConfigModel, NodesManager, StaticParamsConfigLoader

inputsHasChanged = False

class ConfigPanel(QWidget):
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
            "localizacion": self.config_acml,
            "planeacion": self.config_dwa_planner,
        }

        StaticParamsConfigLoader("./navigation_move_base_static_params.yml", "/move_base/")

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

        self.save_config_button = QPushButton("save me")
        self.apply_config_button = QPushButton("apply me")
        self.start_nodes_button = QPushButton("rese to default")
        self.buttons_layout.addWidget(self.save_config_button)
        self.buttons_layout.addWidget(self.apply_config_button)
        # self.layout.addWidget(self.set_config_button)

        self.save_config_button.clicked.connect(self.saveClickHandler)
        self.apply_config_button.clicked.connect(self.applyClickHandler)
        # self.start_nodes_button.clicked.connect(self.startNodesClickHandler)

        self.layout.addWidget(self.tabs)
        self.layout.addWidget(self.start_nodes_button)
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


class Panel(QWidget):
    def __init__(self, configs: dict, ranges: dict = {}) -> None:
        super().__init__()
        self.MAX_ELEMENT_BY_COLUMN = 6
        self.BLACK_LIST = ["map_frame"]
        self.layout = QGridLayout()
        self.input_values = configs.copy()

        self.config_inputs = []
        for name, value in configs.items():
            if (
                not isinstance(value, str)
                and not isinstance(value, bool)
                and value != 0
            ):
                slider_range = ranges.get(name)
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

        for index, input in enumerate(self.config_inputs):
            row = index % self.MAX_ELEMENT_BY_COLUMN
            column = index // self.MAX_ELEMENT_BY_COLUMN
            self.layout.addWidget(input, row, column)

        self.setLayout(self.layout)

    def getInputs(self) -> list:
        return self.config_inputs

    def getInputsValue(self) -> dict:
        for input in self.config_inputs:
            self.input_values.update(input.value())
        return self.input_values

    def update(self):
        pass


class ConfigInput(QGroupBox):
    def __init__(
        self, label_text: str, default_value: int = 0, _range: List[float] = [1, 2]
    ) -> None:
        super().__init__()
        self.setFixedHeight(110)
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
        self.value_lebel.setText(str(default_value))

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setValue(self.mapValueToRange(default_value))
        self.slider.setTickInterval(20)
        self.slider.setTickPosition(QSlider.TicksBothSides)
        # self.slider.setRange(min(range), max(range))

        self.slider.valueChanged.connect(self.changeEventHandler)

        [
            self.layout.addWidget(element)
            for element in (self.label, self.value_lebel, self.slider)
        ]
        self.setLayout(self.layout)

    @pyqtSlot(int)
    def changeEventHandler(self, slider_value):
        # range_index = [value * step for value in _range]
        global inputsHasChanged
        inputsHasChanged = True
        value = self.mapRangeToValue(slider_value)
        self.value_lebel.setText(str(value))

    def mapRangeToValue(self, index: int) -> float:
        return self.param_values_range[index]

    def mapValueToRange(self, value) -> int:
        try:
            index = self.param_values_range.index(value)
            print(f"Found at index {index}")
        except ValueError:
            index = 0
            print("Not found")
        return index

    def value(self) -> dict:
        return {self.label.text(): self.mapRangeToValue(self.slider.value())}


if __name__ == "__main__":
    app = QApplication([])
    ex = ConfigPanel()
    ex.show()
    app.exec()
