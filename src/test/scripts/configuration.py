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
    QStackedLayout,
    QFileDialog,
)
from PyQt5.QtCore import Qt, pyqtSlot, pyqtSignal
from config_model import ConfigModel, NodesManager, StaticParamsConfigLoader


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
    simple_slider_rightleft_style,
    simple_slider_leftright_style,
    modern_line_edit_style,
    simple_line_edit_style,
    error_simple_line_edit_style,
)

from styles.labels import inactive_label_style, minimal_label_style


class ConfigPanel(QWidget):
    query_param = pyqtSignal(str)

    def __init__(self, nodes_manager) -> None:
        super().__init__()
        self.stacklayout = QStackedLayout()
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
        self.filter_text.setPlaceholderText("Busca los parametros por nombre")
        self.filter_text.returnPressed.connect(self.handleLineEditEnter)
        self.filter_text.setStyleSheet(simple_line_edit_style)

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

        self.tabs.setStyleSheet("""
            
            QTabBar::tab {
                background: #F5F5F5;
                padding: 8px;
                color: #555;
            }
            
            QTabBar::tab:selected {
                color: #333;
                border-bottom: 4px solid #2196F3;
            }
        """)
        # self.tabs.setTabPosition(QTabWidget.West)  # Set tabs to left side
        # self.tabs.setTabPosition(QTabWidget.West)

        self.save_config_button = QPushButton("Guardar Configuracion")
        self.apply_config_button = QPushButton("Aplicar Cambios")
        self.reset_config_btn = QPushButton("Restaurar valores")
        self.buttons_layout.addWidget(self.save_config_button, 2)
        # self.buttons_layout.addWidget(self.reset_config_btn, 1 )
        self.buttons_layout.addWidget(self.apply_config_button, 1)

        self.apply_config_button.hide()

        self.save_config_button.clicked.connect(self.saveClickHandler)
        self.apply_config_button.clicked.connect(self.applyClickHandler)
        self.save_config_button.setStyleSheet(colored_button_style)
        self.apply_config_button.setStyleSheet(primary_button_style)
        # self.reset_config_btn.setStyleSheet(secondary_button_style)
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
        # self.stacklayout.addLayout(self.layout)
        self.advance_config_wrapper = QWidget()
        self.basic_config_wrapper = FriendlyConfig()
        self.advance_config_wrapper.setLayout(self.layout)
        self.stacklayout.addWidget(self.advance_config_wrapper)
        self.stacklayout.addWidget(self.basic_config_wrapper)

        self.upper_layout = QVBoxLayout()
        # self.upper_layout.addWidget(QPushButton('Advence'))
        self.upper_layout.addLayout(self.stacklayout)

        # self.setLayout(self.layout)
        self.setLayout(self.upper_layout)

    @pyqtSlot()
    def saveClickHandler(self) -> None:
        self.apply_config_button.hide()
        for panel in self.panels:
            [(name, tab)] = panel.items()
            self.configs.get(name).set_params(tab.getInputsValue())
        self.apply_config_button.show()

    @pyqtSlot()
    def applyClickHandler(self) -> None:
        # self.nodes_manager.stopNodes(["turtlebot3_slam_gmapping"])
        # self.nodes_manager.bringUpStop()
        self.nodes_manager.restartNodes()
        time.sleep(2)
        print("RE-START NODES")
        # self.nodes_manager.bringUpStart()
        # self.nodes_manager.startNodes(self.nodes_manager.initNodes(self.nodes))

    @pyqtSlot()
    def startNodesClickHandler(self) -> None:
        self.nodes_manager.bringUpStart()
        self.nodes_manager.startNodes(self.nodes_manager.initNodes(self.nodes))

    @pyqtSlot()
    def handleLineEditEnter(self):
        text = self.filter_text.text()
        self.query_param.emit(text)

    @pyqtSlot(int)
    def update_query_state(self):
        text = ""
        if text:
            self.filter_text.setStyleSheet(simple_line_edit_style)
        else:
            self.filter_text.setStyleSheet(error_simple_line_edit_style)


class Panel(QWidget):
    query_result = pyqtSignal(int)

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
        print("filter from panel", text)
        for index in range(len(self.config_inputs)):
            row = index % self.MAX_ELEMENT_BY_COLUMN
            column = index // self.MAX_ELEMENT_BY_COLUMN
            item = self.layout.itemAtPosition(row, column)
            if item:
                widget = item.widget()
                if widget:
                    widget.deleteLater()

        self.generateConfigInputs()

        widget_array = [item for item in self.config_inputs if text in item.title()]

        for index, input in enumerate(widget_array):
            row = index % self.MAX_ELEMENT_BY_COLUMN
            column = index // self.MAX_ELEMENT_BY_COLUMN
            self.layout.addWidget(
                input, row, column, alignment=(Qt.AlignTop | Qt.AlignLeft)
            )
        self.query_result.emit(len(widget_array))


class ConfigInput(QGroupBox):
    def __init__(
        self, label_text: str, default_value: int = 0, _range: List[float] = [1, 2]
    ) -> None:
        super().__init__(label_text)
        self.setFixedHeight(105)
        self.setFixedWidth(300)
        # self.setStyleSheet("background-color: navy;")
        self.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 14px;
            }


            """)
        self.range = _range
        self.name = label_text

        step = (max(self.range) - min(self.range)) / 100
        if min(self.range) < 0:
            slider_range = range(100, 0, -1)
            step = -step
            style = simple_slider_rightleft_style
        else:
            slider_range = range(0, 100, 1)
            style = simple_slider_leftright_style
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
        self.slider.setStyleSheet(style)
        self.line_edit.setStyleSheet(modern_line_edit_style)

        self.slider.valueChanged.connect(self.changeEventHandler)
        self.line_edit.returnPressed.connect(self.handleLineEditChange)

        [self.layout.addWidget(element) for element in (self.line_edit, self.slider)]
        self.setLayout(self.layout)

    @pyqtSlot(int)
    def changeEventHandler(self, slider_value):
        # range_index = [value * step for value in _range]
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
            value = float(x)
            range_values_float = self.param_values_range.copy()
            range_values_float.pop()
            range_values_float.append(value)
            range_values_float.sort()
            index = range_values_float.index(value)
            # index = self.mapValueToRange(float(x))
            self.slider.setValue(index)
        except ValueError as e:
            print("config Panel-Input", e)
        # print(x, index)
        # if index > 0:


class FriendlyConfig(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.layout = QGridLayout()
        self.layout.addWidget(DescriptionConfigContainer(title="Crecionde mapa"), 1, 1)
        self.layout.addWidget(
            DescriptionConfigContainer(title="Desepeno de localizacion"), 2, 1
        )
        self.layout.addWidget(
            DescriptionConfigContainer(title="Desepeno de Navegacion"), 3, 1
        )
        self.layout.addWidget(
           DataSetConfigContainer(), 4, 1
        )
        self.setLayout(self.layout)


class DescriptionConfigContainer(QGroupBox):
    def __init__(self, title="Estatdiaticas") -> None:
        super().__init__(title)
        self.layout = QVBoxLayout()
        self.option_layout = QHBoxLayout()
        self.layout.addWidget(QLabel("Mucho texto aburriod"))
        self.firtoption_button = QPushButton("pequeno")
        self.secondoption_button = QPushButton("Mediano")
        self.thirdoption_button = QPushButton("Grande")

        self.option_layout.addWidget(self.firtoption_button)
        self.option_layout.addWidget(self.secondoption_button)
        self.option_layout.addWidget(self.thirdoption_button)
        self.layout.addLayout(self.option_layout)
        self.setLayout(self.layout)


class DataSetConfigContainer(QGroupBox):
    def __init__(self, parent = None) -> None:
        super().__init__()
        self.main_layout = QVBoxLayout()
        self.load_dataset_btn = QPushButton('Cargar Dataset')
        self.load_dataset_btn.clicked.connect(self.open_file_dialog)
        self.parent = parent

        self.main_layout.addWidget(self.load_dataset_btn)
        self.setLayout(self.main_layout)


    def open_file_dialog(self):
        options = QFileDialog.Options()
        options |=  QFileDialog.ShowDirsOnly
        file_name, _ = QFileDialog.getOpenFileName(
            self,
            "Select a File",
            "",
            "All Files (*);;Text Files (*.txt);;Python Files (*.py)",
            options=options,
        )


if __name__ == "__main__":
    app = QApplication([])
    ex = ConfigPanel()
    ex.show()
    app.exec()
