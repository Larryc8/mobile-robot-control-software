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
from config_model import ConfigModel


class ConfigPanel(QWidget):
    def __init__(self) -> None:
        super().__init__()
        test_conf = [10 * str(i) for i in range(5)]

        config = ConfigModel()

        self.layout = QVBoxLayout()
        self.tabs = QTabWidget()
        self.mapping_tab = Panel(config.get_params())
        self.nav_tab = Panel(config.get_params())
        self.tabs.addTab(self.mapping_tab, "mapping")
        self.tabs.addTab(self.nav_tab, "navigation")

        self.save_config_button = QPushButton("save me")
        self.save_config_button.clicked.connect(self.clickHandler)
        self.layout.addWidget(self.save_config_button)
        self.layout.addWidget(self.tabs)

        self.setLayout(self.layout)

    @pyqtSlot()
    def clickHandler(self) -> None:
        print("start ")
        [print(input.value()) for input in self.mapping_tab.getInputs()]
        print("ddd")
        print('navigation')
        a = [input.value()  for input in self.nav_tab.getInputs()]


class Panel(QWidget):
    def __init__(self, configs: dict) -> None:
        super().__init__()
        self.MAX_ELEMENT_BY_COLUMN = 6
        self.BLACK_LIST = ["map_frame"]
        self.layout = QGridLayout()

        self.config_inputs = []
        for name, value in configs.items():
            self.config_inputs.append(ConfigInput(name, default_value=50.3))

        for index, input in enumerate(self.config_inputs):
            row = index % self.MAX_ELEMENT_BY_COLUMN
            column = index // self.MAX_ELEMENT_BY_COLUMN
            self.layout.addWidget(input, row, column)

        self.setLayout(self.layout)

    def getInputs(self) -> list:
        return self.config_inputs

    def update(self):
        pass


class ConfigInput(QGroupBox):
    def __init__(self, label_text: str, default_value: float = 23.4) -> None:
        super().__init__()
        self.setFixedHeight(110)

        self.layout = QVBoxLayout()
        self.label = QLabel()
        self.label.setText(label_text)
        self.value_lebel = QLabel()
        self.value_lebel.setText(str(default_value))
        self.slider = QSlider(Qt.Horizontal)
        self.slider.valueChanged.connect(self.changeEventHandler)

        [self.layout.addWidget(element) for element in (self.label, self.value_lebel, self.slider)]
        self.setLayout(self.layout)

    @pyqtSlot(int)
    def changeEventHandler(self, slider_value):
        self.value_lebel.setText(str(slider_value))

    def value(self) -> int:
        return self.slider.value()


if __name__ == "__main__":
    app = QApplication([])
    ex = ConfigPanel()
    ex.show()
    app.exec()
