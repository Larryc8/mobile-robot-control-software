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
    QGroupBox
)
from PyQt5.QtCore import Qt, pyqtSlot


class ConfigPanel(QWidget):
    def __init__(self) -> None:
        super().__init__()
        config_labels_mapping = [
            "maximo",
            "minimo",
            "cambiar algo",
            "cambiar otra cosa harold andres riascos manyoma",
        ]

        config_labels_navigation = [
            "navegacion maximo navgation",
            "navegacion minimo navigation",
            "navegacion cambiar algo",
            "navegacion cambiar otra cosa harold andres riascos manyoma",
        ]

        self.layout = QVBoxLayout()
        self.main_panel = QHBoxLayout()
        panel = QVBoxLayout()
        panel2 = QVBoxLayout()
        # panel3 = QVBoxLayout()

        self.save_config_button = QPushButton("save me")
        self.save_config_button.clicked.connect(self.clickHandler)
        self.layout.addWidget(self.save_config_button)

        self.config_inputs_mapping = [
            ConfigInput(input_label) for input_label in config_labels_mapping
        ]
        self.config_inputs_navigation = [
            ConfigInput(input_label) for input_label in config_labels_navigation
        ]

        label1 = QLabel()
        label2 = QLabel()
        label1.setText("--MAPEO--")
        label2.setText("--NAVEGACION--")

        panel.addWidget(label1)
        [panel.addWidget(input) for input in self.config_inputs_mapping]
        panel2.addWidget(label2)
        [panel2.addWidget(input) for input in self.config_inputs_navigation]

        self.main_panel.addLayout(panel)
        self.main_panel.addLayout(panel2)
        self.layout.addLayout(self.main_panel)
        self.setLayout(self.layout)

    @pyqtSlot()
    def clickHandler(self) -> None:
        print("start ")
        [print(input.value()) for input in self.config_inputs_mapping]
        print("ddd")


class ConfigInput(QGroupBox):
    def __init__(self, label_text: str) -> None:
        super().__init__()
        self.layout = QVBoxLayout()
        self.label = QLabel()
        self.label.setText(label_text)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.valueChanged.connect(self.eventHandler)

        self.layout.addWidget(self.label)
        self.layout.addWidget(self.slider)
        self.setLayout(self.layout)

    @pyqtSlot(int)
    def eventHandler(self, slider_value):
        # print("valor del slider cambio", slider_value)
        pass

    def value(self) -> int:
        return self.slider.value()


if __name__ == "__main__":
    app = QApplication([])
    # ex = ConfigInput()
    ex = ConfigPanel()
    ex.show()
    app.exec()
