import sys
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
    QCheckBox,
    QComboBox,
    QGroupBox,
    QScrollArea,
)
from PyQt5.QtCore import Qt, pyqtSlot
from rview import MyViz


class HomePanel(QWidget):
    def __init__(self):
        super().__init__()
        # self.resize(900, 900)
        self.layout = QGridLayout()
        # self.showMaximized()
        # self.layout = QHBoxLayout()
        # [self.layout.addWidget(element) for element in (VisualizationPanel(), PatrolsPanel())]

        visualization_panel, patrol_panel, select_mode_panel = (
            VisualizationPanel(),
            PatrolsPanel(),
            SelectModePanel(),
        )
        self.layout.addWidget(visualization_panel, 0, 0, -1, 1)
        self.layout.addWidget(select_mode_panel, 0, 2)
        self.layout.addWidget(patrol_panel, 1, 2, 1, -1)
        self.layout.setColumnStretch(0, 1)

        self.setLayout(self.layout)


class VisualizationPanel(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.layout = QGridLayout()
        self.buttons_layout = QHBoxLayout()

        [
            self.buttons_layout.addWidget(button)
            for button in (
                QPushButton("cargar mapa"),
                QPushButton("borrar punto de initeres"),
                QPushButton("crear punto de intere"),
            )
        ]
        map = MyViz(configfile="./config.myviz")
        self.layout.addWidget(map, 0, 0)
        self.layout.addLayout(self.buttons_layout, 1, 0)
        self.setLayout(self.layout)


class SelectModePanel(QGroupBox):
    def __init__(self) -> None:
        super().__init__()
        self.layout = QGridLayout()
        self.name = QLabel()
        self.name.setText("Modo de operacion")

        auto_mode_button, manual_mode_button = (
            QPushButton("auto"),
            QPushButton("manual"),
        )

        self.layout.addWidget(self.name, 0, 0, 1, -1)
        self.layout.addWidget(auto_mode_button, 1, 0)
        self.layout.addWidget(manual_mode_button, 1, 1)

        self.setLayout(self.layout)


class PatrolsPanel(QGroupBox):
    def __init__(self) -> None:
        super().__init__()
        self.scroll_area = QScrollArea()
        # self.scroll_area.setAlignment(Qt.AlignCenter)
        # self.resize(370, 400)
        self.patrols_panel = QWidget()
        self.patrols_panel.layout = QVBoxLayout()
        self.layout = QVBoxLayout()
        self.name = QLabel()
        self.name.setText("Patrullaje")

        self.layout.addWidget(self.name)
        self.control_panel_buttons = QHBoxLayout()
        [
            self.control_panel_buttons.addWidget(button)
            for button in [QPushButton("delete"), QPushButton("crear")]
        ]

        self.navigation_buttons = QHBoxLayout()
        [
            self.navigation_buttons.addWidget(button)
            for button in (QPushButton("<"), QPushButton(">"))
        ]

        patrols_data = {}
        [patrols_data.update({str(i): "harold"}) for i in range(4)]

        patrols = PatrolsContainer(patrols_data)

        # self.scroll_area.setWidget(patrols)

        self.layout.addLayout(self.control_panel_buttons)
        self.layout.addLayout(self.navigation_buttons)
        # self.layout.addWidget(self.scroll_area)
        self.layout.addWidget(patrols)

        self.setLayout(self.layout)


class PatrolsContainer(QWidget):
    def __init__(self, patrols_data: dict) -> None:
        super().__init__()
        self.layout = QVBoxLayout()
        patrols = [Patrol(id=index) for index, _ in patrols_data.items()]

        [self.layout.addWidget(patrol) for patrol in patrols]
        self.setLayout(self.layout)


class Patrol(QGroupBox):
    def __init__(self, name: str = "harold lo mas lindo", id: str = "999"):
        super().__init__()
        # self.setStyleSheet("background-color: gray")
        self.layout = QGridLayout()
        # self.layout = QVBoxLayout()
        # self.resize(300, 100)
        self.patrol_name = QLabel()
        self.patrol_name.setText(id)

        (delete_botton, exec_botton, time_select, checkbox, repeate_patrol_botton) = (
            QPushButton("update"),
            QPushButton("exec"),
            TimeSelect(),
            QCheckBox(),
            QPushButton("loop"),
        )

        self.layout.addWidget(self.patrol_name, 1, 3)
        # self.layout.addWidget(self.patrol_name)
        self.layout.addWidget(delete_botton, 3, 0)
        self.layout.addWidget(exec_botton, 3, 2)
        # self.layout.addWidget(repeate_patrol_botton, 1, 1)
        self.layout.addWidget(checkbox, 1, 0, alignment=Qt.AlignLeft)
        self.layout.addWidget(time_select, 1, 2, alignment=Qt.AlignRight)
        self.setLayout(self.layout)


class TimeSelect(QWidget):
    def __init__(self) -> None:
        super().__init__()
        maxItems = 3
        self.layout = QHBoxLayout()
        hours_label = QLabel()
        minuts_label = QLabel()

        [
            label.setText(text)
            for text, label in zip(["hh", "mm"], [hours_label, minuts_label])
        ]

        hours = QComboBox()
        hours.addItems([str(hour) for hour in range(24)])
        # hours.hidePopup()
        hours.setMaxVisibleItems(maxItems)
        hours.setStyleSheet("combobox-popup: 0")

        minuts = QComboBox()
        minuts.setMaxVisibleItems(maxItems)
        minuts.setStyleSheet("combobox-popup: 0")
        minuts.addItems([str(minut) for minut in range(60)])

        [
            self.layout.addWidget(element)
            for element in [hours_label, hours, minuts_label, minuts]
        ]

        print(hours.maxVisibleItems())
        self.setLayout(self.layout)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    t = HomePanel()
    # t = PatrolsPanel()
    # t = VisualizationPanel()
    t.show()

    sys.exit(app.exec())
