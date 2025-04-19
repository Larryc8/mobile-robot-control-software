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

        visualization_panel = VisualizationPanel()
            # PatrolsPanel(),
            # SelectModePanel(),
        
        self.layout.addWidget(visualization_panel, 0, 0 )
        # self.layout.addWidget(select_mode_panel, 0, 2)
        # self.layout.addWidget(patrol_panel, 1, 2, 1, -1)
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
        map = MyViz()
        self.layout.addWidget(map, 0, 0)
        self.layout.addLayout(self.buttons_layout, 1, 0)
        self.setLayout(self.layout)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    t = HomePanel()
    # t = PatrolsPanel()
    # t = VisualizationPanel()
    t.show()

    sys.exit(app.exec())

