#!/usr/bin/env python
import sys
from PyQt5.QtWidgets import (
    QMainWindow,
    QApplication,
    QPushButton,
    QWidget,
    QTabWidget,
    QVBoxLayout,
)

from config_model import NodesManager

from rvizgui import MyViz
from navbar import TopBar
from home_view import HomePanel  
from configuration import ConfigPanel
from map_view import MappingPanel


class App(QMainWindow):
    def __init__(self):
        super().__init__()
        title = "The best GUI of the whole world"
        left = 50
        top = 50
        width = 1000
        height = 800
        self.setWindowTitle(title)
        self.setGeometry(left, top, width, height)

        self.table_widget = MyTableWidget(self)
        self.setCentralWidget(self.table_widget)

        # self.show()


class MyTableWidget(QWidget):
    def __init__(self, parent):
        super().__init__(parent)
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.tabs_names = ["Home", "Configuracion", "Crear mapa", "Log system"]

        self.tabs = QTabWidget()
        self.tab_widget = [QWidget() for _ in self.tabs_names]

        [
            self.tabs.addTab(tab, name)
            for name, tab in zip(self.tabs_names, self.tab_widget)
        ]

        topbar = TopBar()

        # Create  tab
        for i, _ in enumerate(self.tab_widget):
            self.tab_widget[i].layout = QVBoxLayout()

        self.tab_widget[0].layout.addWidget(HomePanel())
        self.tab_widget[1].layout.addWidget(ConfigPanel())
        # self.tab_widget[2].layout.addWidget(MappingPanel())

        [tab.setLayout(tab.layout) for tab in self.tab_widget]
        
        self.layout.addWidget(topbar)
        self.layout.addWidget(self.tabs)
        self.setLayout(self.layout)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    ex = App()
    ex.show()
    sys.exit(app.exec())
