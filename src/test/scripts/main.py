#!/usr/bin/env python
import sys
from PyQt5.QtWidgets import (
    QMainWindow,
    QApplication,
    QPushButton,
    QWidget,
    QTabWidget,
    QVBoxLayout,
    QLabel
)

from config_model import NodesManager

# from rvizgui import MyViz
from navbar import TopBar
from home_view import HomePanel  
from configuration import ConfigPanel
# from map_view import MappingPanel
from better_image_display import ImageViewer
from logsystem_view import LogPanel

class App(QMainWindow):
    def __init__(self):
        super().__init__()
        title = "The best GUI of the whole world"
        left = 50
        top = 50
        width = 1500
        height = 800
        self.setWindowTitle(title)
        self.setGeometry(left, top, width, height)

        self.table_widget = MyTableWidget(self)
        self.setCentralWidget(self.table_widget)


class MyTableWidget(QWidget):
    def __init__(self, parent):
        super().__init__(parent)
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.nodes_manager = NodesManager()
        self.tabs_names = ["Home", "Configuracion", "Log system"]
        # self.tabs_names = ["Home", "Configuracion"]
        self.parent = parent
        self.parent.w = None

        self.tabs = QTabWidget()
        self.tabs_widget = {name: QWidget() for name in self.tabs_names}

        [
            self.tabs.addTab(tab, name)
            for name, tab in  self.tabs_widget.items()
        ]

        self.tabs.setStyleSheet("""
            QTabBar::tab {
                background: transparent;
                color: #555;
                padding: 10px 8px;
                margin: 4px 2px;
                border-radius: 2px;
            }
            
            QTabBar::tab:selected {
                background: lightblue;
                color: #333;
            }
            QTabBar::tab:hover {
                background: lightgray;
                color: #333;
            }
        """)

        topbar = TopBar()

        # Create  tab
        for name in self.tabs_widget.keys():
            self.tabs_widget[name].layout = QVBoxLayout()

        print('main', parent)
        self.tabs_widget['Home'].layout.addWidget(HomePanel(nodes_manager=self.nodes_manager, parent=parent))
        self.tabs_widget['Configuracion'].layout.addWidget(ConfigPanel(nodes_manager=self.nodes_manager))
        self.tabs_widget['Log system'].layout.addWidget(LogPanel(node_manager=self.nodes_manager))

        [tab.setLayout(tab.layout) for tab in self.tabs_widget.values()]
        self.button = QPushButton("Push for Window")
        self.button.clicked.connect(self.show_new_window)
        
        self.layout.addWidget(topbar)
        # self.layout.addWidget(self.button)
        self.layout.addWidget(self.tabs)
        self.setLayout(self.layout)

    def show_new_window(self, checked):
        # self.parent.w = None
        if self.parent.w is None:
            self.parent.w = ImageViewer()
            print('None ;; multiwindows')
        self.parent.w.show()



if __name__ == "__main__":
    app = QApplication(sys.argv)
    ex = App()
    ex.show()
    sys.exit(app.exec())
