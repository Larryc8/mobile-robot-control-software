import sys
from PyQt5.QtWidgets import (
    QMainWindow,
    QApplication,
    QPushButton,
    QWidget,
    QTabWidget,
    QVBoxLayout,
)

# from PySide6.QtGui import QIcon
# from PyQt5.QtCore import pyqtSlot
from rvizgui import MyViz
from navbar import TopBar


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

        # Initialize tab screen
        self.tabs = QTabWidget()
        self.tab_widget = [QWidget() for _ in self.tabs_names]
        [
            self.tabs.addTab(tab, name)
            for name, tab in zip(self.tabs_names, self.tab_widget)
        ]

        # myviz = MyViz()
        topbar = TopBar()

        # Create first tab
        self.tab_widget[0].layout = QVBoxLayout(self)
        self.pushButton1 = QPushButton("PyQt5 button")
        self.tab_widget[0].layout.addWidget(self.pushButton1)
        self.tab_widget[0].setLayout(self.tab_widget[0].layout)
        #
        # # Add tabs to widget
        self.layout.addWidget(topbar)
        self.layout.addWidget(self.tabs)
        self.setLayout(self.layout)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    ex = App()
    ex.show()
    sys.exit(app.exec())
