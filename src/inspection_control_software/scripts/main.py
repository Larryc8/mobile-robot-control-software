#!/usr/bin/env python
import sys
import rospy

from PyQt5.QtWidgets import (
    QMainWindow,
    QApplication,
    QWidget,
    QTabWidget,
    QVBoxLayout,
    QMessageBox
)

from config_model import NodesManager
from navbar import TopBar
from home_view import HomePanel
from configuration import ConfigPanel
from logsystem_view import LogPanel

class App(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Panel de Control del Robot")
        self.setGeometry(50, 50, 1500, 800)

        self.table_widget = MyTableWidget(self)
        self.setCentralWidget(self.table_widget)

    def closeEvent(self, event):
        reply = QMessageBox.warning(
            self,
            "Confirmar salida",
            "¿Está seguro de que desea cerrar la aplicación?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            event.accept()
        else:
            event.ignore()


class MyTableWidget(QWidget):
    def __init__(self, parent):
        super().__init__(parent)
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.nodes_manager = NodesManager()
        self.tabs_names = ["Home", "Configuracion", "Sistema de alertas"]

        self.parent = parent

        self.tabs = QTabWidget()
        self.tabs_widget = {name: QWidget() for name in self.tabs_names}

        [self.tabs.addTab(tab, name) for name, tab in self.tabs_widget.items()]

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

        for name in self.tabs_widget.keys():
            self.tabs_widget[name].layout = QVBoxLayout()

        home_panel = HomePanel(nodes_manager=self.nodes_manager, parent=parent)
        patrols_scheduler = home_panel.patrol_panel.patrols_container.patrols_scheduler
        config_panel = ConfigPanel(nodes_manager=self.nodes_manager, parent=parent, patrols_scheduler=patrols_scheduler)

        config_panel.basic_config_wrapper.calibration_started.connect(home_panel.patrol_panel.enable_components)
        config_panel.basic_config_wrapper.calibration_started.connect(home_panel.visualization_panel.enable_components)

        #
        # home_panel.visualization_panel.map_loaded.connect(
        #     config_panel.basic_config_wrapper
        #     .patrols_scheduler.send_points_data
        # )

        # home_panel.visualization_panel.update_points.connect(self.patrol_panel.update_points)
        # home_panel.visualization_panel.parent.pointsWindow.save_selected_points.connect(
        #     config_panel.basic_config_wrapper.patrols_scheduler.setPointsToVisit
        # )

        # self.patrols_scheduler.setPointsToVisit(points)

        self.tabs_widget['Home'].layout.addWidget(home_panel)
        self.tabs_widget['Configuracion'].layout.addWidget(config_panel)
        self.tabs_widget["Sistema de alertas"].layout.addWidget(LogPanel(node_manager=self.nodes_manager, parent=parent))

        [tab.setLayout(tab.layout) for tab in self.tabs_widget.values()]

        self.layout.addWidget(topbar)
        self.layout.addWidget(self.tabs)
        self.setLayout(self.layout)


if __name__ == "__main__":
    rospy.init_node("harold_start_launch", anonymous=True)
    app = QApplication(sys.argv)
    app.setStyleSheet("""
        QToolTip {
            background-color: #3498db;
            color: white;
            border: 1px solid lightgray;
            padding: 2px;
            border-radius: 3px;
        }
    """)
    ex = App()
    ex.show()
    sys.exit(app.exec())
