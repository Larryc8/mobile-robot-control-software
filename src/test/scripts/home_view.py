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
    QStackedLayout,
    QMenu,
    QProgressBar,
    QMenu,
    QAction,
)
from PyQt5.QtCore import Qt, pyqtSlot, pyqtSignal
from datetime import datetime

# from test_patrol_tree_view import pannel


# from points_manager import PointsGenerator
from better_image_display import ImageViewer
from joystick import Joypad
from patrols_scheduler import PatrolsEscheduler
from patrol_menu import PatrolsMenu
from rview import MyViz


class HomePanel(QWidget):
    def __init__(self, nodes_manager=None, parent=None):
        super().__init__()
        # self.resize(900, 900)
        self.layout = QGridLayout()
        # self.showMaximized()
        # self.layout = QHBoxLayout()
        # [self.layout.addWidget(element) for element in (VisualizationPanel(), PatrolsPanel())]
        print("home panel", parent)

        visualization_panel, patrol_panel, select_mode_panel, joypad = (
            VisualizationPanel(nodes_manager=nodes_manager),
            PatrolsPanel(parent=parent),
            # pannel(self),
            SelectModePanel(),
            Joypad(),
            # PointsGenerator()
            # ImageViewer()
        )
        joypad.setEnabled(True)

        self.layout.addWidget(visualization_panel, 0, 0, -1, 1)
        self.layout.addWidget(select_mode_panel, 0, 2, 1, 1)

        self.layout.addWidget(patrol_panel, 1, 2, 1, 1)
        # self.layout.addWidget(Color('red'), 1, 2, 1, 1)

        self.layout.addWidget(joypad, 2, 2)
        # self.layout.addWidget(btn1, 3, 2)
        # self.layout.setColumnStretch(2, 2)

        self.setLayout(self.layout)


class VisualizationPanel(QWidget):
    def __init__(self, nodes_manager=None, parent=None) -> None:
        super().__init__()
        self.layout = QGridLayout()
        self.buttons_layout = QHBoxLayout()

        self.save_button, self.load_button = (
            QPushButton("cargar mapa"),
            QPushButton("borrar punto de initeres"),
        )

        [
            self.buttons_layout.addWidget(button)
            for button in (self.load_button, self.save_button)
        ]
        self.save_button.clicked.connect(self.saveMapClickHandler)
        self.load_button.clicked.connect(self.loadMapClickHandler)
        self.save_button.setEnabled(False)

        map = MyViz(configfile="./config_navigation.rviz")
        self.layout.addWidget(map, 0, 0)
        self.layout.addLayout(self.buttons_layout, 1, 0)
        self.setLayout(self.layout)

    def loadMapClickHandler(self):
        pass

    def saveMapClickHandler(self):
        pass


class SelectModePanel(QGroupBox):
    def __init__(self, nodes_manager=None) -> None:
        super().__init__("select operation mode")
        self.setMaximumHeight(80)
        self.layout = QGridLayout()
        # self.name = QLabel()
        # self.name.setText("Modo de operacion")

        auto_mode_button, manual_mode_button = (
            QPushButton("auto"),
            QPushButton("manual"),
        )
        auto_mode_button.setEnabled(False)

        self.layout.addWidget(auto_mode_button, 1, 0)
        self.layout.addWidget(manual_mode_button, 1, 1)

        self.setLayout(self.layout)


class PatrolsPanel(QGroupBox):
    def __init__(self, nodes_manager=None, parent=None) -> None:
        super().__init__("Patrols")
        # self.setMaximumHeight(500)
        self.setMaximumWidth(300)
        # self.scroll_area = QScrollArea()
        self.patrols_panel = QWidget()
        self.patrols_panel.layout = QVBoxLayout()
        self.layout = QVBoxLayout()
        self.patrols_scheduler = PatrolsEscheduler()
        self.parent = parent

        # self.layout.addWidget(self.name)
        (
            self.create_btn,
            self.delete_btn,
            self.start_patrols_btn,
            self.stop_patrols_btn,
        ) = (
            QPushButton("crear"),
            QPushButton("delete"),
            QPushButton("start"),
            QPushButton("stop"),
        )
        self.control_panel_buttons = QHBoxLayout()
        [
            self.control_panel_buttons.addWidget(button)
            for button in [
                self.create_btn,
                self.delete_btn,
                self.start_patrols_btn,
                self.stop_patrols_btn,
            ]
        ]

        self.navigation_buttons = QHBoxLayout()
        self.btn1, btn2 = QPushButton("<"), QPushButton(">")

        [self.navigation_buttons.addWidget(button) for button in (self.btn1, btn2)]

        self.create_btn.clicked.connect(self.add_patrol)
        self.start_patrols_btn.clicked.connect(self.start_patrols)
        self.delete_btn.clicked.connect(self.delete_patrols)

        self.patrols_container = PatrolsContainer(
            self.patrols_scheduler.patrols_data,
            parent,
            patrols_scheduler=self.patrols_scheduler,
        )
        self.patrols_scheduler.update_patrols_view.connect(
            self.patrols_container.update_patrols
        )
        # self.patrols_scheduler.

        self.layout.addLayout(self.control_panel_buttons)
        self.layout.addLayout(self.navigation_buttons)
        self.layout.addWidget(self.patrols_container)

        self.setLayout(self.layout)

    def add_patrol(self, check):
        self.popup = PatrolsMenu(
            self.parent, selected_days=[], patrolid=datetime.now().timestamp()
        )
        self.popup.update_date.connect(self.patrols_scheduler.update_patrol)
        self.popup.show_popup()
        pass

    def start_patrols(self):
        self.patrols_scheduler.start_patrols()
        self.start_patrols_btn.setEnabled(False)
        self.create_btn.setEnabled(False)
        self.delete_btn.setEnabled(False)

    def delete_patrols(self):
        toDelete = self.patrols_container.get_selected_patrolsid()
        self.patrols_scheduler.delete_patrols(toDelete)


class PatrolsContainer(QWidget):
    def __init__(self, patrols_data: dict, parent, patrols_scheduler) -> None:
        super().__init__()
        self.layout = QVBoxLayout()
        self.layout.setAlignment(Qt.AlignTop)
        self.patrols_data = patrols_data
        self.parent = parent
        self.layout.setSpacing(0)
        self.patrols = []
        self.patrols_scheduler = patrols_scheduler
        self.update_patrols(patrols_data)
        self.pageindex = 0

        self.setLayout(self.layout)

    def update_patrols(self, patrols_data):
        [w.setParent(None) for w in self.patrols]
        [self.layout.removeWidget(w) for w in self.patrols]
        self.patrols = []
        for id, patrol in patrols_data.items():
            print(patrol.keys())
            days = list(patrol["days"].keys())
            time = patrol["time"]
            self.patrols.append(
                Patrol(
                    parent=self.parent,
                    id=id,
                    days=days,
                    time=time,
                    patrols_scheduler=self.patrols_scheduler,
                )
            )

        print("hola soy patrolcontainer", len(self.patrols), patrols_data)
        [self.layout.addWidget(patrol) for patrol in self.patrols]
        self.update()

    def get_selected_patrolsid(self):
        return [patrol.selected for patrol in self.patrols if patrol.selected]

    def add_patrol(self, patrol):
        pass


class Patrol(QWidget):
    def __init__(
        self, parent, id: str = "999", days=[], time=2233, patrols_scheduler=None
    ):
        super().__init__()
        self.days = days
        self.id = id
        self.patrols_scheduler = patrols_scheduler
        self.time = time
        self.setStyleSheet("border-color: gray")
        self.parent = parent
        self.progress = QProgressBar()
        self.points_checked_label = QLabel()
        self.points_checked_label.setText("activo 1/34")
        self.progress.setTextVisible(False)
        self.progress.setValue(10)
        self.progress.setMaximumHeight(6)
        self.progress.setEnabled(False)
        self.checked = False

        self.layout = QGridLayout()
        self.patrol_name = QLabel()
        self.patrol_name.setText(" ".join(days))

        (delete_botton, self.exec_botton, self.checkbox, repeate_patrol_botton) = (
            QPushButton("exec"),
            QPushButton(),
            QCheckBox(),
            QPushButton("loop"),
        )
        self.exec_botton.clicked.connect(self.task)
        self.menu = QMenu()
        edit = self.menu.addAction("editar")
        edit.triggered.connect(self.edit_patrol)

        force_exec = self.menu.addAction("ejecutar")
        force_exec.triggered.connect(self.force_execution)
        self.exec_botton.setMenu(self.menu)

        self.checkbox.stateChanged.connect(self.select)

        self.layout.addWidget(self.patrol_name, 1, 3)
        self.layout.addWidget(self.exec_botton, 1, 4)
        self.layout.addWidget(self.progress, 2, 0, 1, 4)
        self.layout.addWidget(self.points_checked_label, 2, 4, 1, 1)
        self.layout.addWidget(self.checkbox, 1, 0, alignment=Qt.AlignLeft)
        self.setLayout(self.layout)

    def task(self):
        self.popup = PatrolsMenu(self.parent, selected_days=self.days, patrolid=self.id, time=self.time)
        self.popup.update_date.connect(self.patrols_scheduler.update_patrol)
        self.popup.show_popup()

    def edit_patrol(self):
        # print("Option 1 selected")
        self.task()

    def force_execution(self):
        print("Option 2 selected")

    def select(self):
        self.checked = self.checkbox.isChecked()

    @property
    def selected(self):
        if self.checked:
            return self.id
        else:
            return None


if __name__ == "__main__":
    app = QApplication(sys.argv)
    t = HomePanel()
    # t = PatrolsPanel()
    # t = VisualizationPanel()
    t.show()

    sys.exit(app.exec())
