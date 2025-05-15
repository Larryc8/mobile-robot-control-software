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
    QFileDialog,
)
from PyQt5.QtCore import (
    Qt,
    pyqtSlot,
    pyqtSignal,
    QPropertyAnimation,
    QParallelAnimationGroup,
    QEasingCurve,
)
from datetime import datetime

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
        # print("home panel", parent)

        visualization_panel, self.patrol_panel, select_mode_panel, joypad = (
            VisualizationPanel(nodes_manager=nodes_manager, parent=parent),
            PatrolsPanel(parent=parent),
            # pannel(self),
            SelectModePanel(nodes_manager=nodes_manager),
            Joypad(),
            # PointsGenerator()
            # ImageViewer()
        )
        joypad.setEnabled(True)
        visualization_panel.update_points.connect(self.patrol_panel.update_points)

        self.layout.addWidget(visualization_panel, 0, 0, -1, 1)
        self.layout.addWidget(select_mode_panel, 0, 2, 1, 1)

        self.layout.addWidget(self.patrol_panel, 1, 2, 1, 1)
        # self.layout.addWidget(Color('red'), 1, 2, 1, 1)

        self.layout.addWidget(joypad, 2, 2)
        # self.layout.addWidget(btn1, 3, 2)
        # self.layout.setColumnStretch(2, 2)

        self.setLayout(self.layout)

    # def updatePoints(self, points):
    # self.patrol_panel.update_points(points)


class VisualizationPanel(QWidget):
    update_points = pyqtSignal(list)
    map_loaded = pyqtSignal(str)

    def __init__(self, nodes_manager=None, parent=None) -> None:
        super().__init__()
        self.layout = QGridLayout()
        self.buttons_layout = QHBoxLayout()
        self.parent = parent
        # ImageViewer()
        # self.parent.pointWindow = None
        self.parent.pointsWindow = ImageViewer()
        self.nodes = [
            {
                "package": "gmapping",
                "exec": "slam_gmapping",
                "name": "turtlebot3_slam_gmapping",
            }
        ]

        self.save_button, self.load_button, self.points_window_btn = (
            QPushButton("Soy un boton xd"),
            QPushButton("Cargar mapa"),
            QPushButton("puntos de interes"),
        )

        [
            self.buttons_layout.addWidget(button)
            for button in (self.load_button, self.save_button, self.points_window_btn)
        ]
        self.save_button.clicked.connect(self.saveMapClickHandler)
        self.load_button.clicked.connect(self.handleLoadMap)
        self.map_loaded.connect(self.parent.pointsWindow.load_map)

        self.save_button.setEnabled(False)
        self.points_window_btn.clicked.connect(self.show_points_window)
        self.parent.pointsWindow.save_selected_points.connect(self.handleSavePoints)

        map = MyViz(configfile="./config_navigation.rviz")
        self.layout.addWidget(map, 0, 0)
        self.layout.addLayout(self.buttons_layout, 1, 0)
        self.setLayout(self.layout)

    def handleLoadMap(self):
        try:
            
            file_path, _ = QFileDialog.getOpenFileName(
                self, "Open Image File", "",
                "Image Files (*.yaml)"
            )
            print(file_path)
            self.map_loaded.emit(file_path)

        except FileNotFoundError as e:
            print(e) 
        # file_path = "./map2.pgm"


    def saveMapClickHandler(self):
        pass

    def handleSavePoints(self, points):
        self.update_points.emit(points)
        pass

    def show_points_window(self, checked):
        # self.parent.w = None
        # if self.parent.pointsWindow is None:
        # self.parent.pointsWindow = ImageViewer()
        # print('None ;; multiwindows')
        self.parent.pointsWindow.show()


class SelectModePanel(QGroupBox):
    def __init__(self, nodes_manager=None, parent=None) -> None:
        super().__init__("select operation mode")
        self.setMaximumHeight(80)
        self.layout = QGridLayout()
        self.nodes_manager = nodes_manager
        # self.name = QLabel()
        # self.name.setText("Modo de operacion")
        self.nodes = [
            {
                "package": "gmapping",
                "exec": "slam_gmapping",
                "name": "turtlebot3_slam_gmapping",
            }
        ]

        auto_mode_button, manual_mode_button = (
            QPushButton("auto"),
            QPushButton("manual"),
        )
        # auto_mode_button.setEnabled(False)
        auto_mode_button.clicked.connect(self.handleAutoMode)

        self.layout.addWidget(auto_mode_button, 1, 0)
        self.layout.addWidget(manual_mode_button, 1, 1)

        self.setLayout(self.layout)

    def handleAutoMode(self):
        pass


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
        self.left_btn, self.right_btn = QPushButton("<"), QPushButton(">")

        [
            self.navigation_buttons.addWidget(button)
            for button in (self.left_btn, self.right_btn)
        ]

        self.create_btn.clicked.connect(self.add_patrol)
        self.start_patrols_btn.clicked.connect(self.start_patrols)
        self.delete_btn.clicked.connect(self.delete_patrols)
        self.left_btn.clicked.connect(self.set_previous_page)
        self.right_btn.clicked.connect(self.set_next_page)
        self.patrols_scheduler.patrol_finished.connect(self.patrol_finished)
        self.stop_patrols_btn.clicked.connect(self.stop_any_patrol)

        self.patrols_container = PatrolsContainer(
            self.patrols_scheduler.patrols_data,
            parent,
            patrols_scheduler=self.patrols_scheduler,
            callback=self.handleSinglepatrolExec,
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
        self.stop_patrols_btn.setEnabled(True)

    def stop_any_patrol(self, c):
        # self.patrols_container
        self.start_patrols_btn.setEnabled(True)
        self.create_btn.setEnabled(True)
        self.delete_btn.setEnabled(True)
        self.stop_patrols_btn.setEnabled(False)
        self.patrols_scheduler.cancel_task()
        print("PATROL STOPPED")

    def delete_patrols(self):
        toDelete = self.patrols_container.get_selected_patrolsid()
        self.patrols_scheduler.delete_patrols(toDelete)

    def handleSinglepatrolExec(self, x=None):
        self.start_patrols_btn.setEnabled(False)
        self.create_btn.setEnabled(False)
        self.delete_btn.setEnabled(False)

    def patrol_finished(self, x):
        self.start_patrols_btn.setEnabled(False)
        self.create_btn.setEnabled(False)
        self.delete_btn.setEnabled(False)

    def set_next_page(self):
        self.patrols_container.move_page_index(1)
        self.patrols_scheduler.get_current_patrols()

    def set_previous_page(self):
        self.patrols_container.move_page_index(-1)
        self.patrols_scheduler.get_current_patrols()

    def update_points(self, points):
        self.patrols_scheduler.pointsToVisit(points)
        print("from patrolpanel ", points)


class PatrolsContainer(QWidget):
    def __init__(
        self, patrols_data: dict, parent, patrols_scheduler, callback=None
    ) -> None:
        super().__init__()
        self.layout = QVBoxLayout()
        self.layout.setAlignment(Qt.AlignTop)
        self.patrols_data = patrols_data
        self.parent = parent
        self.layout.setSpacing(2)
        self.patrols = []
        self.patrols_scheduler = patrols_scheduler
        self.pageindex = 0
        self.max_element_view = 2
        self.element_count = 0
        self.isSinglePatrolExec = False
        self.callback = callback

        self.update_patrols(patrols_data)
        self.setLayout(self.layout)

    def update_patrols(self, patrols_data):
        [w.setParent(None) for w in self.patrols]
        [self.layout.removeWidget(w) for w in self.patrols]
        self.patrols = []
        self.element_count = len(patrols_data)
        patrols_items = list(patrols_data.items())

        for id, patrol in patrols_items[
            self.pageindex * self.max_element_view : self.max_element_view
            + self.pageindex * self.max_element_view
        ]:
            print(patrol.keys())
            days = list(patrol["days"].keys())
            time = patrol["time"]
            patrol_widget = Patrol(
                parent=self.parent,
                id=id,
                days=days,
                time=time,
                patrols_scheduler=self.patrols_scheduler,
            )
            patrol_widget.enable_single_patrol_exec.connect(self.handleSinglepatrolExec)
            self.patrols.append(patrol_widget)
        print("hola soy patrolcontainer", len(self.patrols), patrols_data)
        [self.layout.addWidget(patrol) for patrol in self.patrols]
        self.update()

    def move_page_index(self, move):
        self.pageindex = self.pageindex + move
        if (
            self.pageindex * self.max_element_view
            + self.max_element_view
            - self.element_count % 2
        ) > self.element_count:
            self.pageindex = 0
        if self.pageindex < 0:
            self.pageindex = 0

    def get_selected_patrolsid(self):
        return [patrol.selected for patrol in self.patrols if patrol.selected]

    def handleSinglepatrolExec(self, x):
        self.isSinglePatrolExec = x
        self.callback()

    def patrol_finished(self):
        pass

    def add_patrol(self, patrol):
        pass


class Patrol(QGroupBox):
    enable_single_patrol_exec = pyqtSignal(bool)

    def __init__(
        self, parent, id: str = "999", days=[], time=2233, patrols_scheduler=None
    ):
        super().__init__()
        self.days = days
        self.id = id
        self.patrols_scheduler = patrols_scheduler
        self.time = time
        # self.setStyleSheet("border-color: gray")
        self.parent = parent
        self.progress = QProgressBar()
        self.points_checked_label = QLabel()
        self.points_checked_label.setText("activo 1/34")
        self.progress.setTextVisible(False)
        self.progress.setValue(10)
        self.progress.setMaximumHeight(6)
        self.progress.setEnabled(False)
        self.checked = False
        self.setObjectName("mainWidget")

        self.layout = QGridLayout()
        self.patrol_name = QLabel()
        self.patrol_name.setText(" ".join(days))
        # self.setStyleSheet("""
        #     #mainWidget {
        #         border-radius: 5px;
        #         padding: 5px;
        #     }
        #
        #     #mainWidget QLabel,
        #     #mainWidget QPushButton {
        #         border-radius: 0;
        #     }
        # """)

        (delete_botton, self.exec_button, self.checkbox, repeate_patrol_botton) = (
            QPushButton("exec"),
            QPushButton(),
            QCheckBox(),
            QPushButton("loop"),
        )
        self.exec_button.clicked.connect(self.task)
        self.menu = QMenu()
        edit = self.menu.addAction("editar")
        edit.triggered.connect(self.edit_patrol)

        force_exec = self.menu.addAction("ejecutar")
        force_exec.triggered.connect(self.force_execution)
        force_exec.setEnabled(False)
        self.exec_button.setMenu(self.menu)

        self.checkbox.stateChanged.connect(self.select)
        self.patrols_scheduler.patrol_progress.connect(
            lambda x: self.progress.setValue(10 * x)
        )

        self.layout.addWidget(self.patrol_name, 1, 3)
        self.layout.addWidget(self.exec_button, 1, 4)
        self.layout.addWidget(self.progress, 2, 0, 1, 4)
        self.layout.addWidget(self.points_checked_label, 2, 4, 1, 1)
        self.layout.addWidget(self.checkbox, 1, 0, alignment=Qt.AlignLeft)
        self.setLayout(self.layout)

        self.animate_show()

    def task(self):
        self.popup = PatrolsMenu(
            self.parent, selected_days=self.days, patrolid=self.id, time=self.time
        )
        self.popup.update_date.connect(self.patrols_scheduler.update_patrol)
        self.popup.show_popup()

    def edit_patrol(self):
        # print("Option 1 selected")
        self.task()

    def force_execution(self):
        self.enable_single_patrol_exec.emit(True)
        # self.exec_botton.setEnabled(False)
        print("Option 2 selected")

    def select(self):
        self.checked = self.checkbox.isChecked()

    @property
    def selected(self):
        if self.checked:
            return self.id
        else:
            return None

    def animate_show(self):
        anim = QPropertyAnimation(self, b"maximumWidth")
        anim.setDuration(200)
        anim.setStartValue(0)
        anim.setEndValue(250)
        anim.setEasingCurve(QEasingCurve.OutBack)

        self.setGraphicsEffect(None)  # Clear any previous effects
        opacity_effect = self.graphicsEffect()
        if not opacity_effect:
            from PyQt5.QtWidgets import QGraphicsOpacityEffect
            opacity_effect = QGraphicsOpacityEffect(self)
            self.setGraphicsEffect(opacity_effect)

        opacity_anim = QPropertyAnimation(opacity_effect, b"opacity")
        opacity_anim.setDuration(200)
        opacity_anim.setStartValue(0)
        opacity_anim.setEndValue(1)
        opacity_anim.setEasingCurve(QEasingCurve.InOutQuad)
        # Run animations in parallel
        group = QParallelAnimationGroup(self)
        group.addAnimation(anim)
        group.addAnimation(opacity_anim)
        group.start()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    t = HomePanel()
    # t = PatrolsPanel()
    # t = VisualizationPanel()
    t.show()

    sys.exit(app.exec())
