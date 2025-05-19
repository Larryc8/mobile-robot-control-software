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
    QMessageBox,
    QStyle,
)
from PyQt5.QtCore import (
    Qt,
    pyqtSlot,
    pyqtSignal,
    QPropertyAnimation,
    QParallelAnimationGroup,
    QEasingCurve,
)

from pyqttoast import Toast, ToastPreset
from datetime import datetime
import rospy

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

        # joypad.setEnabled(True)
        select_mode_panel.set_operation_mode.connect(joypad.update_operation_mode)
        select_mode_panel.set_operation_mode.connect(visualization_panel.update_operation_mode)
        visualization_panel.update_points.connect(self.patrol_panel.update_points)
        visualization_panel.enable.connect(select_mode_panel.enable)

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
    update_points = pyqtSignal(dict)
    map_loaded = pyqtSignal(str)
    map_change = pyqtSignal(bool)
    enable = pyqtSignal(str, bool)

    def __init__(self, nodes_manager=None,parent=None) -> None:
        super().__init__()
        self.layout = QGridLayout()
        self.buttons_layout = QHBoxLayout()
        self.parent = parent
        self.nodes_manager = nodes_manager
        # ImageViewer()
        # self.parent.pointWindow = None
        self.currentOperationMode = 'manual'
        self.parent.pointsWindow = ImageViewer(nodes_manager=nodes_manager, parent=self.parent)
        self.nodes = [
            {
                "package": "gmapping",
                "exec": "slam_gmapping",
                "name": "turtlebot3_slam_gmapping",
            }
        ]

        self.save_button, self.load_button, self.points_window_btn, self.create_map_btn = (
            QPushButton("Guardar mi mapita"),
            QPushButton("Cargar mapa"),
            QPushButton("puntos de interes"),
            QPushButton("Crear mapa compa"),
        )

        [
            self.buttons_layout.addWidget(button)
            for button in (self.load_button, self.save_button, self.points_window_btn, self.create_map_btn)
        ]
        self.save_button.clicked.connect(self.saveMapClickHandler)
        self.load_button.clicked.connect(self.handleLoadMap)
        self.map_loaded.connect(self.parent.pointsWindow.load_map)
        self.create_map_btn.clicked.connect(self.handleCreateMap)

        # self.save_button.setEnabled(False)
        self.points_window_btn.clicked.connect(self.show_points_window)
        self.parent.pointsWindow.save_selected_points.connect(self.handleSavePoints)

        map = MyViz(configfile="./config_navigation.rviz")
        self.layout.addWidget(map, 0, 0)
        self.layout.addLayout(self.buttons_layout, 1, 0)
        self.setLayout(self.layout)

    def handleLoadMap(self):
        self.nodes_manager.stopNodes(["turtlebot3_slam_gmapping"])
        if self.nodes_manager.topicHasPublisher('/map_metadata'):
            button = QMessageBox.critical(
                self,
                "oh no!",
                "Ya hay un mapa, CONSERVAR el que tienes?",
                buttons=QMessageBox.No | QMessageBox.Yes,
                defaultButton=QMessageBox.Yes,
            )
            try:
                if button == QMessageBox.Yes:
                    print("OK!")
                    return
            except AttributeError as error:
                print(error)

        try:
            self.nodes_manager.bringUpStop()
            file_path, _ = QFileDialog.getOpenFileName(
                self, "Open Image File", "", "Image Files (*.yaml)"
            )
            print(file_path)
            self.map_loaded.emit(file_path)
            map = file_path
            self.nodes_manager.stopNodes(['map_server'])
            self.nodes_manager.startNodes(
                self.nodes_manager.initNodes(
                    [
                        {
                            "package": "map_server",
                            "name": "map_server",
                            "exec": "map_server",
                            "arg": f"{map}",
                        }
                    ]
                )
            )
            if file_path:
                toast = Toast(self.parent)
                toast.setDuration(3000)  # Hide after 5 seconds
                toast.setTitle('Success!.')
                toast.setText('Map upload completed')
                toast.applyPreset(ToastPreset.SUCCESS)  # Apply style preset
                Toast.setPositionRelativeToWidget(self.parent)  
                toast.show()
                return

            toast = Toast(self.parent)
            toast.setDuration(3000)  # Hide after 5 seconds
            toast.setTitle('ERROR!.')
            toast.setText('NO Map slected')
            toast.applyPreset(ToastPreset.ERROR)  # Apply style preset
            Toast.setPositionRelativeToWidget(self.parent)  
            toast.show()


            if self.save_button.isEnabled():
                self.save_button.setEnabled(False)

        except FileNotFoundError as error:
            print(error)

    def saveMapClickHandler(self):
        if self.nodes_manager.topicHasPublisher('/scan'):
            self.save_button.setEnabled(False)
            self.nodes_manager.save_map()
            return

        button = QMessageBox.critical(
                self,
                "oh no!",
                "No hay datos del lidar?",
            )

    def handleSavePoints(self, points):
        self.update_points.emit(points)
        pass

    def handleCreateMap(self):
        if not self.nodes_manager.topicHasPublisher('/scan'):
            dlg = QMessageBox(self)
            dlg.setWindowTitle("I have a question!")
            dlg.setText("No LiDar Data, Ayyyy!!! \n Anda pasha BOBO")
            button = dlg.exec()
            return

        if self.currentOperationMode == 'manual':
            self.enable.emit('localization', False)
            self.save_button.setEnabled(True)
            self.nodes_manager.bringUpStop()
            self.nodes_manager.stopNodes(['map_server', 'amcl'])
            # self.create_map_btn.setEnabled(False)
            self.nodes_manager.bringUpStart()
            self.nodes_manager.startNodes(self.nodes_manager.initNodes(self.nodes))
            return 

        dlg = QMessageBox(self)
        dlg.setWindowTitle("I have a question!")
        dlg.setText("Solo se pueden crear mapas en el modo manual")
        dlg.exec()

    def show_points_window(self, checked):
        # self.parent.w = None
        # if self.parent.pointsWindow is None:
        # self.parent.pointsWindow = ImageViewer()
        # print('None ;; multiwindows')
        self.parent.pointsWindow.show_win()
        # self.parent.pointsWindow.save_points()

    def update_operation_mode(self, mode):
        self.currentOperationMode = mode


class SelectModePanel(QGroupBox):
    set_operation_mode = pyqtSignal(str)
    def __init__(self, nodes_manager=None, parent=None) -> None:
        super().__init__("select operation mode")
        self.setMaximumHeight(100)
        self.layout = QGridLayout()
        self.nodes_manager = nodes_manager
        self.nodes = [
           {
                "package": "amcl",
                "exec": "amcl",
                "name": "amcl",
            },
             {
                "package": "move_base",
                "exec": "move_base",
                "name": "move_base",
                "respawn": True
            },
        ]
        # self.name = QLabel()
        # self.name.setText("Modo de operacion")

        self.auto_mode_button, self.manual_mode_button, self.localize_button = (
            QPushButton("auto"),
            QPushButton("manual"),
            QPushButton('localize robot')
        )
        self.auto_mode_button.setEnabled(True)
        self.manual_mode_button.setEnabled(False)
        # auto_mode_button.setEnabled(False)
        self.auto_mode_button.clicked.connect(self.handleAutoMode)
        self.manual_mode_button.clicked.connect(self.handleManualMode)
        self.localize_button.clicked.connect(self.handleLocalization)

        self.layout.addWidget(self.auto_mode_button, 1, 0)
        self.layout.addWidget(self.manual_mode_button, 1, 1)
        self.layout.addWidget(self.localize_button, 2, 1)

        self.setLayout(self.layout)
        # self.localize_button.hide()

    def handleAutoMode(self):
        self.set_operation_mode.emit('auto')
        self.nodes_manager.bringUpStop()
        self.nodes_manager.bringUpStart()

        self.nodes_manager.startNodes(self.nodes_manager.initNodes(self.nodes))
        self.auto_mode_button.setEnabled(False)
        self.manual_mode_button.setEnabled(True)
        self.localize_button.setEnabled(False)

    def handleManualMode(self):
        self.set_operation_mode.emit('manual')
        self.auto_mode_button.setEnabled(True)
        self.manual_mode_button.setEnabled(False)
        self.localize_button.setEnabled(True)

    def handleLocalization(self):
        if not self.nodes_manager.topicHasPublisher('/scan'):
            dlg = QMessageBox(self)
            dlg.setWindowTitle("I have a question!")
            dlg.setText("No LiDar Data, Ayyyy!!! \n Anda pasha BOBO")
            button = dlg.exec()
            return

        if not self.nodes_manager.topicHasPublisher('/map_metadata'):
            dlg = QMessageBox(self)
            dlg.setWindowTitle("I have a question!")
            dlg.setText("Agrega el mapa primer, Ayyyy!!! \n Anda pasha BOBO")
            button = dlg.exec()
            return
        self.nodes_manager.bringUpStop()
        self.nodes_manager.bringUpStart()
         
        self.nodes_manager.stopNodes(['amcl'])
        self.nodes_manager.startNodes(self.nodes_manager.initNodes(self.nodes[:1]))

    def enable(self, object, state):
        if object == 'localization':
            self.localize_button.setEnabled(state)


    # def update_operation_mode(self, mode):
    #     self.currentOperationMode = mode



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

        self.patrols_container = PatrolsContainer(
            self.patrols_scheduler.patrols_data,
            parent,
            patrols_scheduler=self.patrols_scheduler,
            callback=self.handleSinglepatrolExec,
        )

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
        max_pag, pag_index = self.patrols_container.move_page_index(0)
        self.patrol_indexing = QLabel(f'{pag_index}/{max_pag}')
        self.left_btn, self.right_btn = QPushButton("<"), QPushButton(">")

        [
            self.navigation_buttons.addWidget(button)
            for button in (self.patrol_indexing, self.left_btn, self.right_btn)
        ]

        self.create_btn.clicked.connect(self.add_patrol)
        self.start_patrols_btn.clicked.connect(self.start_patrols)
        self.delete_btn.clicked.connect(self.delete_patrols)
        self.left_btn.clicked.connect(self.set_previous_page)
        self.right_btn.clicked.connect(self.set_next_page)
        self.patrols_scheduler.patrol_finished.connect(self.patrol_finished)
        self.stop_patrols_btn.clicked.connect(self.stop_any_patrol)

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
        # max_pag, pag_index = self.patrols_container.move_page_index(0)
        # self.patrol_indexing.setText(f'{pag_index}/{max_pag}')
        # pass

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
        # max_pag, pag_index = self.patrols_container.move_page_index(0)
        # self.patrol_indexing.setText(f'{pag_index}/{max_pag}')

    def handleSinglepatrolExec(self, x=None):
        self.start_patrols_btn.setEnabled(False)
        self.create_btn.setEnabled(False)
        self.delete_btn.setEnabled(False)

    def patrol_finished(self, x):
        self.start_patrols_btn.setEnabled(False)
        self.create_btn.setEnabled(False)
        self.delete_btn.setEnabled(False)

    def set_next_page(self):
        max_pag, pag_index = self.patrols_container.move_page_index(1)
        self.patrol_indexing.setText(f'{pag_index}/{max_pag}')
        # print('nex page fuc', a)
        self.patrols_scheduler.get_current_patrols()

    def set_previous_page(self):
        max_pag, pag_index = self.patrols_container.move_page_index(-1)
        self.patrol_indexing.setText(f'{pag_index}/{max_pag}')
        self.patrols_scheduler.get_current_patrols()

    def update_patrols_indexing_label(self, step=0):
        max_pag, pag_index = self.patrols_container.move_page_index(step)
        self.patrol_indexing.setText(f'{pag_index}/{max_pag}')
        # self.patrol_indexing.setText(f'{pag_index}/{max_pag}')
        pass

    def update_points(self, points):
        self.patrols_scheduler.setPointsToVisit(points)
        # print("from patrolpanel "ToDelete, points)
        pass


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

        max_pages_count = self.element_count// self.max_element_view + self.element_count% self.max_element_view

        return (max_pages_count, self.pageindex) 

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

        (delete_botton, self.menu_button, self.checkbox, repeate_patrol_botton) = (
            QPushButton("exec"),
            QPushButton(),
            QCheckBox(),
            QPushButton("loop"),
        )
        self.menu_button.clicked.connect(self.task)
        self.menu = QMenu()
        edit = self.menu.addAction("editar")
        edit.triggered.connect(self.edit_patrol)

        force_exec = self.menu.addAction("ejecutar")
        force_exec.triggered.connect(self.force_execution)
        force_exec.setEnabled(False)
        self.menu_button.setMenu(self.menu)

        pixmapi = QStyle.SP_ComputerIcon
        icon = self.style().standardIcon(pixmapi)
        self.menu_button.setIcon(icon)
        self.menu_button.setMaximumWidth(30)

        self.checkbox.stateChanged.connect(self.select)
        self.patrols_scheduler.patrol_progress.connect(
            lambda x: self.progress.setValue(10 * x)
        )

        self.layout.addWidget(self.patrol_name, 1, 3)
        self.layout.addWidget(self.menu_button, 1, 4,  alignment=Qt.AlignRight)
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
