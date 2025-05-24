import sys
import re
from typing import List

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
    QGraphicsOpacityEffect,
)
from PyQt5.QtCore import (
    Qt,
    pyqtSlot,
    pyqtSignal,
    QPropertyAnimation,
    QParallelAnimationGroup,
    QEasingCurve,
    QTimer,
)
from PyQt5.QtGui import QIcon, QPixmap, QTransform

from pyqttoast import Toast, ToastPreset
from datetime import datetime
import rospy
from styles.buttons import (
    border_button_style,
    border_button_style_danger,
    primary_button_style,
    secondary_button_style,
    colored_button_style,
    tertiary_button_style,
    toggle_button_style,
    minimal_button_style,
    patrol_checkbox_style,
)

from styles.labels import inactive_label_style, minimal_label_style, muted_label_style

from styles.patrols import patrol_base_style, patrol_selected_style

# from points_manager import PointsGenerator
from better_image_display import ImageViewer
from joystick import Joypad
from patrols_scheduler import PatrolsEscheduler
from patrol_menu import PatrolsMenu
from rview import MyViz
from input_textdialog import InputDialog 


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
            PatrolsPanel(parent=parent, nodes_manager=nodes_manager),
            # pannel(self),
            SelectModePanel(nodes_manager=nodes_manager, parent=parent),
            Joypad(),
            # PointsGenerator()
            # ImageViewer()
        )

        # joypad.setEnabled(True)
        select_mode_panel.set_operation_mode.connect(joypad.update_operation_mode)
        select_mode_panel.set_operation_mode.connect(
            visualization_panel.update_operation_mode
        )
        visualization_panel.update_points.connect(self.patrol_panel.update_points)
        visualization_panel.enable.connect(select_mode_panel.enable)

        self.patrol_panel.patrols_container.patrols_scheduler.points_scheduler.points_state.connect(
            visualization_panel.parent.pointsWindow.update_points_state
        )

        self.layout.addWidget(visualization_panel, 0, 0, 8, 1)
        self.layout.addWidget(select_mode_panel, 0, 2, 1, 1)

        self.layout.addWidget(self.patrol_panel, 1, 2, 6, 1)
        # self.layout.addWidget(Color('red'), 1, 2, 1, 1)

        self.layout.addWidget(joypad, 7, 2)
        # self.layout.addWidget(btn1, 3, 2)
        self.layout.setColumnStretch(2, 2)
        self.layout.setRowStretch(1, 2)


        self.setLayout(self.layout)

    # def updatePoints(self, points):
    # self.patrol_panel.update_points(points)


class VisualizationPanel(QWidget):
    update_points = pyqtSignal(dict)
    map_loaded = pyqtSignal(str)
    map_change = pyqtSignal(bool)
    enable = pyqtSignal(str, bool)

    def __init__(self, nodes_manager=None, parent=None) -> None:
        super().__init__()
        self.layout = QGridLayout()
        self.buttons_layout = QGridLayout()
        self.rviz_options_layout = QHBoxLayout()
        self.parent = parent
        self.nodes_manager = nodes_manager
        self.isCreate = True
        # ImageViewer()
        # self.parent.pointWindow = None
        self.currentOperationMode = "manual"
        self.parent.pointsWindow = ImageViewer(
            nodes_manager=self.nodes_manager, parent=self.parent
        )
        self.nodes = [
            {
                "package": "gmapping",
                "exec": "slam_gmapping",
                "name": "turtlebot3_slam_gmapping",
            }
        ]

        (
            self.save_map_button,
            self.load_map_button,
            self.points_window_btn,
            self.create_map_btn,
            self.followrobot_check,
        ) = (
            QPushButton("Guardar mi mapita"),
            QPushButton("Cargar mapa"),
            QPushButton("puntos de interes"),
            QPushButton("Crear mapa"),
            QCheckBox(),
        )

        [
            self.buttons_layout.addWidget(button, r, c)
            for button, r, c, rx, cx in (
                (self.load_map_button, 0, 0, 1, 4),
                # (self.save_map_button, 0, 1, 1, 3),
                (self.points_window_btn, 0, 2, 1, 3),
                (self.create_map_btn, 0, 1, 1, 3),
            )
        ]
        self.load_map_button.setStyleSheet(primary_button_style)
        self.points_window_btn.setStyleSheet(secondary_button_style)
        self.create_map_btn.setStyleSheet(border_button_style)
        self.save_map_button.setStyleSheet(primary_button_style)

        icon_start = QApplication.style().standardIcon(QStyle.SP_MediaPlay)
        self.create_map_btn.setIcon(icon_start)
        # self.create_map_btn.setText('ddd')
        
        icon_load = QApplication.style().standardIcon(QStyle.SP_FileLinkIcon)
        self.load_map_button.setIcon(icon_load)

        [
            self.rviz_options_layout.addWidget(widget, alignment=Qt.AlignLeft)
            for widget in (self.followrobot_check,)
        ]
        self.followrobot_check.setText("hola soy harold")
        # self.rviz_options_layout.setContentsMargins(0, 0, 0, 0)
        # self.rviz_options_layout.setSpacing(0)
        # self.rviz_options_layout.setAlignment(Qt.AlignLeft)
        # self.followrobot_label.setStyleSheet("background-color: #f0f0f0;")

        # self.save_map_button.clicked.connect(self.saveMapClickHandler)
        self.load_map_button.clicked.connect(self.handleLoadMap)
        self.map_loaded.connect(self.parent.pointsWindow.load_map)
        self.create_map_btn.clicked.connect(self.toggleCreaeteSaveMap)
        self.followrobot_check.stateChanged.connect(self.setFollowRobot)
        self.save_map_button.hide()


        # self.save_button.setEnabled(False)
        self.points_window_btn.clicked.connect(self.show_points_window)
        self.parent.pointsWindow.save_selected_points.connect(self.handleSavePoints)

        self.rviz = MyViz(configfile="./config_navigation.rviz")
        self.layout.addLayout(self.rviz_options_layout, 0, 0)
        self.layout.addWidget(BatteryIndicator(), 0, 2)
        self.layout.addWidget(self.rviz, 1, 0, 1, 3)
        self.layout.addLayout(self.buttons_layout, 2, 0, 1, 3)
        self.setLayout(self.layout)


    def toggleCreaeteSaveMap(self):
        if self.isCreate:
            self.handleCreateMap()
        else:
            self.saveMapClickHandler()

    def setFollowRobot(self, x):
        self.rviz.setUp("followrobot", x)
        if not x:
            self.rviz.setUp("scale", 60)
            self.rviz.setUp("globalframe", "map")

    def handleLoadMap(self):
        self.nodes_manager.stopNodes(["turtlebot3_slam_gmapping"])
        self.rviz.setUp("globalframe", "map")
        if self.nodes_manager.topicHasPublisher("/map_metadata"):
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
            self.nodes_manager.stopNodes(["map_server"])
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
            self.save_map_button.hide()
            # self.rviz.setUp('globalframe', 'odom')
            if file_path:
                toast = Toast(self.parent)
                toast.setDuration(3000)  # Hide after 5 seconds
                toast.setTitle("Success!.")
                toast.setText("Map upload completed")
                toast.applyPreset(ToastPreset.SUCCESS)  # Apply style preset
                Toast.setPositionRelativeToWidget(self.parent)
                toast.show()
                return

            toast = Toast(self.parent)
            toast.setDuration(3000)  # Hide after 5 seconds
            toast.setTitle("ERROR!.")
            toast.setText("NO Map slected")
            toast.applyPreset(ToastPreset.ERROR)  # Apply style preset
            Toast.setPositionRelativeToWidget(self.parent)
            toast.show()

            self.save_map_button.hide()

        except FileNotFoundError as error:
            print(error)

    def saveMapClickHandler(self):
        if not self.nodes_manager.nodeIsRunning('turtlebot3_slam_gmapping'):
            toast = Toast(self.parent)
            toast.setDuration(4000)  # Hide after 5 seconds
            toast.setTitle('Fallo en la creacion del mapa')
            toast.setText("El nodo turtlebot3_slam_gmapping no disponible")
            toast.applyPreset(ToastPreset.ERROR)  # Apply style preset
            Toast.setPositionRelativeToWidget(self.parent)
            toast.show()
            return 

        # if self.nodes_manager.topicHasPublisher("/scan"):
        # self.save_map_button.setEnabled(False)
        dialog = InputDialog(self.parent)
        dialog.exec_()   
        print('VizPanel,dialog', dialog.filename)
        mapname = dialog.filename
        self.nodes_manager.save_map(mapname)
        # self.nodes_manager.stopNodes(['turtlebot3_slam_gmapping'])
        toast = Toast(self.parent)
        toast.setDuration(4000)  # Hide after 5 seconds
        toast.setTitle('Se guardo el mapa exitosamente')
        toast.setText("Ahora puedes usar tu mapa para la navegacion")
        toast.applyPreset(ToastPreset.SUCCESS)  # Apply style preset
        Toast.setPositionRelativeToWidget(self.parent)

        icon = QApplication.style().standardIcon(QStyle.SP_MediaPlay)
        self.create_map_btn.setIcon(icon)
        self.create_map_btn.setText('Crear Mapa')
        self.isCreate = True
        toast.show()

    def handleSavePoints(self, points):
        self.update_points.emit(points)
        pass

    def handleCreateMap(self):
        if not self.currentOperationMode == "manual":
            toast = Toast(self.parent)
            toast.setDuration(5000)  # Hide after 5 seconds
            toast.setTitle("Solo se puede crear mapa en el modo manual")
            toast.setText("Seleccione el modo de operacion manual")
            toast.applyPreset(ToastPreset.INFORMATION)  # Apply style preset
            Toast.setPositionRelativeToWidget(self.parent)
            toast.show()
            return

        if not self.nodes_manager.topicHasPublisher("/scan"):
            toast = Toast(self.parent)
            toast.setDuration(5000)  # Hide after 5 seconds
            toast.setTitle("No hay datos del lidar disponible")
            toast.setText("el pepe manito")
            toast.applyPreset(ToastPreset.ERROR)  # Apply style preset
            Toast.setPositionRelativeToWidget(self.parent)
            toast.show()
            # dlg = QMessageBox(self)
            # dlg.setWindowTitle("I have a question!")
            # dlg.setText("No LiDar Data, Ayyyy!!! \n Anda pasha BOBO")
            # button = dlg.exec()
            return

        self.enable.emit("localization", False)
        # self.save_map_button.show()

        icon = QApplication.style().standardIcon(QStyle.SP_DialogSaveButton)
        self.create_map_btn.setIcon(icon)
        self.create_map_btn.setText('Guardar')
        self.isCreate = False

        self.nodes_manager.bringUpStop()
        self.nodes_manager.stopNodes(["map_server", "amcl", "turtlebot3_slam_gmapping"])
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
        self.rviz.setUp("reset", True)


class SelectModePanel(QGroupBox):
    set_operation_mode = pyqtSignal(str)

    def __init__(self, nodes_manager=None, parent=None) -> None:
        super().__init__("Seleccione modo de operacion")
        # self.setMaximumHeight(70)
        # self.setMaximumWidth(400)
        self.layout = QHBoxLayout()
        self.nodes_manager = nodes_manager
        self.parent = parent
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
                "respawn": True,
            },
        ]
        # self.name = QLabel()
        # self.name.setText("Modo de operacion")
        # self.setStyleSheet("""
        #     QGroupBox {
        #         padding: 4px;
        #     }

        # """)

        self.auto_mode_button, self.manual_mode_button, self.localize_button = (
            QPushButton("Auto"),
            QPushButton("Manual"),
            QPushButton("localize robot"),
        )
        self.auto_mode_button.setEnabled(True)
        self.manual_mode_button.setEnabled(False)
        # auto_mode_button.setEnabled(False)
        self.auto_mode_button.clicked.connect(self.handleAutoMode)
        self.manual_mode_button.clicked.connect(self.handleManualMode)
        self.localize_button.clicked.connect(self.handleLocalization)

        self.auto_mode_button.setStyleSheet(toggle_button_style)
        self.manual_mode_button.setStyleSheet(toggle_button_style)

        self.layout.addWidget(self.auto_mode_button, alignment=Qt.AlignTop)
        self.layout.addWidget(self.manual_mode_button, alignment=Qt.AlignTop)
        # self.layout.addWidget(self.localize_button, 2, 1)

        self.setLayout(self.layout)
        # self.localize_button.hide()

    def handleAutoMode(self):
        self.nodes_manager.stopNodes(["amcl", "move_base", "turtlebot3_slam_gmapping"])

        if not self.nodes_manager.topicHasPublisher("/scan"):
            toast = Toast(self.parent)
            toast.setDuration(3000)  # Hide after 5 seconds
            toast.setTitle("No hay datos del LiDar")
            toast.setText("Configure el robot")
            toast.applyPreset(ToastPreset.ERROR)  # Apply style preset
            Toast.setPositionRelativeToWidget(self.parent)
            toast.show()  # button = dlg.exec()
            return

        if not self.nodes_manager.topicHasPublisher("/map_metadata"):
            toast = Toast(self.parent)
            toast.setDuration(5000)  # Hide after 5 seconds
            toast.setTitle("No hay mapa disponible")
            toast.setText("Carga un mapa para primero")
            toast.applyPreset(ToastPreset.ERROR)  # Apply style preset
            Toast.setPositionRelativeToWidget(self.parent)
            toast.show()
            return

        toast = Toast(self.parent)
        toast.setDuration(5000)  # Hide after 5 seconds
        toast.setTitle("Modo autonomo establecido")
        toast.setText("blah blah blah")
        toast.applyPreset(ToastPreset.SUCCESS)  # Apply style preset
        Toast.setPositionRelativeToWidget(self.parent)
        toast.show()

        self.nodes_manager.stopNodes(["amcl", "move_base", "turtlebot3_slam_gmapping"])
        self.set_operation_mode.emit("auto")
        self.nodes_manager.bringUpStop()
        self.nodes_manager.bringUpStart()

        self.nodes_manager.startNodes(self.nodes_manager.initNodes(self.nodes))
        self.auto_mode_button.setEnabled(False)
        self.manual_mode_button.setEnabled(True)
        self.localize_button.setEnabled(False)

    def handleManualMode(self):
        self.nodes_manager.stopNodes(["amcl", "move_base"])
        self.nodes_manager.bringUpStop()
        self.set_operation_mode.emit("manual")
        self.auto_mode_button.setEnabled(True)
        self.manual_mode_button.setEnabled(False)
        self.localize_button.setEnabled(True)

        toast = Toast(self.parent)
        toast.setDuration(5000)  # Hide after 5 seconds
        toast.setTitle("Modo manual establecido")
        toast.setText("Carga un mapa para primero")
        toast.applyPreset(ToastPreset.SUCCESS)  # Apply style preset
        Toast.setPositionRelativeToWidget(self.parent)
        toast.show()

    def handleLocalization(self):
        if not self.nodes_manager.topicHasPublisher("/scan"):
            # dlg = QMessageBox(self)
            # dlg.setWindowTitle("I have a question!")
            # dlg.setText("No LiDar Data, Ayyyy!!! \n Anda pasha BOBO")
            toast = Toast(self.parent)
            toast.setDuration(4000)  # Hide after 5 seconds
            toast.setTitle("No hay datos del LiDar")
            toast.setText("configure el robot")
            toast.applyPreset(ToastPreset.ERROR)  # Apply style preset
            Toast.setPositionRelativeToWidget(self.parent)
            toast.show()  # button = dlg.exec()
            return

        if not self.nodes_manager.topicHasPublisher("/map_metadata"):
            # dlg = QMessageBox(self)
            # dlg.setWindowTitle("I have a question!")
            # dlg.setText("Agrega el mapa primer, Ayyyy!!! \n Anda pasha BOBO")
            # button =   dlg.exec()
            toast = Toast(self.parent)
            toast.setDuration(4000)  # Hide after 5 seconds
            toast.setTitle("No hay mapa disponible")
            toast.setText("cargue un map")
            toast.applyPreset(ToastPreset.ERROR)  # Apply style preset
            Toast.setPositionRelativeToWidget(self.parent)
            toast.show()
            return
        self.nodes_manager.bringUpStop()
        self.nodes_manager.bringUpStart()

        self.nodes_manager.stopNodes(["amcl"])
        self.nodes_manager.startNodes(self.nodes_manager.initNodes(self.nodes[:1]))

    def enable(self, object, state):
        if object == "localization":
            self.localize_button.setEnabled(state)

    # def update_operation_mode(self, mode):
    #     self.currentOperationMode = mode


class PatrolsPanel(QGroupBox):
    def __init__(self, nodes_manager=None, parent=None) -> None:
        super().__init__("Patrullajes")
        # self.setMaximumHeight(500)
        self.setMaximumWidth(400)
        self.setMinimumWidth(400)
        # self.scroll_area = QScrollArea()
        self.patrols_panel = QWidget()
        self.patrols_panel.layout = QVBoxLayout()
        self.layout = QVBoxLayout()
        self.patrols_scheduler = PatrolsEscheduler()
        self.nodes_manager = nodes_manager
        self.parent = parent
        self.timer = QTimer(self)
        # self.timer.timeout.connect(self.blink)
        self.timer.start(1000)
        self.is_blinking = False
        self.blink_state = False
        self.isStart = True

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
            QPushButton("+ Crear"),
            QPushButton("Eliminar"),
            QPushButton("Comenzar"),
            QPushButton(" Parar"),
        )
        self.control_panel_buttons = QGridLayout()
        [
            self.control_panel_buttons.addWidget(button, r, c, rx, cx)
            for button, r, c, rx, cx in [
                (self.create_btn, 0, 0, 1, -1),
                (self.delete_btn, 1, 0, 1, 1),
                (self.start_patrols_btn, 1, 1, 1, 1),
                # (self.stop_patrols_btn, 1, 1,1,1),
            ]
        ]

        self.navigation_buttons = QHBoxLayout()
        self.labels_layout = QHBoxLayout()
        max_pag, pag_index = self.patrols_container.move_page_index(0)
        self.max_pag_count_elements = self.patrols_container.max_element_view
        self.current_index_label = QLabel("0")
        self.total_pages_label = QLabel("0")

        self.left_btn, self.right_btn = QPushButton("<"), QPushButton(">")

        [
            self.navigation_buttons.addWidget(button)
            for button in (
                self.current_index_label,
                self.total_pages_label,
                self.left_btn,
                self.right_btn,
            )
        ]

        self.create_btn.clicked.connect(self.add_patrol)
        # self.start_patrols_btn.clicked.connect(self.start_patrols)
        self.start_patrols_btn.clicked.connect(self.toggleStartStop)
        self.delete_btn.clicked.connect(self.delete_patrols)
        self.left_btn.clicked.connect(self.set_previous_page)
        self.right_btn.clicked.connect(self.set_next_page)
        self.patrols_scheduler.patrol_finished.connect(self.patrol_finished)
        # self.stop_patrols_btn.clicked.connect(self.stop_any_patrol)
        self.patrols_scheduler.update_patrols_view.connect(self.get_current_patrols)
        # self.patrols_scheduler.update_patrols_view.connect(self.update_patrols_indexing_label)

        self.patrols_scheduler.update_patrols_view.connect(
            self.patrols_container.update_patrols
        )
        self.patrols_container.exec_single_patrol.connect(self.start_any_patrol)
        # self.patrols_scheduler.
        [
            button.setIcon(QApplication.style().standardIcon(icon))
            for button, icon in (
                (self.start_patrols_btn, QStyle.SP_MediaPlay),
                (self.delete_btn, QStyle.SP_DialogDiscardButton),
            )
        ]

        # icon_start = QApplication.style().standardIcon(QStyle.SP_MediaPlay)
        # self.start_patrols_btn.setIcon(icon_start)

        icon_start = QApplication.style().standardIcon(QStyle.SP_CommandLink)
        # self.left_btn.setIcon(icon_start)

        icon = QApplication.style().standardIcon(QStyle.SP_ArrowBack)
        pixmap = icon.pixmap(64, 64)
        transform = QTransform().rotate(180)
        rotated_pixmap = pixmap.transformed(transform, Qt.SmoothTransformation)
        # self.right_btn.setIcon(QIcon(rotated_pixmap))

        self.delete_btn.setStyleSheet(border_button_style_danger)
        self.create_btn.setStyleSheet(colored_button_style)
        self.start_patrols_btn.setStyleSheet(border_button_style)
        self.stop_patrols_btn.setStyleSheet(border_button_style)
        self.right_btn.setStyleSheet(minimal_button_style)
        self.left_btn.setStyleSheet(minimal_button_style)
        self.current_index_label.setStyleSheet(minimal_label_style)
        self.total_pages_label.setStyleSheet(minimal_label_style)
        self.current_index_label.setMaximumWidth(30)
        self.total_pages_label.setMaximumWidth(30)

        # pixmapi =
        # icon = self.style().standardIcon(QStyle.SP_ComputerIcon)
        # self.menu_button.setIcon(icon)

        self.layout.addLayout(self.control_panel_buttons)
        self.layout.addLayout(self.navigation_buttons)
        self.layout.addWidget(self.patrols_container)

        self.setLayout(self.layout)

    def blink(self):
        """Toggle between two colors to create blink effect"""
        if self.blink_state:
            self.start_patrols_btn.setStyleSheet("background-color: red; color: white;")
        else:
            self.start_patrols_btn.setStyleSheet(
                "background-color: blue; color: white;"
            )

        self.blink_state = not self.blink_state

    def get_current_patrols(self, x):
        n = len(x.keys())
        self.total_pages_label.setText(
            f"{int(n/self.max_pag_count_elements) + int(n%self.max_pag_count_elements)}"
        )

    def add_patrol(self, check):
        self.parent.popup = PatrolsMenu(
            self.parent, selected_days=[], patrolid=datetime.now().timestamp()
        )
        self.parent.popup.update_date.connect(self.patrols_scheduler.update_patrol)
        self.parent.popup.update_date.connect(self.alert)
        self.parent.popup.show_popup()
        # max_pag, pag_index = self.patrols_container.move_page_index(0)
        # self.patrol_indexing.setText(f'{pag_index}/{max_pag}')
        # pass

    def alert(self, x):
        if self.patrols_scheduler.patrolIsRunning:
            dlg = QMessageBox(self)
            dlg.setWindowTitle("Importante")
            # dlg.setWindowFlags(Qt.FramelessWindowHint)
            dlg.setText(
                "INFO: Se agrego un patrullaje mientras el la ejecucion de patrullajes esta corriendo \nparata que tus cambios surtan efecto deten e inicia de nuevo los patrullajes"
            )
            button = dlg.exec()
            dlg.setStyleSheet("""
                QDialog {
                    background-color: #333;
                    border-radius: 4px;
                    color: white;
                }
                QLabel {
                    font-size: 12px;
                    padding: 8px;
                    color: white;
                }
            """)

    def toggleStartStop(self):
        if self.isStart:
            self.start_patrols()
            return
        self.stop_any_patrol()

    def start_patrols(self):
        if not self.nodes_manager.nodeIsRunning("amcl"):
            toast = Toast(self.parent)
            toast.setDuration(5000)  # Hide after 5 seconds
            toast.setTitle("Error: Fallo inicio de patrullajes")
            toast.setText("El node de amcl no esta activo")
            toast.applyPreset(ToastPreset.ERROR)  # Apply style preset
            Toast.setPositionRelativeToWidget(self.parent)
            toast.show()
            return

        if not self.nodes_manager.nodeIsRunning("move_base"):
            toast = Toast(self.parent)
            toast.setDuration(5000)  # Hide after 5 seconds
            toast.setTitle("Error: Fallo inicio de patrullajes")
            toast.setText("El node de move_base no esta activo")
            toast.applyPreset(ToastPreset.ERROR)  # Apply style preset
            Toast.setPositionRelativeToWidget(self.parent)
            toast.show()
            return

        if len(self.patrols_scheduler._points_to_visit) == 0:
            toast = Toast(self.parent)
            toast.setDuration(5000)  # Hide after 5 seconds
            toast.setTitle("Error: Fallo inicio de patrullajes")
            toast.setText("No hay puntos de interes")
            toast.applyPreset(ToastPreset.ERROR)  # Apply style preset
            Toast.setPositionRelativeToWidget(self.parent)
            toast.show()
            return

        if len(self.patrols_scheduler.patrols_data) == 0:
            toast = Toast(self.parent)
            toast.setDuration(5000)  # Hide after 5 seconds
            toast.setTitle("Error: Fallo inicio de patrullajes")
            toast.setText("No hay patrullajes en la lista")
            toast.applyPreset(ToastPreset.ERROR)  # Apply style preset
            Toast.setPositionRelativeToWidget(self.parent)
            toast.show()
            return

        if self.patrols_scheduler.patrolIsRunning:
            toast = Toast(self.parent)
            toast.setDuration(5000)  # Hide after 5 seconds
            toast.setTitle("INFO: Ya hay un patrullaje corriendo")
            toast.setText("Para iniciar un  nuevo patrullaje, detenga el actual")
            toast.applyPreset(ToastPreset.INFORMATION)  # Apply style preset
            Toast.setPositionRelativeToWidget(self.parent)
            toast.show()
            return


        self.start_any_patrol()
        # self.start_patrols_btn.setEnabled(False)
        # self.create_btn.setEnabled(False)
        # self.delete_btn.setEnabled(False)
        # self.stop_patrols_btn.setEnabled(True)

        toast = Toast(self.parent)
        toast.setDuration(3000)  # Hide after 5 seconds
        toast.setTitle("Programacion Exitosa`")
        toast.setText("NO los patrullajes an sido programados")
        toast.applyPreset(ToastPreset.SUCCESS)  # Apply style preset
        Toast.setPositionRelativeToWidget(self.parent)
        toast.show()

    def start_any_patrol(self, patrolid=None):
        icon_stop = QApplication.style().standardIcon(QStyle.SP_MediaStop)
        self.start_patrols_btn.setIcon(icon_stop)
        self.start_patrols_btn.setText("Parar")
        self.isStart = False

        self.patrols_scheduler.start_patrols()
        pass

    def stop_any_patrol(self, c=None):
        # self.patrols_container
        # self.start_patrols_btn.setEnabled(True)
        # self.create_btn.setEnabled(True)
        # self.delete_btn.setEnabled(True)
        # self.stop_patrols_btn.setEnabled(False)

        if not self.patrols_scheduler.cancel_task():
            icon_start = QApplication.style().standardIcon(QStyle.SP_MediaPlay)
            self.start_patrols_btn.setIcon(icon_start)
            self.start_patrols_btn.setText('Comenzar')
            self.isStart = True
            toast = Toast(self.parent)
            toast.setDuration(4000)  # Hide after 5 seconds
            toast.setTitle("No hay patrollajes corriendo")
            toast.setText("Reinicie los patrullajes")
            toast.applyPreset(ToastPreset.INFORMATION)  # Apply style preset
            Toast.setPositionRelativeToWidget(self.parent)
            toast.show()
            return

        # icon_start = QApplication.style().standardIcon(QStyle.SP_MediaPlay)
        # self.start_patrols_btn.setIcon(icon_start)
        # self.start_patrols_btn.setText("Comenzar")
        # self.isStart = True
        # print("PATROL STOPPED")



    def delete_patrols(self):
        toDelete = self.patrols_container.get_selected_patrolsid()
        self.patrols_scheduler.delete_patrols(toDelete)
        if self.patrols_scheduler.patrolIsRunning:
            dlg = QMessageBox(self)
            dlg.setWindowTitle("importante")
            dlg.setText(
                "INFO: Se borro un patrullaje mientras el la ejecucion de patrullajes esta corriendo\nparata que tus cambios surtan efecto deten e inicia de nuevo los patrullajes"
            )
            button = dlg.exec()

        # max_pag, pag_index = self.patrols_container.move_page_index(0)
        # self.patrol_indexing.setText(f'{pag_index}/{max_pag}')

    def handleSinglepatrolExec(self, x=None):
        # self.start_patrols_btn.setEnabled(False)
        # self.create_btn.setEnabled(False)
        # self.delete_btn.setEnabled(False)
        pass

    def patrol_finished(self, message):
        # self.start_patrols_btn.setEnabled(False)
        # self.create_btn.setEnabled(False)
        # self.delete_btn.setEnabled(False)
        if message == 'TaskSuccessfully' :
            toast = Toast(self.parent)
            toast.setDuration(3000)  # Hide after 5 seconds
            toast.setTitle("Todos los Patrullajes se Ejecutado con Exito")
            toast.setText("Cree nuevos patrullajes y de Iniciar")
            toast.applyPreset(ToastPreset.SUCCESS)  # Apply style preset
            Toast.setPositionRelativeToWidget(self.parent)
            toast.show()

        if message == 'TaskCancelled':
            toast = Toast(self.parent)
            toast.setDuration(3000)  # Hide after 5 seconds
            toast.setTitle("Programacion cancelada`")
            toast.setText("La programacion ha sido cancelada")
            toast.applyPreset(ToastPreset.WARNING)  # Apply style preset
            Toast.setPositionRelativeToWidget(self.parent)
            toast.show()

        icon_start = QApplication.style().standardIcon(QStyle.SP_MediaPlay)
        self.start_patrols_btn.setIcon(icon_start)
        self.start_patrols_btn.setText("Comenzar")
        self.isStart = True


    def set_next_page(self):
        max_pag, pag_index = self.patrols_container.move_page_index(1)
        self.current_index_label.setText(f"{1 + pag_index}")
        # print('nex page fuc', a)
        self.patrols_scheduler.get_current_patrols()

    def set_previous_page(self):
        max_pag, pag_index = self.patrols_container.move_page_index(-1)
        self.current_index_label.setText(f"{1 + pag_index}")
        self.patrols_scheduler.get_current_patrols()

    def update_patrols_indexing_label(self, patrols):
        max_pag_count, pag_index = self.patrols_container.move_page_index(0)
        if max_pag_count == 0:
            self.current_index_label.setText(f"{1}/{1}")
            return
        self.current_index_label.setText(
            f"{pag_index}/{len(patrols)/max_pag_count + len(patrols)%max_pag_count}"
        )

        # self.patrol_indexing.setText(f'{pag_index}/{max_pag}')
        pass

    def update_points(self, points):
        self.patrols_scheduler.setPointsToVisit(points)
        # print("from patrolpanel "ToDelete, points)
        pass


class PatrolsContainer(QWidget):
    exec_single_patrol = pyqtSignal(str)
    def __init__(
        self, patrols_data: dict, parent, patrols_scheduler, callback=None
    ) -> None:
        super().__init__()
        self.layout = QVBoxLayout()
        self.layout.setAlignment(Qt.AlignTop)
        self.patrols_data = patrols_data
        self.parent = parent
        # self.layout.setSpacing(0)
        self.patrols_widgets = []
        self.patrols_scheduler = patrols_scheduler
        self.pageindex = 0
        self.max_element_view = 2
        self.element_count = 0
        self.isSinglePatrolExec = False
        self.callback = callback
        self.running_patrol = None
        self.layout.setSpacing(2)

        self.update_patrols(patrols_data)
        self.setLayout(self.layout)
        self.patrols_scheduler.patrol_state.connect(self.update_running_patrol)
        self.patrols_scheduler.single_patrol_active.connect(self.handleSinglePatrolActive)

    def update_patrols(self, patrols_data):
        [w.setParent(None) for w in self.patrols_widgets]
        [self.layout.removeWidget(w) for w in self.patrols_widgets]
        self.patrols_widgets = []
        self.element_count = len(patrols_data)
        patrols_items = list(patrols_data.items())
        patrols_items.reverse()
        i = 0

        for id, patrol in patrols_items[
            self.pageindex * self.max_element_view : self.max_element_view
            + self.pageindex * self.max_element_view
        ]:
            print(patrol.keys())
            days = list(patrol["days"].keys())
            time = patrol["time"]
            state = patrol.get("state")
            patrol_widget = Patrol(
                parent=self.parent,
                id=id,
                days=days,
                time=time,
                patrols_scheduler=self.patrols_scheduler,
                state=state,
            )
            i = i + 1
            patrol_widget.enable_single_patrol_exec.connect(self.handleSinglepatrolExec)
            self.patrols_widgets.append(patrol_widget)

            if str(self.running_patrol) == str(id):
                self.patrols_scheduler.points_scheduler.patrol_progress.connect(
                    patrol_widget.update_patrol_progress
                )


        print("hola soy patrolcontainer", len(self.patrols_widgets), patrols_data)
        [self.layout.addWidget(patrol) for patrol in self.patrols_widgets]
        self.update()

    def update_running_patrol(self, id):
        print("PATROL RUNNNing ", id)
        self.running_patrol = id
        for patrol in self.patrols_widgets:
            if str(patrol.id) == str(id):
                self.patrols_scheduler.points_scheduler.patrol_progress.connect(
                    patrol.update_patrol_progress
                )
                self.patrols_scheduler.points_scheduler.patrol_progress.connect(
                    self.update_patrol_model
                )
            # else:
            #     # button.clicked.disconnect(self.on_button_clicked)
            #     self.patrols_scheduler.points_scheduler.patrol_progress.disconnect(
            #         patrol.update_patrol_progress
            #     )

    def update_patrol_model(self, id, patrols_left, total_patrols):
        print("PATROLD UPDATE", id, self.patrols_scheduler.patrols_data.get(id))
        if self.patrols_scheduler.patrols_data.get(id):
            self.patrols_scheduler.patrols_data.get(id)["state"] = (
                patrols_left,
                total_patrols,
            )

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

        max_pages_count = (
            self.element_count // self.max_element_view
            + self.element_count % self.max_element_view
        )

        return (max_pages_count, self.pageindex)

    def get_selected_patrolsid(self):
        return [patrol.selected for patrol in self.patrols_widgets if patrol.selected]

    def handleSinglepatrolExec(self, patrolid):
        self.patrols_scheduler.start_single_patrol_scheduling(patrolid)
        for patrol in self.patrols_widgets:
            if str(patrol.id) == str(patrolid):
                self.patrols_scheduler.points_scheduler.patrol_progress.connect(
                    patrol.update_patrol_progress
                )

    def handleSinglePatrolActive(self, patrolid=None):
        self.exec_single_patrol.emit(str(patrolid))


    def patrol_finished(self):
        pass

    def add_patrol(self, patrol):
        pass


class Patrol(QGroupBox):
    enable_single_patrol_exec = pyqtSignal(float)

    def __init__(
        self,
        parent,
        id: str = "999",
        days:List[str] =[],
        time:str ='2323',
        patrols_scheduler=None,
        state=None,
        delay = 0
    ):
        super().__init__()
        self.days = days
        self.delay = delay
        self.id = id
        self.patrols_scheduler = patrols_scheduler
        self.time = time
        self.state = state
        # self.setStyleSheet("border-color: gray")
        self.parent = parent
        self.progress = QProgressBar()
        self.points_checked_label = QLabel()
        # self.points_checked_label.setText("activo 1/34")
        self.progress.setTextVisible(False)
        # self.progress.setValue(10)
        self.progress.hide()
        self.points_checked_label.hide()
        self.progress.setMaximumHeight(6)
        self.progress.setEnabled(False)
        self.checked = False
        # self.setObjectName("mainWidget")
        self.setStyleSheet(patrol_base_style)

        if state:
            self.progress.show()
            self.points_checked_label.show()
            self.points_checked_label.setText(f"{state[0]}/{state[1]}")
            self.progress.setValue(100 * (state[1] - state[0]) / state[1])

        self.layout = QGridLayout()
        self.patrol_name_label = QLabel()
        self.patrol_name_label.setText(" ".join(days))
        self.patrol_time_label = QLabel(self.time)

        ( self.menu_button, self.checkbox) = (
            QPushButton("Menu"),
            QCheckBox(),
        )

        self.menu_button.clicked.connect(self.edit_patrol_task)
        self.menu = QMenu()
        edit = self.menu.addAction("Editar /")
        edit.triggered.connect(self.edit_patrol)

        force_exec = self.menu.addAction("Ejecutar ⤭")
        force_exec.triggered.connect(self.force_execution)
        # force_exec.setEnabled(False)
        self.menu_button.setMenu(self.menu)

        pixmapi = QStyle.SP_MessageBoxInformation
        icon = self.style().standardIcon(pixmapi)
        self.menu_button.setIcon(icon)
        self.menu_button.setStyleSheet(tertiary_button_style)
        self.checkbox.setStyleSheet(patrol_checkbox_style)
        # self.menu_button.setMaximumWidth(30)
        self.patrol_name_label.setToolTip("This is a tooltip!")
        self.patrol_time_label.setToolTip("This is a tooltip!")

        self.checkbox.stateChanged.connect(self.select)

        self.layout.addWidget(self.patrol_time_label, 1, 1, alignment=Qt.AlignLeft)
        self.layout.addWidget(self.patrol_name_label, 1, 3)
        self.layout.addWidget(self.menu_button, 1, 4, alignment=Qt.AlignRight)
        self.layout.addWidget(self.progress, 2, 0, 1, 4)
        self.layout.addWidget(self.points_checked_label, 2, 4, 1, 1)
        self.layout.addWidget(self.checkbox, 1, 0, alignment=Qt.AlignLeft)
        self.setLayout(self.layout)

        # self.animate_show()

    def edit_patrol_task(self):
        self.popup = PatrolsMenu(
            self.parent,
            selected_days=self.days,
            patrolid=self.id,
            time=self.time,
            state=self.state,
        )
        self.popup.update_date.connect(self.patrols_scheduler.update_patrol)
        self.popup.show_popup()

    def edit_patrol(self):
        # print("Option 1 selected")
        self.edit_patrol_task()

    def force_execution(self):
        self.enable_single_patrol_exec.emit(self.id)
        # self.exec_botton.setEnabled(False)
        print("Option 2 selected")

    def select(self):
        self.checked = self.checkbox.isChecked()
        if self.checked:
            self.setStyleSheet(patrol_selected_style)
        else:
            self.setStyleSheet(patrol_base_style)

    # @pyqtSlot(str, int, int)
    def update_patrol_progress(self, id, patrols_left, total_patrols):
        if str(id) == str(self.id):
            print(
                "Hola soy un Patrol estoy RUNNUing",
                id,
                self.id,
                int(100 * patrols_left / total_patrols),
            )
            self.progress.show()
            self.points_checked_label.show()
            self.points_checked_label.setText(f"activo {patrols_left}/{total_patrols}")
            self.progress.setValue(
                int(100 * (total_patrols - patrols_left) / total_patrols)
            )
            if patrols_left == 0:
                self.patrols_scheduler.points_scheduler.patrol_progress.disconnect(
                    self.update_patrol_progress
                )

    @property
    def selected(self):
        if self.checked:
            return self.id
        else:
            return None

    def animate_show(self):
        anim = QPropertyAnimation(self, b"maximumWidth")
        anim.setDuration(200 + 100*self.delay)
        anim.setStartValue(300)
        anim.setEndValue(500)
        anim.setEasingCurve(QEasingCurve.InOutExpo)

        self.setGraphicsEffect(None)  # Clear any previous effects
        opacity_effect = self.graphicsEffect()

        opacity_anim = QPropertyAnimation(self, b"opacity")
        opacity_anim.setDuration(200)
        opacity_anim.setStartValue(0)
        opacity_anim.setEndValue(1)
        opacity_anim.setEasingCurve(QEasingCurve.InOutQuad)
        # Run animations in parallel
        group = QParallelAnimationGroup(self)
        group.addAnimation(anim)
        group.addAnimation(opacity_anim)
        group.start()

class BatteryIndicator(QWidget):
    def __init__(self):
        super().__init__()
        # Create UI elements
        self.layout = QHBoxLayout()
        
        # Battery percentage with dynamic colorSP_DialogCloseButton
        text = QLabel('♥ Estado de la bateria del robot: ')
        text.setStyleSheet(muted_label_style)
        self.percentage_label = QLabel("100%")
        self.percentage_label.setAlignment(Qt.AlignLeft)
        self.percentage_label.setStyleSheet("""
            font-size: 12px;
            font-weight: bold;
            color: #4CAF50;  /* Initial green color */
        """)
        
        self.status_label = QLabel("Cargada")
        self.status_label.setAlignment(Qt.AlignLeft)
        self.status_label.setStyleSheet("font-size: 14px; color: #666;")
        
        self.charging_icon = QLabel("⚡ CHARGING")
        # self.charging_icon.setAlignment(Qt.AlignCenter)
        self.charging_icon.setStyleSheet("""
            font-size: 14px;
            color: #FFC107;
            font-weight: bold;
        """)

        self.layout.addWidget(text, alignment=Qt.AlignLeft)
        self.layout.addWidget(self.percentage_label, alignment=Qt.AlignLeft)
        self.layout.addWidget(self.status_label, alignment=Qt.AlignRight)
        # self.layout.addWidget(self.charging_icon, alignment=Qt.AlignLeft)
        self.setLayout(self.layout)
        
        self.current_level = 100
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_battery)
        self.timer.start(1000)  # Update every second
    
    def update_battery(self):
        if self.current_level > 0:
            self.current_level -= 2  # Drain faster for demo
        else:
            self.current_level += 5  # Charge faster for demo
            if self.current_level >= 100:
                self.current_level = 100
        
        if self.current_level > 70:
            battery_color = "#4CAF50"  # Green
            text_color = "#4CAF50"     # Green
            status = "Buena"
        elif self.current_level > 30:
            battery_color = "#FFC107"   # Amber
            text_color = "#FF9800"     # Darker amber
            status = "Baja"
        else:
            battery_color = "#F44336"   # Red
            text_color = "#D32F2F"     # Darker red
            status = "Muy Baja"
            
        if self.current_level >= 95:
            status = "Cargada"
        elif self.charging_icon.isVisible():
            status = "Charging"
        
        self.percentage_label.setStyleSheet(f"""
            font-size: 13px;
            font-weight: bold;
            color: {text_color};
        """)
        self.percentage_label.setText(f"{self.current_level}%")
        self.status_label.setText(status)
    


if __name__ == "__main__":
    app = QApplication(sys.argv)
    t = HomePanel()
    # t = PatrolsPanel()
    # t = VisualizationPanel()
    t.show()

    sys.exit(app.exec())
