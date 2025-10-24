import sys
import re
import rospy
from typing import List
from enum import Enum

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
    QActionGroup,
    QAction,
    QFileDialog,
    QMessageBox,
    QStyle,
    QGraphicsOpacityEffect,
    QToolTip,
    QStackedLayout,
)
from PyQt5.QtCore import (
    QPoint,
    Qt,
    pyqtSlot,
    pyqtSignal,
    QPropertyAnimation,
    QParallelAnimationGroup,
    QEasingCurve,
    QTimer,
    QTime,
    QSize,
    QEvent,
)
from PyQt5.QtGui import QFont, QIcon, QPixmap, QTransform, QFontMetrics

from database_manager import AlertStatus
from pyqttoast import Toast, ToastPreset, ToastPosition
from notification import Notification, NotificationType

from datetime import datetime
import rospy
from styles.buttons import (
    border_button_style,
    border_button_style_danger,
    menu_style,
    primary_button_style,
    secondary_button_style,
    colored_button_style,
    tertiary_button_style,
    toggle_button_style,
    minimal_button_style,
    patrol_checkbox_style,
    button_with_menu_style
)

from styles.labels import (
    inactive_label_style,
    minimal_label_style,
    muted_label_style,
    muted_mini_label_style,
    warning_label_style,
    info_label_style,
)

from styles.patrols import patrol_base_style, patrol_selected_style

from sensor_msgs.msg import BatteryState 

# from points_manager import PointsGenerator
from better_image_display import ImageViewer
from joystick import Joypad
from patrols_scheduler import PatrolsEscheduler
from patrol_menu import PatrolsMenu
from rview import MyViz
from input_textdialog import InputDialog, CustomDialog
from robot_camera_view import RobotCamera
from image_carousel import ImageCarousel
from robot_actions_logger import RobotActionsLoggerView 
from place_form import PlaceForm
import  robot_actions_logger

from utils.patrol import PatrolEndState, userOperation, operationMode


class HomePanel(QWidget):
    def __init__(self, nodes_manager=None, parent=None):
        super().__init__()
        # self.resize(900, 900)
        self.layout = QGridLayout()
        self.currentUserOperation = userOperation.IDLE
        # self.showMaximized()
        # self.layout = QHBoxLayout()
        # [self.layout.addWidget(element) for element in (VisualizationPanel(), PatrolsPanel())]
        # print("home panel", parent)

        self.visualization_panel, self.patrol_panel, joypad = (
            VisualizationPanel(
                nodes_manager=nodes_manager, parent=parent, global_state_holder=self
            ),
            PatrolsPanel(parent=parent, nodes_manager=nodes_manager),
            # pannel(self),
            Joypad(),
            # PointsGenerator()
            # ImageViewer()
        )
        select_mode_panel = SelectModePanel(
            nodes_manager=nodes_manager,
            parent=parent,
            patrol_panel=self.patrol_panel,
            global_state_holder=self,
        )

        # joypad.setEnabled(True)
        select_mode_panel.set_operation_mode.connect(joypad.update_operation_mode)
        select_mode_panel.set_operation_mode.connect(
            self.visualization_panel.update_operation_mode
        )
        self.visualization_panel.update_points.connect(self.patrol_panel.update_points)
        self.visualization_panel.enable.connect(select_mode_panel.enable)
        self.visualization_panel.save_in_database.connect(
            self.patrol_panel.patrols_container.handleSavePointsInDatabase
        )
        self.visualization_panel.map_loaded.connect(
            self.patrol_panel.patrols_container.patrols_scheduler.send_points_data
        )
        self.patrol_panel.patrols_container.patrols_scheduler.set_stored_database_points.connect(
            self.visualization_panel.parent.pointsWindow.load_stored_points
        )

        self.patrol_panel.patrols_container.patrols_scheduler.points_scheduler.points_state.connect(
            self.visualization_panel.parent.pointsWindow.update_points_state
        )
        self.patrol_panel.patrols_container.patrols_scheduler.set_running_patrol.connect(
            self.visualization_panel.parent.pointsWindow.reset_points_state
        )
        select_mode_panel.cancel_user_operation.connect(
            self.visualization_panel.setMapOperationState
        )
        # setCreatMapState

        self.visualization_panel.map_loaded.connect(select_mode_panel.checkMap)
        self.visualization_panel.map_loaded.connect(
            self.patrol_panel.patrols_scheduler.points_scheduler.setMap
        )
        self.patrol_panel.patrols_scheduler.points_scheduler.alert_generated.connect(self.visualization_panel.handleAlertGeneration)


        self.patrol_panel.patrols_container.patrols_scheduler.points_scheduler.points_state.connect(
            self.visualization_panel.drainage_checkpoints_win.update_dreinage_info
        )

        self.patrol_panel.patrols_container.patrols_scheduler.set_stored_database_points.connect(
            self.visualization_panel.drainage_checkpoints_win.load_stored_points
        )

        self.visualization_panel.change_mode.connect(select_mode_panel.handleChangeMode)


        self.patrol_panel.patrols_container.patrols_scheduler.set_running_patrol.connect(
            self.visualization_panel.drainage_checkpoints_win.reset_dreinage_status
        )


        self.layout.addWidget(self.visualization_panel, 0, 0, 8, 1)
        self.layout.addWidget(select_mode_panel, 0, 2, 1, 1)

        self.layout.addWidget(self.patrol_panel, 1, 2, 6, 1)


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
    save_in_database = pyqtSignal(dict)
    enable = pyqtSignal(str, bool)
    map_saved = pyqtSignal(str)
    selected_user_operation = pyqtSignal(userOperation)
    change_mode = pyqtSignal(operationMode)

    def __init__(
        self, nodes_manager=None, parent=None, global_state_holder=None
    ) -> None:
        super().__init__()
        self.layout = QGridLayout()
        self.buttons_layout = QGridLayout()
        self.rviz_options_layout = QHBoxLayout()

        self.stacklayout = QStackedLayout()
        self.stacklayout.setStackingMode(QStackedLayout.StackingMode.StackAll)
        self.stacked_widgets_container = QWidget()
        self.stacked_widgets_container.setLayout(self.stacklayout)

        self.parent = parent
        self.nodes_manager = nodes_manager
        self.isCreateMap = True
        self.global_state_holder = global_state_holder
        self.mapAsPrincipalView = True
        self.buffer_data_robot_camera = []
        self.view_options = {}

        self.message = QLabel('MENSAJE: Mientras Calibracinon activa, los patrullajes estaran desactivas ')
        self.message.hide()

        self.rviz = MyViz(configfile="./config/config_navigation.rviz")

        self.robotcamera = RobotCamera(
            buffer=self.buffer_data_robot_camera, parent=self.parent
        )



        self.robot_actions_logger = RobotActionsLoggerView()
        self.logger = robot_actions_logger.logger

        self.drainage_checkpoints_win = ImageCarousel(
            buffer=self.buffer_data_robot_camera
        )

        # self.parent.pointWindow = None
        self.currentOperationMode = operationMode.MANUAL
        # self.global_state_holder.currentUserOperation = userOperation.IDLE
        # self.mini_display = RobotCamera()
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
        ) = (
            QPushButton("Guardar mi mapita"),
            QPushButton("Cargar mapa"),
            QPushButton("puntos de interes"),
            QPushButton("Crear mapa"),
        )

        self.view_menu_btn = QPushButton("Opciones")
        
        [
            self.buttons_layout.addWidget(button, r, c)
            for button, r, c, rx, cx in (
                (self.view_menu_btn, 0, 1, 0, 0),
                (self.load_map_button, 0, 0, 1, 4),
                (self.points_window_btn, 0, 2, 1, 3),
                (self.create_map_btn, 0, 3, 1, 3),
            )
        ]



        # Create the QMenu
        view_menu = QMenu(self)
        view_options_action_group = QActionGroup(self) 

        # Add actions to the menu 
        action1 = view_menu.addAction( "Modo de pantalla: Camara")
        action2 = view_menu.addAction( "Modo de pantalla: Mapa")

        action1.setCheckable(True)
        action2.setCheckable(True)

        view_options_action_group.addAction(action1)
        view_options_action_group.addAction(action2)

        view_menu.addSeparator() # Add a separator line
        self.action3 = view_menu.addAction("Enfocar vista al robot")
        self.action3.setWhatsThis('No se que poner')
        self.action3.setCheckable(True)

        view_menu.addSeparator() # Add a separator line
        self.action4 = view_menu.addAction("Marcar lugar como referencia")
        self.action4.setEnabled(False)

        # Connect actions to methods
        action1.triggered.connect(lambda: self.toggleCameraMapView())
        action2.triggered.connect(lambda: self.toggleCameraMapView())
        self.action3.triggered.connect(lambda: self.handleActionSelected(self.action3))
        self.action4.triggered.connect(lambda: self.robotcamera.buffer_reference_image())

        # Set the menu to the button
        self.view_menu_btn.setMenu(view_menu)

        # self.view_menu_btn.setStyleSheet(border_button_style + button_with_menu_style)
        self.message.setStyleSheet(warning_label_style)
        view_menu.setStyleSheet(menu_style)
        self.view_menu_btn.setStyleSheet(secondary_button_style)
        self.load_map_button.setStyleSheet(secondary_button_style + button_with_menu_style)
        self.points_window_btn.setStyleSheet(secondary_button_style + button_with_menu_style)
        self.create_map_btn.setStyleSheet(border_button_style)

        icon_start = QApplication.style().standardIcon(QStyle.SP_MediaPlay)
        self.create_map_btn.setIcon(icon_start)
        # self.create_map_btn.setText('ddd')

        icon_load = QApplication.style().standardIcon(QStyle.SP_FileLinkIcon)
        self.load_map_button.setIcon(icon_load)


        self.load_map_button.clicked.connect(self.handleLoadMap)
        self.map_loaded.connect(self.parent.pointsWindow.load_map)
        self.create_map_btn.clicked.connect(self.toggleCreaeteSaveMap)

        menu = QMenu("checkpoints", self)
        menu.setStyleSheet(menu_style)
        show_checkpoints_action = QAction("Puntos de interes", self)
        show_dreinage_action = QAction("Fotos de referencia", self)

        show_checkpoints_action.triggered.connect(self.show_points_window)
        menu.addAction(show_checkpoints_action)
        show_dreinage_action.triggered.connect(self.show_dreinage_window)
        menu.addAction(show_dreinage_action)
        self.points_window_btn.setMenu(menu)
        # self.save_button.setEnabled(False)
        #

        self.logger.log_changed.connect(self.robot_actions_logger.update_log)
        self.robot_actions_logger.log_file_updated.connect(self.logger.update_log_file)

        # self.points_window_btn.clicked.connect(self.show_points_window)
        self.parent.pointsWindow.save_selected_points.connect(self.handleSavePoints)
        self.parent.pointsWindow.save_in_database.connect(self.handleSaveInDatabase)
        self.map_saved.connect(self.robotcamera.save_buffered_data)
        self.robotcamera.send_buffered_data.connect(
            self.drainage_checkpoints_win.load_images
        )
        # self.robotcamera.custom_option_clicked.connect(self.toggleCameraMapView)
        self.selected_user_operation.connect(self.drainage_checkpoints_win.get_user_operation)


        self.stacklayout.addWidget(self.rviz)
        self.stacklayout.addWidget(self.robotcamera)
        # self.setView(0)
        # self.rviz.hide()
        # self.toggleCameraMapView()
        self.battery_state = BatteryIndicator()

        

        # self.layout.addLayout(self.rviz_options_layout, 0, 0)
        self.layout.addLayout(self.buttons_layout, 1, 0)
        self.layout.addWidget(self.message, 0, 0, 1, -1)
        self.layout.addWidget(self.battery_state, 1, 3)
        self.layout.addWidget(self.stacked_widgets_container, 2, 0, 1, 4)
        self.layout.addWidget(self.robot_actions_logger, 3, 0, 1, 4)
        self.setLayout(self.layout)

        self.layout.setAlignment(self.battery_state, Qt.AlignLeft)

    def enable_components(self, enable):
        if not enable:
            self.message.show()
            return

        self.message.hide()

    def handleActionSelected(self, action3):
        self.setFollowRobot(action3.isChecked())
        action3.setText('Desenfocar vista del robot' if action3.isChecked() else 'Enfocar vista al robot')
        print(action3.isChecked())


    def show_log(self):
        self.stacklayout.setCurrentIndex(2)

    def handleAlertGeneration(self, message, status):

        if status == AlertStatus.ERROR:
            ntf = Notification(title='Ha ocurrido una alerta', msg=f"{message}", preset=ToastPreset.ERROR, parent=self.parent)
        if status == AlertStatus.WARNING:
            ntf = Notification(title='Ha ocurrido una alerta', msg=f"{message}", preset=ToastPreset.WARNING, parent=self.parent)
        if status == AlertStatus.INFO:
            ntf = Notification(title='Ha ocurrido una alerta', msg=f"{message}", preset=ToastPreset.INFO, parent=self.parent)

        ntf.show()

    def toggleCameraMapView(self):
        if self.mapAsPrincipalView:
            self.stacklayout.setCurrentIndex(1)
            self.robotcamera.raise_()
            print(self.stacklayout.currentWidget())
            self.robotcamera.update()
            self.mapAsPrincipalView = False
            self.robotcamera.setMaximumSize(250, 190)
            self.robotcamera.move(50, 50)
            self.rviz.setMaximumSize(2000, 1000)
            return

        self.stacklayout.setCurrentIndex(0)
        self.rviz.raise_()
        print(self.stacklayout.currentWidget())
        self.rviz.update()
        self.mapAsPrincipalView = True
        self.rviz.setMaximumSize(250, 250)
        self.rviz.move(50, 50)
        self.robotcamera.setMaximumSize(2000, 1000)

    def setView(self, index):
        i = self.stacklayout.currentIndex()
        self.stacklayout.setCurrentIndex(index)

        self.stacklayout.currentWidget().setMaximumSize(250, 250)
        self.stacklayout.currentWidget().update()

        if index == 0:
            index = 1
        if index == 1:
            index = 0

        self.stacklayout.widget(index).setMaximumSize(2000, 1000)
        self.stacklayout.widget(index).update()
        

    # def resizeEvent(self, event):
    def paintEvent(self, event):
        currentWidget = self.stacklayout.currentWidget()
        currentWidget.raise_()
        currentWidget.update()
        super().paintEvent(event)

    def toggleCreaeteSaveMap(self):
        if self.isCreateMap:
            self.handleCreateMap()
        else:
            self.saveMapHandler()

    def setFollowRobot(self, x):
        self.rviz.setUp("followrobot", x)
        # if not x:
        #     self.rviz.setUp("scale", 60)
        #     self.rviz.setUp("globalframe", "map")

    def handleSaveInDatabase(self, data):
        self.save_in_database.emit(data)

    def handleLoadMap(self):
        if self.currentOperationMode == operationMode.AUTO:
            dg = CustomDialog(
                self.parent,
                "No se puede cargar mapa en el modo Auto",
                message='Para cargar un nuevo mapa cambie al modo manual',
                positive_response="Seguir en modo Auto",
                negative_response="Cambiar a Manual",
                retries=1,
            )
            dg.exec_()
            if not dg.response == "Positive":
                self.change_mode.emit(operationMode.MANUAL)
                return

        if self.global_state_holder.currentUserOperation == userOperation.CREATEMAP:
            dg = CustomDialog(
                self.parent,
                "Creacion de mapa sigue activo",
                message='perderá todo el progreso que tiene hasta ahora!',
                positive_response="Seguir creando",
                negative_response="Descartar",
                retries=1,
            )
            dg.exec_()
            if dg.response == "Positive":
                return

            # self.saveMapHandler()
            # self.nodes_manager.save_map()
            icon = QApplication.style().standardIcon(QStyle.SP_MediaPlay)
            self.create_map_btn.setIcon(icon)
            self.create_map_btn.setText("Crear Mapa")
            self.isCreateMap = True

        self.nodes_manager.stopNodes(["turtlebot3_slam_gmapping"])
        self.rviz.setUp("globalframe", "map")
        if self.nodes_manager.topicHasPublisher("/map_metadata"):
            dg = CustomDialog(
                self.parent,
                "Ya hay un mapa cargado",
                message="Si cambia el mapa perderá los puntos de interés que haya agregado <span style='font-weight: bold'> ¡para recuperarlos cargue el mapa de nuevo! </span>",
                positive_response="Conservar Mapa",
                negative_response="Descartar",
                retries=0,
            )
            dg.exec_()
            if dg.response == "Positive":
                return

        try:
            self.nodes_manager.bringUpStop()
            file_path, _ = QFileDialog.getOpenFileName(
                self, "Open Image File", "", "Image Files (*.yaml)"
            )
            print(file_path)
            self.map_loaded.emit(file_path)

            # self.rviz.setUp('globalframe', 'odom')
            if file_path:
                map = file_path
                self.rviz.setUp("reset", True)

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
                ntf = Notification(
                    title='Operacion realizada con exito!', 
                    msg='EL mapa ha sido subido', 
                    preset=ToastPreset.SUCCESS,
                    parent=self.parent
                )
                ntf.show()

                self.global_state_holder.currentUserOperation = userOperation.LOADMAP
                self.selected_user_operation.emit(userOperation.LOADMAP)
                return

            self.rviz.setUp("reset", True)

            self.global_state_holder.currentUserOperation = userOperation.IDLE
    

            ntf = Notification(
                title='Operacion realizada sin exito!', 
                msg='Seleccione un mapa valido!', 
                preset=ToastPreset.ERROR,
                parent=self.parent
            )
            ntf.show()

            # self.save_map_button.hide()

        except FileNotFoundError as error:
            print(error)

    def saveMapHandler(self):
        if not self.nodes_manager.nodeIsRunning("turtlebot3_slam_gmapping"):
         

            ntf = Notification(
                title='Fallo en la creación del mapa', 
                msg='El node de mapao no esta disponible!', 
                preset=ToastPreset.ERROR,
                parent=self.parent
            )
            ntf.show()
            return

        # if self.nodes_manager.topicHasPublisher("/scan"):
        # self.save_map_button.setEnabled(False)
        place_form = PlaceForm()
        dialog = InputDialog(self.parent, title='Guardar archivo de mapeo como:', child=place_form)
        dialog.exec_()
        print("VizPanel,dialog", dialog.filename)
        mapname = dialog.filename
        if mapname:
            self.nodes_manager.save_map(f"./maps/{mapname}")
            # self.nodes_manager.stopNodes(['turtlebot3_slam_gmapping'])
            ntf = Notification(
                title='Se guardo el mapa exitosamente', 
                msg='Ahora el mapa está disponible para la navegación', 
                preset=ToastPreset.SUCCESS,
                parent=self.parent
            )
            ntf.show()

            # icon = QApplication.style().standardIcon(QStyle.SP_MediaPlay)
            # self.create_map_btn.setIcon(icon)
            # self.create_map_btn.setText("Crear Mapa")
            # self.isCreateMap = True
            # # self.nodes_manager.stopNodes(['turtlebot3_slam_gmapping'])
            # self.global_state_holder.currentUserOperation = userOperation.IDLE
            # self.setCreatMapState()
            self.map_saved.emit(mapname)

        self.setMapOperationState()

    def setMapOperationState(self):
        icon = QApplication.style().standardIcon(QStyle.SP_MediaPlay)
        self.create_map_btn.setIcon(icon)
        self.create_map_btn.setText("Crear Mapa")
        self.isCreateMap = True
        # self.nodes_manager.stopNodes(['turtlebot3_slam_gmapping'])
        self.global_state_holder.currentUserOperation = userOperation.IDLE
        self.selected_user_operation.emit(userOperation.IDLE)

        self.action4.setEnabled(False)

    def handleSavePoints(self, points):
        self.update_points.emit(points)
        pass

    def handleCreateMap(self):
        if not self.currentOperationMode == operationMode.MANUAL:


            ntf = Notification(
                title='Solo se puede crear mapas en modo manual', 
                msg='Seleccione el modo manual', 
                preset=ToastPreset.WARNING,
                parent=self.parent
            )
            ntf.show()

            return

        if not self.nodes_manager.topicHasPublisher("/scan"):
  
            ntf = Notification(parent=self.parent, type=NotificationType.LIDAR_ERROR)
            ntf.show()
            # dlg = QMessageBox(self)
            # dlg.setWindowTitle("I have a question!")
            # dlg.setText("No LiDar Data, Ayyyy!!! \n Anda pasha BOBO")
            # button = dlg.exec()
            return

        self.action4.setEnabled(True)
        self.enable.emit("localization", False)
        self.global_state_holder.currentUserOperation = userOperation.CREATEMAP
        file_path = ""
        self.map_loaded.emit(file_path)

        # self.save_map_button.show()

        icon = QApplication.style().standardIcon(QStyle.SP_DialogSaveButton)
        self.create_map_btn.setIcon(icon)
        self.create_map_btn.setText("Guardar Mapa")
        self.isCreateMap = False

        self.nodes_manager.bringUpStop()
        self.nodes_manager.stopNodes(["map_server", "amcl", "turtlebot3_slam_gmapping"])
        # self.create_map_btn.setEnabled(False)
        self.nodes_manager.bringUpStart()
        self.nodes_manager.startNodes(self.nodes_manager.initNodes(self.nodes))
        self.selected_user_operation.emit(userOperation.CREATEMAP)
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
        # if self.global_state_holder.currentUserOperation == userOperation.CREATEMAP:
        self.parent.pointsWindow.show_win()
        # self.parent.pointsWindow.save_points()

    def show_dreinage_window(self, checked):
        self.drainage_checkpoints_win.show()

    def update_operation_mode(self, mode):
        self.currentOperationMode = mode
        if mode == operationMode.AUTO:
            # self.setCreatMapState
            self.setMapOperationState()
        self.rviz.setUp("reset", True)


class SelectModePanel(QGroupBox):
    set_operation_mode = pyqtSignal(operationMode)
    cancel_user_operation = pyqtSignal(userOperation)

    def __init__(
        self,
        nodes_manager=None,
        parent=None,
        patrol_panel=None,
        global_state_holder=None,
    ) -> None:
        super().__init__("Seleccione modo de operacion")
        # self.setMaximumHeight(70)
        # self.setMaximumWidth(400)
        self.patrol_panel = patrol_panel
        self.layout = QHBoxLayout()
        self.nodes_manager = nodes_manager
        self.global_state_holder = global_state_holder
        self.parent = parent
        self.map = None
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

        self.layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(self.layout)

        # self.localize_button.hide()

    def handleChangeMode(self, mode):
        if mode == operationMode.AUTO:
            self.handleAutoMode()
            return 
        if mode == operationMode.MANUAL:
            self.handleManualMode()
            return



    def checkMap(self, map_filepath):
        self.map = map_filepath

    def handleAutoMode(self):
        if self.global_state_holder.currentUserOperation == userOperation.CREATEMAP:
            dg = CustomDialog(
                self.parent,
                "Creacion de mapa sigue activo!",
                message='perderá todo el progreso que tiene hasta ahora',
                positive_response="Seguir creando",
                negative_response="Descartar",
                retries=1,
            )
            dg.exec_()
            if dg.response == "Positive":
                return
            pass

        self.cancel_user_operation.emit(userOperation.CREATEMAP)

        if not self.nodes_manager.topicHasPublisher("/scan"):

            ntf = Notification(
                type=NotificationType.LIDAR_ERROR,
                parent=self.parent
            )
            ntf.show()
            return

        if not self.nodes_manager.topicHasPublisher("/map_metadata"):

            ntf = Notification(
                title="No hay mapa disponible", 
                msg="Cargue un mapa para continuar con la operación", 
                preset=ToastPreset.ERROR,
                parent=self.parent
            )
            ntf.show()
            return

        # self.nodes_manager.stopNodes(["amcl", "move_base", "turtlebot3_slam_gmapping"])
        print("AUTO MODE", self.map)
        if not self.map:

            ntf = Notification(
                    title="No hay mapa disponible", 
                    msg="Cargue un mapa para continuar con la operación", 
                    preset=ToastPreset.ERROR,
                    parent=self.parent
                )
            ntf.show()
            return

        self.nodes_manager.stopNodes(["amcl", "move_base", "turtlebot3_slam_gmapping"])
        self.set_operation_mode.emit(operationMode.AUTO)
        self.nodes_manager.bringUpStop()
        self.nodes_manager.bringUpStart()

        self.nodes_manager.startNodes(self.nodes_manager.initNodes(self.nodes))
        self.auto_mode_button.setEnabled(False)
        self.manual_mode_button.setEnabled(True)
        self.localize_button.setEnabled(False)


        ntf = Notification(
                title="Modo auto establecido", 
                msg="Programacion de patrullajes disponible", 
                preset=ToastPreset.SUCCESS,
                parent=self.parent
            )
        ntf.show()



    def handleManualMode(self):
        patroslRunning = (
            self.patrol_panel.patrols_container.patrols_scheduler.patrolIsRunning
        )
        if patroslRunning:
            dg = CustomDialog(
                parent=self.parent,
                title="Patrullajes corriendo",
                message='Si pasa el modo manual, sera cancelada la programacion de los patrullajes.',
                positive_response="Continuar en modo Auto",
                negative_response="Cambiar a modo Manual",
                retries=0,
            )
            dg.exec_()
            if dg.response == "Positive":
                return
            self.patrol_panel.patrols_container.patrols_scheduler.cancel_task()
            self.global_state_holder.visualization_panel.rviz.setUp('reset')

        self.nodes_manager.stopNodes(["amcl", "move_base"])
        self.nodes_manager.bringUpStop()
        self.set_operation_mode.emit(operationMode.MANUAL)
        self.auto_mode_button.setEnabled(True)
        self.manual_mode_button.setEnabled(False)
        self.localize_button.setEnabled(True)


        ntf = Notification(
                title="Modo manual establecido", 
                msg="Cargue un mapa para continuar con la operacion", 
                preset=ToastPreset.SUCCESS,
                parent=self.parent
            )
        ntf.show()

    def handleLocalization(self):
        if not self.nodes_manager.topicHasPublisher("/scan"):
            # dlg = QMessageBox(self)
            # dlg.setWindowTitle("I have a question!")
            # dlg.setText("No LiDar Data, Ayyyy!!! \n Anda pasha BOBO")
            ntf = Notification(parent=self.parent, type=NotificationType.LIDAR_ERROR)
            ntf.show()

       
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
    set_stored_database_points = pyqtSignal(dict)

    def __init__(self, nodes_manager=None, parent=None) -> None:
        super().__init__("Cree, edite y programe Patrullajes")
        # self.setMaximumHeight(500)
        self.setMaximumWidth(400)
        self.setMinimumWidth(400)
        # self.scroll_area = QScrollArea()
        self.patrols_panel = QWidget()
        self.patrols_panel.layout = QVBoxLayout()
        self.layout = QVBoxLayout()
        self.patrols_scheduler = PatrolsEscheduler(parent=parent)
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
        self.current_index_label = QLabel("1")
        self.total_pages_label = QLabel("1")

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
        self.current_index_label.setMaximumWidth(50)
        self.total_pages_label.setMaximumWidth(50)

        # pixmapi =
        # icon = self.style().standardIcon(QStyle.SP_ComputerIcon)
        # self.menu_button.setIcon(icon)

        self.layout.addLayout(self.control_panel_buttons)
        self.layout.addLayout(self.navigation_buttons)
        self.layout.addWidget(self.patrols_container)

        self.setLayout(self.layout)

    def enable_components(self, enable) -> None:
        self.start_patrols_btn.setEnabled(enable)
        self.create_btn.setEnabled(enable)
        self.right_btn.setEnabled(enable)
        self.left_btn.setEnabled(enable)
        self.patrols_container.setEnabled(enable)


    def handleLoadStoredPoints(self, data):
        self.set_stored_database_points.emit(data)

    def get_current_patrols(self, patrols):
        n = len(patrols.keys())
        if (int(n%self.max_pag_count_elements)):
            additinal_page = 1
        else:
            additinal_page = 0
        self.total_pages_label.setText(
            f"{int(n//self.max_pag_count_elements) + additinal_page}"
        )

    def add_patrol(self, check):
        now = QTime.currentTime()
        hour = now.hour()
        minute = now.minute()

        if hour < 10:
            hour = f"0{hour}"
        if minute < 10:
            minute = f"0{minute}"

        self.parent.popup = PatrolsMenu(
            self.parent,
            selected_days=[],
            time=f"{hour}{minute}",
            patrolid=datetime.now().timestamp(),
        )
        self.parent.popup.update_date.connect(self.patrols_scheduler.update_patrol)
        self.parent.popup.update_date.connect(self.alert)
        self.parent.popup.show_popup()
        # max_pag, pag_index = self.patrols_container.move_page_index(0)
        # self.patrol_indexing.setText(f'{pag_index}/{max_pag}')
        # pass

    def alert(self, x):
        if self.patrols_scheduler.patrolIsRunning:
            dg = CustomDialog(
                self.parent,
                "Se detectaron algunos cambios",
                message="Se agregó un patrullaje con la ejecución  patrullajes activa, para que tus cambios surtan efecto <span style='font-weight: bold'>detén e inicia de nuevo los patrullajes</span>",
                interative=False,
            )
            dg.exec_()

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

        ntf = Notification(parent=self.parent, title='Programación iniciada con exito', msg='Continua con la operacion', preset=ToastPreset.SUCCESS)
        ntf.show()

    def start_any_patrol(self, patrolid=None):
        icon_stop = QApplication.style().standardIcon(QStyle.SP_MediaStop)
        self.start_patrols_btn.setIcon(icon_stop)
        self.start_patrols_btn.setText("Parar")
        self.isStart = False

        if not patrolid:
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
            self.start_patrols_btn.setText("Comenzar")
            self.isStart = True
            return

        ntf = Notification(
            parent=self.parent, 
            title='Los patrullajes se estan cancelando...', 
            msg='Espere a que los patrullajes se cancelen', 
            preset=ToastPreset.INFORMATION
        )
        ntf.show()


    def delete_patrols(self):
        toDelete = self.patrols_container.get_selected_patrolsid()
        self.patrols_scheduler.delete_patrols(toDelete)
        if self.patrols_scheduler.patrolIsRunning:
            dlg = CustomDialog(
                self, 
                title='Se detectaron cambios',
                message="Se borro un patrullaje mientras el la ejecucion de patrullajes esta corriendo parata que tus cambios surtan efecto <span styele='font-weight: bold'>deten e inicia de nuevo los patrullajes</span>"
            )
  
            button = dlg.exec_()

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
        if message == "TaskSuccessfully":
            toast = Toast(self.parent)
            toast.setDuration(3000)  # Hide after 5 seconds
            toast.setTitle("Todos los Patrullajes se Ejecutado con Exito")
            toast.setText("Cree nuevos patrullajes y de Iniciar")
            toast.applyPreset(ToastPreset.SUCCESS)  # Apply style preset
            Toast.setPositionRelativeToWidget(self.parent)
            toast.show()

        if message == "TaskCancelled":
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
            f"{pag_index}/{len(patrols)//max_pag_count + len(patrols)%max_pag_count}"
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
        self.max_element_view = 4
        self.patrol_elements_count = 0
        self.isSinglePatrolExec = False
        self.callback = callback
        self.running_patrol = "1"
        self.layout.setSpacing(2)

        self.patrols_scheduler.set_running_patrol.connect(self.update_running_patrol)
        self.patrols_scheduler.patrol_delayed.connect(self.handlePatrolDelayed)
        self.patrols_scheduler.points_scheduler.patrol_progress.connect(
            self.update_patrol_model
        )
        self.patrols_scheduler.single_patrol_active.connect(
            self.handleSinglePatrolActive
        )
        self.patrols_scheduler.weekday_changed.connect(self.handleWeekDayChanged)
        self.update_patrols(patrols_data)
        self.setLayout(self.layout)

    def update_patrols(self, patrols_data: dict):
        [w.setParent(None) for w in self.patrols_widgets]
        [self.layout.removeWidget(w) for w in self.patrols_widgets]
        self.patrols_widgets = []
        self.patrol_elements_count = len(patrols_data)
        patrols_items = list(patrols_data.items())
        patrols_items.reverse()
        i = 0

        for id, patrol in patrols_items[
            self.pageindex * self.max_element_view : self.max_element_view
            + self.pageindex * self.max_element_view
        ]:
            # print(patrol.keys())
            days = list(patrol["days"].keys())
            time = patrol["time"]
            state = patrol.get("state")
            current_weekday = patrol.get("current_weekday")
            delayed = patrol.get("delayed")
            print("Patrol COntiner For", patrol)
            patrol_widget = Patrol(
                parent=self.parent,
                id=id,
                days=days,
                time=time,
                patrols_scheduler=self.patrols_scheduler,
                state=state,
                current_weekday=current_weekday,
                delayed=delayed,
            )
            i = i + 1
            patrol_widget.enable_single_patrol_exec.connect(self.handleSinglepatrolExec)
            # self.patrols_scheduler.patrols_scheduling_state.connect(patrol_widget.handleSchedulingStart)
            self.patrols_scheduler.weekday_changed.connect(
                patrol_widget.handleWeekDayChanged
            )
            self.patrols_widgets.append(patrol_widget)

            print("RUNNNing PATROL id", self.running_patrol)
            if str(self.running_patrol) == str(id):
                self.patrols_scheduler.points_scheduler.patrol_progress.connect(
                    patrol_widget.update_patrol_progress
                )
            # self.patrols_scheduler.points_scheduler.patrol_progress.connect(
            #     self.update_patrol_model
            # )

        print("hola soy patrolcontainer", len(self.patrols_widgets), patrols_data)
        [self.layout.addWidget(patrol) for patrol in self.patrols_widgets]
        self.update()

    def handleWeekDayChanged(self, current_weekday):
        for id, patrol in self.patrols_scheduler.patrols_data.items():
            patrol["current_weekday"] = current_weekday
        # pass

    def handlePatrolDelayed(self, patrolid):
        if self.patrols_scheduler.patrols_data.get(patrolid):
            self.patrols_scheduler.patrols_data.get(patrolid)["delayed"] = True
        for patrol in self.patrols_widgets:
            if str(patrol.id) == str(patrolid):
                patrol.patrol_delayed_label.show()

    def update_running_patrol(self, id):
        print("ESTOY CORRIENDO ", id)
        self.running_patrol = id
        for patrol in self.patrols_widgets:
            if str(patrol.id) == str(id):
                print("ESTOY CORRIENDO IF")
                self.patrols_scheduler.points_scheduler.patrol_progress.connect(
                    patrol.update_patrol_progress
                )
                # self.patrols_scheduler.points_scheduler.patrol_progress.connect(
                # self.update_patrol_model
                # )
            # else:
            #     # button.clicked.disconnect(self.on_button_clicked)
            #     self.patrols_scheduler.points_scheduler.patrol_progress.disconnect(
            #         patrol.update_patrol_progress
            #     )

    def handleSavePointsInDatabase(self, points_to_save):
        self.patrols_scheduler.handleSavePointsInDatabase(points_to_save)

    def update_patrol_model(self, id, points_left, total_points, patrol_status):
        print(
            "PATROLD UPDATE patrolContainer",
            id,
            self.patrols_scheduler.patrols_data.get(id),
        )
        if self.patrols_scheduler.patrols_data.get(id):
            self.patrols_scheduler.patrols_data.get(id)["state"] = (
                points_left,
                total_points,
                patrol_status,
            )

    def move_page_index(self, move):
        self.pageindex = self.pageindex + move
        patrols_items = list(self.patrols_data.items())

        if len(patrols_items[ self.pageindex * self.max_element_view : self.max_element_view + self.pageindex * self.max_element_view ]):#( self.pageindex * self.max_element_view + self.max_element_view - self.patrol_elements_count % self.max_element_view) > self.patrol_elements_count:
            self.pageindex = 0
        if self.pageindex < 0:
            self.pageindex = 0

        max_pages_count = (
            self.patrol_elements_count // self.max_element_view
            + self.patrol_elements_count % self.max_element_view
        )

        return (max_pages_count, self.pageindex)

    def get_selected_patrolsid(self):
        return [patrol.selected for patrol in self.patrols_widgets if patrol.selected]

    def handleSinglepatrolExec(self, patrolid):
        for patrol in self.patrols_widgets:
            if str(patrol.id) == str(patrolid):
                self.patrols_scheduler.points_scheduler.patrol_progress.connect(
                    patrol.update_patrol_progress
                )
                self.patrols_scheduler.points_scheduler.patrol_progress.connect(
                    self.update_patrol_model
                )
        self.patrols_scheduler.start_single_patrol_scheduling(patrolid)

    def handleSinglePatrolActive(self, patrolid=None):
        self.exec_single_patrol.emit(str(patrolid))

    def patrol_finished(self):
        pass

    def add_patrol(self, patrol):
        pass


class Patrol(QGroupBox):
    enable_single_patrol_exec = pyqtSignal(str)

    def __init__(
        self,
        parent,
        id: str = "999",
        days: List[str] = [],
        time: str = "2323",
        patrols_scheduler=None,
        state=None,
        current_weekday=None,
        delayed=None,
        delay=0,
    ):
        super().__init__()
        self.days = days
        self.delay = delay
        self.id = id
        self.patrols_scheduler = patrols_scheduler
        self.time = time
        self.state = state
        self.current_weekday = current_weekday
        self.delayed = delayed
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
        self.schedule_label = QLabel("x")
        self.schedule_label.hide()
        self.patrol_status_label = QLabel("xc")
        self.patrol_status_label.hide()
        self.patrol_status_label.setAlignment(Qt.AlignLeft)
        self.patrol_delayed_label = QLabel("retrasado")
        self.patrol_delayed_label.hide()

        # self.setObjectName("mainWidget")
        self.setStyleSheet(patrol_base_style)

        print("Patrol dayleft", self.current_weekday)
        if self.current_weekday is not None:
            self.handleWeekDayChanged(self.current_weekday)

        if delayed:
            self.patrol_delayed_label.show()
            self.schedule_label.hide()

        if state:
            self.patrol_delayed_label.hide()
            self.schedule_label.hide()
            self.progress.show()
            self.points_checked_label.show()
            self.patrol_status_label.show()
            self.points_checked_label.setText(f"{state[0]}/{state[1]}")
            self.progress.setValue(100 * (state[1] - state[0]) / state[1])
            if state[2] == PatrolEndState.FINISHED:
                self.patrol_status_label.setText("terminado")
            else:
                self.patrol_status_label.setText("pendiente")

        self.layout = QGridLayout()
        self.patrol_name_label = QLabel()
        self.patrol_time_label = QLabel(f"{self.time[:2]}:{self.time[2:]}")

        (self.menu_button, self.checkbox) = (
            QPushButton("menu"),
            QCheckBox(),
        )

        self.menu_button.clicked.connect(self.edit_patrol_task)
        self.menu = QMenu()
        edit = self.menu.addAction("Editar /")
        edit.triggered.connect(self.edit_patrol)

        force_exec = self.menu.addAction("Ejecutar ⤭")
        force_exec.triggered.connect(self.force_execution)
        self.menu_button.setMenu(self.menu)

        # pixmapi = QStyle.SP_MessageBoxInformation
        # icon = self.style().standardIcon(pixmapi)
        # self.menu_button.setIcon(icon)
        self.menu.setStyleSheet(menu_style)
        self.menu_button.setStyleSheet(tertiary_button_style + button_with_menu_style)
        self.checkbox.setStyleSheet(patrol_checkbox_style)
        self.schedule_label.setStyleSheet(muted_mini_label_style)
        self.patrol_status_label.setStyleSheet(muted_mini_label_style)
        self.points_checked_label.setStyleSheet(muted_mini_label_style)
        # self.patrol_time_label.setStyleSheet("color: #2C3E50; font-weight: bold;")
        self.patrol_delayed_label.setStyleSheet("color: darkred;")

        self.patrol_name_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.patrol_time_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.patrol_delayed_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        self.patrol_name_label.setFixedWidth(120)
        metrics = QFontMetrics(self.patrol_name_label.font())
        elided_text = metrics.elidedText(
            " ".join(days), Qt.ElideRight, self.patrol_name_label.width()
        )
        self.patrol_name_label.setText(elided_text)

        # self.menu_button.setMaximumWidth(30)
        self.patrol_time_label.setToolTip("<b>Hora</b> programada para el patrullaje")
        self.patrol_name_label.setToolTip("<b>Dias</b> programados para el patrullaje")
        self.points_checked_label.setToolTip("<b>Puntos recorridos</b>/puntos totales")

        self.checkbox.stateChanged.connect(self.select)
        self.layout.setSpacing(2)

        self.layout.addWidget(self.patrol_time_label, 1, 1, alignment=Qt.AlignLeft)
        self.layout.addWidget(self.patrol_name_label, 1, 2, 1, 2)
        self.layout.addWidget(self.menu_button, 1, 4, 1, 2, alignment=Qt.AlignRight)
        self.layout.addWidget(self.progress, 2, 0, 1, 4)
        self.layout.addWidget(
            self.points_checked_label, 2, 4, 1, 1, alignment=Qt.AlignRight
        )
        self.layout.addWidget(self.checkbox, 1, 0, alignment=Qt.AlignLeft)
        self.layout.addWidget(self.schedule_label, 0, 0, 1, 4)
        self.layout.addWidget(self.patrol_delayed_label, 0, 4, 1, 2)
        self.layout.addWidget(self.patrol_status_label, 2, 5, 1, 1)
        self.setLayout(self.layout)

    def handleWeekDayChanged(self, current_weekday=None):
        # self.schedule_label.hide()
        self.schedule_label.show()
        weekdays = ["Lun", "Mar", "Mie", "Jue", "Vie", "Sab", "Dom"]
        patrols_weekdays_number = []
        days_left_per_patrolsdays = []
        for weekday in self.days:
            i = weekdays.index(weekday)
            # patrols_weekdays_number.append(i)
            days_left_per_patrolsdays.append((i - current_weekday) % 7)

        print(days_left_per_patrolsdays)
        days_left = min(days_left_per_patrolsdays)
        if days_left == 0:
            now = datetime.now()
            hour, minute = self.getDatetime(self.time)
            msg = "Comenzara hoy"
            if hour - now.hour < 0:
                msg = "Comenzara en 7 dia(s)"
            elif hour - now.hour == 0 and minute - now.minute < 0:
                msg = "Comenzara en 7 dia(s)"
                if len(days_left_per_patrolsdays) > 1:
                    days_left = min(
                        [day for day in days_left_per_patrolsdays if not day == 0]
                    )

                    if days_left == 1:
                        msg = "Comenzara mañana"
                    else:
                        msg = f"Comenzara en {days_left} dia(s)"

        elif days_left == 1:
            msg = "Comenzara mañana"
        else:
            msg = f"Comenzara en {days_left} dia(s)"

        self.schedule_label.setText(f"⏱ {msg}")
        if self.patrols_scheduler.patrols_data.get(self.id):
            self.patrols_scheduler.patrols_data.get(self.id)["current_weekday"] = (
                current_weekday
            )
        # print('WEEKDay Changed', self.patrols_scheduler.patrols_data)

    def getDatetime(self, time):
        patrol_time = list(time)
        hour = "".join(patrol_time[:2])
        minute = "".join(patrol_time[2:])
        #             "Lun": {"day": "Lun", "time": 20205, "finished": False},
        return int(hour), int(minute)

    def edit_patrol_task(self):
        self.popup = PatrolsMenu(
            self.parent,
            selected_days=self.days,
            patrolid=self.id,
            time=self.time,
            state=self.state,
            current_weekday=self.current_weekday,
        )
        self.popup.update_date.connect(self.patrols_scheduler.update_patrol)
        self.popup.show_popup()

    def edit_patrol(self):
        # print("Option 1 selected")
        self.show_edit_alert()
        self.edit_patrol_task()

    def show_edit_alert(self):
        if self.patrols_scheduler.patrolIsRunning:
            dg = CustomDialog(
                self.parent,
                "Se detectaron algununos cambios",
                message="Par que sus  cambios surtan efecto reinicie la programacion de los patrullajes, Dando click en los botones de <spna style='font-weight: bold'>Detener y luego en Comenzar</span>",
                interative=False,
            )
            dg.exec_()

    def force_execution(self):
        if self.patrols_scheduler.patrolIsRunning:
            dg = CustomDialog(
                self.parent,
                "Ejecion forzada de patrulla!",
                message="Forzar la ejecución de patrullajes, detendra la ejecion de todas las demás patrullas. Para reprogramar de click en el botón <span style='font-weight: bold'>Comenzar</span>",
                positive_response="Descartar",
                negative_response="Ejecutar de todos modos",
                retries=0,
            )
            dg.exec_()
            if dg.response == "Positive":
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

        self.enable_single_patrol_exec.emit(str(self.id))
        # self.exec_botton.setEnabled(False)
        print("force executio  selected from Patrol")

    def select(self):
        self.checked = self.checkbox.isChecked()
        if self.checked:
            self.setStyleSheet(patrol_selected_style)
        else:
            self.setStyleSheet(patrol_base_style)

    # @pyqtSlot(str, int, int)
    def update_patrol_progress(self, id, patrols_left, total_patrols, patrol_status):
        if str(id) == str(self.id):
            print(
                "Hola soy un Patrol estoy RUNNUing",
                id,
                self.id,
                int(100 * patrols_left / total_patrols),
            )
            # self.schedule_label.hide()
            self.progress.show()
            self.points_checked_label.show()
            self.patrol_status_label.show()
            self.points_checked_label.setText(f"{patrols_left}/{total_patrols}")
            self.progress.setValue(
                int(100 * (total_patrols - patrols_left) / total_patrols)
            )
            if patrol_status == PatrolEndState.FINISHED:
                self.patrols_scheduler.points_scheduler.patrol_progress.disconnect(
                    self.update_patrol_progress
                )
                self.patrol_status_label.setText("terminado")
            else:
                self.patrol_status_label.setText("pendiente")

    @property
    def selected(self):
        if self.checked:
            return self.id
        else:
            return None

    def animate_show(self):
        anim = QPropertyAnimation(self, b"maximumWidth")
        anim.setDuration(200 + 100 * self.delay)
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


class BatteryIndicator(QGroupBox):
    def __init__(self):
        super().__init__("Bateria del robot: ", None)
        self.battery_state_sub = rospy.Subscriber('/battery_state', BatteryState, self.update_battery) 
        # Create UI elements
        self.layout = QHBoxLayout()

        self.layout.setContentsMargins(1, 1, 1, 1)

        # Battery percentage with dynamic colorSP_DialogCloseButton
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

        self.layout.addWidget(self.percentage_label, alignment=Qt.AlignLeft)
        self.layout.addWidget(self.status_label, alignment=Qt.AlignRight)
        # self.layout.addWidget(self.charging_icon, alignment=Qt.AlignLeft)
        self.setLayout(self.layout)

        self.current_level = 100
        # self.timer = QTimer(self)
        # self.timer.timeout.connect(self.update_battery)
        # self.timer.start(1000)  # Update every second

    def update_battery(self, msg):
        percentage = msg.percentage*100
        self.percentage_label.setText(f"{percentage:.1f}%")
        # self.status_label.setText(status)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    t = HomePanel()
    # t = PatrolsPanel()
    # t = VisualizationPanel()
    t.show()

    sys.exit(app.exec())
