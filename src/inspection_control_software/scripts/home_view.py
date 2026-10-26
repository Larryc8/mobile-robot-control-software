"""Home tab UI: map/camera visualization, mode selection and patrol scheduling.

This is the main screen of the app ("Home" tab), assembled by
:class:`HomePanel` from three panels:

- :class:`VisualizationPanel`: RViz map view / robot camera / drainage image
  carousel, map load/create/save flow, interactive markers for checkpoints,
  battery indicator and the robot action log.
- :class:`SelectModePanel`: switches between Auto (AMCL + move_base) and
  Manual (teleop) operation modes, plus one-shot localization.
- :class:`PatrolsPanel` (+ :class:`PatrolsContainer`, :class:`Patrol`):
  create/edit/delete/run scheduled patrols and show their live progress.

:class:`BatteryIndicator` renders the live battery percentage/icon from the
``/battery_state`` topic.
"""
import re
import sys
from datetime import datetime
from enum import Enum
from typing import List

import robot_actions_logger
import rospy
from better_image_display import ImageViewer
from config_model import UserConfigFileManager
from custom_tooltip import ActionToolTip, CustomToolTip
from database_manager import AlertStatus
from image_carousel import ImageCarousel
from input_textdialog import CustomDialog, InputDialog
from interactive_markers_demo import InteractiveMarkerDemo
from joystick import Joypad
from notification import Notification, NotificationType
from patrol_menu import PatrolsMenu
from patrols_scheduler import PatrolsEscheduler
from place_form import PlaceForm
from PyQt5.QtCore import (
    QEasingCurve,
    QEvent,
    QParallelAnimationGroup,
    QPoint,
    QPropertyAnimation,
    QSettings,
    QSize,
    Qt,
    QTime,
    QTimer,
    pyqtSignal,
    pyqtSlot,
)
from PyQt5.QtGui import QFont, QFontMetrics, QIcon, QPixmap, QTransform
from PyQt5.QtWidgets import (
    QAction,
    QActionGroup,
    QApplication,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QFrame,
    QGraphicsOpacityEffect,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLayout,
    QMainWindow,
    QMenu,
    QMessageBox,
    QProgressBar,
    QPushButton,
    QScrollArea,
    QSlider,
    QStackedLayout,
    QStyle,
    QTabWidget,
    QToolButton,
    QToolTip,
    QVBoxLayout,
    QWidget,
)
from pyqttoast import Toast, ToastPosition, ToastPreset
from robot_actions_logger import RobotActionsLoggerView
from robot_camera_view import RobotCamera
from rview import MyViz
from sensor_msgs.msg import BatteryState
from styles.buttons import (
    border_button_style,
    border_button_style_danger,
    button_with_menu_style,
    colored_button_style,
    menu_style,
    minimal_button_style,
    patrol_checkbox_style,
    primary_button_style,
    secondary_button_style,
    tertiary_button_style,
    toggle_button_style,
)
from styles.labels import (
    inactive_label_style,
    info_label_style,
    minimal_label_style,
    muted_label_style,
    muted_mini_label_style,
    warning_label_style,
)
from styles.patrols import patrol_base_style, patrol_selected_style

# from points_manager import PointsGenerator
from utils.custom_file_dialog import CustomFileDialog
from utils.custom_toolbutton import CustomToolButtom
from utils.patrol import (
    MarkerActionTriggered,
    PatrolEndState,
    operationMode,
    userOperation,
)


class StackingOptions(Enum):
    """Which view (map or camera) should be rendered on top of the stack."""

    MAP_ON_TOP = 1
    CAMERA_ON_TOP = 0


class HomePanel(QWidget):
    """Top-level widget for the "Home" tab.

    Instantiates :class:`VisualizationPanel`, :class:`SelectModePanel` and
    :class:`PatrolsPanel`/:class:`Joypad`, and wires together the many
    signals connecting map loading, interactive markers, patrol scheduling
    and battery alerts between them.
    """

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

        # self.visualization_panel.map_loaded.connect(
        #     self.patrol_panel.patrols_container.patrols_scheduler.send_points_data
        # )

        self.patrol_panel.patrols_container.patrols_scheduler.set_stored_database_points.connect(
            self.visualization_panel.parent.pointsWindow.load_stored_points
        )

        self.patrol_panel.patrols_container.patrols_scheduler.points_scheduler.points_state.connect(
            self.visualization_panel.parent.pointsWindow.update_points_state
        )
        self.patrol_panel.patrols_container.patrols_scheduler.points_scheduler.recovery_mode.connect(
            self.visualization_panel.on_battery_alert
        )
        self.patrol_panel.patrols_container.patrols_scheduler.points_scheduler.points_state.connect(
            self.visualization_panel.interactive_markers.update_markers
        )  # Here is the problematic part mr Rabbit

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
        self.patrol_panel.patrols_scheduler.points_scheduler.alert_generated.connect(
            self.visualization_panel.handleAlertGeneration
        )

        # self.patrol_panel.patrols_container.patrols_scheduler.points_scheduler.points_state.connect(
        #     self.visualization_panel.drainage_checkpoints_win.update_dreinage_info
        # )

        # self.patrol_panel.patrols_container.patrols_scheduler.set_stored_database_points.connect(
        #     self.visualization_panel.drainage_checkpoints_win.load_stored_points
        # )

        self.visualization_panel.change_mode.connect(select_mode_panel.handleChangeMode)

        # self.patrol_panel.patrols_container.patrols_scheduler.set_running_patrol.connect(
        #     self.visualization_panel.drainage_checkpoints_win.reset_dreinage_status
        # )

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
    """Map/camera visualization and map lifecycle (load/create/save) panel.

    Stacks RViz (``rview.MyViz``), the robot camera and a drainage image
    carousel in one ``QStackedLayout``, and drives the create-map /
    load-map / save-map flows through :class:`~config_model.NodesManager`.
    Also hosts interactive markers for checkpoints and forwards alerts to
    on-screen toast notifications.
    """

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
        self.settings = QSettings("MyCompany", "MyApp")

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
        self.map_recent_files = []
        self._stack_policy = 0

        self.visualization_panel_static_message = QLabel()
        self.visualization_panel_static_message.hide()

        self.rviz = MyViz(configfile="./config/config_navigation.rviz")

        self.robotcamera = RobotCamera(
            buffer=self.buffer_data_robot_camera, parent=self.parent
        )

        self.user_configs = UserConfigFileManager()

        self.robot_actions_logger = RobotActionsLoggerView()
        self.logger = robot_actions_logger.logger

        self.drainage_checkpoints_win = ImageCarousel(
            buffer=self.buffer_data_robot_camera
        )

        self.drainage_checkpoints_win.on_exit_view.connect(
            lambda: self.stacklayout.setCurrentIndex(0)
        )

        # self.parent.pointWindow = None
        self.currentOperationMode = operationMode.MANUAL
        # self.global_state_holder.currentUserOperation = userOperation.IDLE
        # self.mini_display = RobotCamera()
        self.parent.pointsWindow = ImageViewer(
            nodes_manager=self.nodes_manager,
            parent=self.parent,
            # widget=self.drainage_checkpoints_win,
        )
        self.nodes = [
            {
                "package": "gmapping",
                "exec": "slam_gmapping",
                "name": "turtlebot3_slam_gmapping",
            }
        ]

        self.load_map_button = CustomToolButtom(
            icon="./public/map-question-contrast.svg", tooltip="Cargar mapa"
        )
        self.create_map_btn = CustomToolButtom(
            icon="./public/map-editing-svgrepo-com.svg",
            icon2="./public/map-save-simple.svg",
            tooltip="Crear nuevo mapa",
        )
        self.points_window_btn = CustomToolButtom(
            icon="./public/map-pin-contrast.svg", tooltip="Ver puntos"
        )
        self.robot_focus_btn = CustomToolButtom(
            icon="./public/worldwide-location-svgrepo-com.svg", tooltip="Enfocar robot"
        )
        self.map_layout_btn = CustomToolButtom(
            icon="./public/flag_corner_brackets_v3.svg", twist=True, tooltip="Mapa"
        )
        self.camera_layout_btn = CustomToolButtom(
            icon="./public/camera_centered.svg", tooltip="Cámara"
        )
        self.layout_type_btn = CustomToolButtom(
            icon="./public/squares_overlapped.svg",
            icon2="./public/squares_single_bigger.svg",
            tooltip="Layout",
        )
        self.set_reference_btn = CustomToolButtom(
            icon="./public/map_pin_stroked.svg", tooltip="Establecer referencia"
        )

        self.load_map_tutorial = ActionToolTip(self)
        # buttons and callbacks
        self.load_map_tutorial.install(
            self.load_map_button,
            "Da click para \ncargar un mapa",
            btn1_text="Entendido",
            btn1_callback=lambda x: print("Hola action"),
            btn2_text="Ignorar",
            btn2_callback=lambda x: print("Hola action"),
        )

        self.create_map_tutorial = ActionToolTip(self)
        # buttons and callbacks
        self.create_map_tutorial.install(
            self.create_map_btn,
            "Da click para \ncrear un nuevo mapa",
            btn1_text="Entendido",
            btn1_callback=lambda x: print("Hola action"),
            btn2_text="Ignorar",
            btn2_callback=lambda x: print("Hola action"),
        )

        # self.tooltip = CustomToolTip(self, delay=100)
        # self.tooltip.install(self.create_map_btn, "Haga clic para crear un nuevo mapa")

        self.view_menu_btn = QPushButton("Opciones")

        self.buttons_layout.addWidget(self.load_map_button, 0, 0)
        # self.buttons_layout.addWidget(self.points_window_btn, 0, 3)
        line = QFrame()
        line.setFrameShape(QFrame.VLine)
        # line.setLineWidth(1)
        line.setStyleSheet("QFrame { background-color: lightgray; }")
        self.buttons_layout.addWidget(line, 0, 2)

        line = QFrame()
        line.setFrameShape(QFrame.VLine)
        # line.setLineWidth(2)
        line.setStyleSheet("QFrame { background-color: lightgray; }")
        self.buttons_layout.addWidget(line, 0, 5)

        self.buttons_layout.addWidget(self.create_map_btn, 0, 1)
        self.buttons_layout.addWidget(self.robot_focus_btn, 0, 6)
        self.buttons_layout.addWidget(self.map_layout_btn, 0, 7)
        self.buttons_layout.addWidget(self.camera_layout_btn, 0, 9)
        self.buttons_layout.addWidget(self.layout_type_btn, 0, 10)
        self.buttons_layout.addWidget(self.set_reference_btn, 0, 4)

        self.set_reference_btn.clicked.connect(self.robotcamera.buffer_reference_image)
        self.robot_focus_btn.clicked.connect(self.handleActionSelected)
        self.map_layout_btn.clicked.connect(
            lambda: self.handleViewActionSelected(StackingOptions.MAP_ON_TOP)
        )
        self.camera_layout_btn.clicked.connect(
            lambda: self.handleViewActionSelected(StackingOptions.CAMERA_ON_TOP)
        )
        self.layout_type_btn.clicked.connect(self.toggleLayoutType)
        # self.stack_policy_btn.clicked.connect(self.toggleStackPolicy)

        # self.set_reference_btn.setEnabled(False)

        # Create the QMenu
        view_menu = QMenu(self)
        view_options_action_group = QActionGroup(self)

        # Add actions to the menu
        action1 = view_menu.addAction("Modo de pantalla: Camara")
        action2 = view_menu.addAction("Modo de pantalla: Mapa")

        action1.setCheckable(True)
        action2.setCheckable(True)

        view_options_action_group.addAction(action1)
        view_options_action_group.addAction(action2)

        view_menu.addSeparator()  # Add a separator line
        self.action3 = view_menu.addAction("Enfocar vista al robot")
        self.action3.setWhatsThis("No se que poner")
        self.action3.setCheckable(True)

        view_menu.addSeparator()  # Add a separator line
        self.action4 = view_menu.addAction("Marcar lugar como referencia")
        self.action4.setEnabled(False)

        # Connect actions to methods
        action1.triggered.connect(lambda: self.toggleCameraMapView())
        action2.triggered.connect(lambda: self.toggleCameraMapView())
        self.action3.triggered.connect(lambda: self.handleActionSelected(self.action3))
        self.action4.triggered.connect(
            lambda: self.robotcamera.buffer_reference_image()
        )

        # Set the menu to the button
        self.view_menu_btn.setMenu(view_menu)

        self.points_window_btn.clicked.connect(self.show_points_window)

        map_files_menu = QMenu(self)
        new_action = QAction("cargar mapa", self)
        new_action.triggered.connect(self.handleLoadMap)
        map_files_menu.addAction(new_action)

        self.recent_files_menu = map_files_menu.addMenu("recent files")

        new_action2 = QAction("New File 2", self)
        new_action2.triggered.connect(lambda: print("adios"))
        self.recent_files_menu.addAction(new_action2)

        # config = self.user_configs.read_data()  # Add recent files
        # print(config)
        # self.map_recent_files = self.setup_recent_files(config["recent_map_files"])

        # self.load_map_button.setMenu(map_files_menu)
        self.load_map_button.clicked.connect(self.handleLoadMap)

        # self.view_menu_btn.setStyleSheet(border_button_style + button_with_menu_style)
        self.visualization_panel_static_message.setStyleSheet(warning_label_style)
        view_menu.setStyleSheet(menu_style)
        self.view_menu_btn.setStyleSheet(tertiary_button_style + button_with_menu_style)

        # self.load_map_button.clicked.connect(self.handleLoadMap)
        # self.map_loaded.connect(self.parent.pointsWindow.load_map)
        self.create_map_btn.clicked.connect(self.toggleCreaeteSaveMap)

        menu = QMenu("checkpoints", self)
        menu.setStyleSheet(menu_style)
        show_checkpoints_action = QAction("Puntos de interes", self)
        show_dreinage_action = QAction("Fotos de referencia", self)

        # show_checkpoints_action.triggered.connect(self.show_points_window)
        menu.addAction(show_checkpoints_action)
        show_dreinage_action.triggered.connect(self.show_dreinage_window)
        menu.addAction(show_dreinage_action)
        # self.points_window_btn.setMenu(menu)
        # self.save_button.setEnabled(False)
        #

        self.logger.log_changed.connect(self.robot_actions_logger.update_log)
        self.robot_actions_logger.log_file_updated.connect(self.logger.update_log_file)

        # self.oints_window_btn.clicked.connect(self.show_points_window)
        self.parent.pointsWindow.save_selected_points.connect(self.handleSavePoints)
        self.interactive_markers = InteractiveMarkerDemo()
        self.interactive_markers.points_changed.connect(self.handleSavePoints)
        self.interactive_markers.action_triggered.connect(
            self.handleMarkerActionTriggered
        )
        self.selected_user_operation.connect(
            self.interactive_markers.set_user_operation
        )
        self.map_loaded.connect(self.interactive_markers.send_database_action)

        self.parent.pointsWindow.save_in_database.connect(self.handleSaveInDatabase)
        self.map_saved.connect(self.robotcamera.save_buffered_data)
        # self.robotcamera.send_buffered_data.connect(
        #     self.drainage_checkpoints_win.load_images
        # )
        # self.robotcamera.custom_option_clicked.connect(self.toggleCameraMapView)
        # self.selected_user_operation.connect(
        #     self.drainage_checkpoints_win.get_user_operation
        # )

        self.stacklayout.addWidget(self.rviz)
        self.stacklayout.addWidget(self.robotcamera)
        self.stacklayout.addWidget(self.drainage_checkpoints_win)
        # self.setView(0)
        # self.rviz.hide()
        # self.toggleCameraMapView()
        self.battery_state = BatteryIndicator()
        self.battery_state.battery_alert.connect(self.on_battery_alert)

        # self.layout.addLayout(self.rviz_options_layout, 0, 0)
        self.layout.addLayout(self.buttons_layout, 1, 0)
        self.layout.addWidget(self.visualization_panel_static_message, 0, 0, 1, -1)
        self.layout.addWidget(self.battery_state, 1, 3)
        self.layout.addWidget(self.stacked_widgets_container, 2, 0, 1, 4)
        self.layout.addWidget(self.robot_actions_logger, 3, 0, 1, 4)
        self.setLayout(self.layout)

        self.layout.setAlignment(self.battery_state, Qt.AlignLeft)

        self.settings.setValue("tutorial_step", 0)
        self.timer = QTimer(self)
        self.timer.setSingleShot(True)
        self.timer.timeout.connect(self.on_mount)
        self.timer.start(2000)

    def on_battery_alert(self, message):
        """Battery-alert hook (currently a no-op; UI update is commented out)."""
        pass
        # if message:
        #     self.visualization_panel_static_message.setText(message)
        #     self.visualization_panel_static_message.show()
        # else:
        #     self.visualization_panel_static_message.hide()

    def on_mount(self):
        """One-shot timer callback meant to drive a first-run tutorial
        (currently returns early, disabling the tutorial)."""
        followTutorial = self.settings.value("followTutorial")
        if True:
            return

        dg = CustomDialog(
            self,
            "Tutorial",
            message="Desea seguir e tutorial?",
            positive_response="Seguir",
            negative_response="Ignorar",
            retries=0,
        )
        if self.settings.value("tutorial_step") == 0:
            dg.exec_()

        if not "Positive" == "Positive":
            self.settings.setValue("followTutorial", False)
            print("Tutorial ignorado")
        else:
            step = self.settings.value("tutorial_step")

            if step == 0:
                self.load_map_tutorial.show_tooltip()
                self.settings.setValue("tutorial_step", 1)
                self.timer.start(1000)

            if step == 1:
                self.create_map_tutorial.show_tooltip()
                self.settings.setValue("tutorial_step", 2)
                self.timer.start(1000)

    def tutorial(self):
        """Unused tutorial hook (no-op)."""
        pass

    def enable_components(self, enable):
        """Show/hide the calibration-in-progress banner (disables patrol menu)."""
        if not enable:
            self.visualization_panel_static_message.show()
            self.visualization_panel_static_message.setText(
                "Calibración activa. El menú de patrullajes estará desactivado"
            )
        else:
            self.visualization_panel_static_message.hide()

    def handleMarkerActionTriggered(self, action, marker):
        """React to an interactive-marker action; ``HELLO`` shows the
        marker's reference image in the drainage carousel."""
        if action == MarkerActionTriggered.HELLO:
            self.stacklayout.setStackingMode(QStackedLayout.StackingMode.StackOne)
            self.stacklayout.setCurrentIndex(2)
            self.drainage_checkpoints_win.set_main_image(marker.mesh_resource)
        elif action == MarkerActionTriggered.GOODBYE:
            pass
        elif action == MarkerActionTriggered.RESET:
            pass
        elif action == MarkerActionTriggered.LOG_STATUS:
            pass
        elif action == MarkerActionTriggered.DELETE_MARKER:
            pass

    def toggleStackPolicy(self):
        """Cycle the (currently unused) stack-policy flag between 0 and 1."""
        self._stack_policy = (self._stack_policy + 1) % 2

    def toggleLayoutType(self):
        """Switch the view stack between showing all widgets overlaid or one at a time."""
        if self.layout_type_btn.toggle():
            self.stacklayout.setStackingMode(QStackedLayout.StackingMode.StackAll)
        else:
            self.stacklayout.setStackingMode(QStackedLayout.StackingMode.StackOne)

    def handleActionSelected(self, action3):
        """Toggle "follow robot" camera mode in RViz."""
        self.robot_focus_btn.toggle_selected()
        self.setFollowRobot(self.robot_focus_btn.isSelected())

    def show_log(self):
        """Switch the stacked view to the drainage/log image carousel."""
        self.stacklayout.setCurrentIndex(2)

    def handleAlertGeneration(self, message, status):
        """Show a toast notification styled by the alert's severity."""
        if status == AlertStatus.ERROR:
            ntf = Notification(
                title="Ha ocurrido una alerta",
                msg=f"{message}",
                preset=ToastPreset.ERROR,
                parent=self.parent,
            )
        if status == AlertStatus.WARNING:
            ntf = Notification(
                title="Ha ocurrido una alerta",
                msg=f"{message}",
                preset=ToastPreset.WARNING,
                parent=self.parent,
            )
        if status == AlertStatus.INFO:
            ntf = Notification(
                title="Ha ocurrido una alerta",
                msg=f"{message}",
                preset=ToastPreset.INFO,
                parent=self.parent,
            )

        ntf.show()

    def handleViewActionSelected(self, index):
        """Bring the map or the camera to the front of the overlaid stack."""
        self.robotcamera.setMaximumSize(2000, 1000)
        self.rviz.setMaximumSize(2000, 1000)

        if index == StackingOptions.MAP_ON_TOP:
            if not self.map_layout_btn.isSelected():
                self.map_layout_btn.toggle_selected()
                self.camera_layout_btn.toggle_selected()
                self.stacklayout.setCurrentIndex(0)
            if self.stacklayout.stackingMode() == QStackedLayout.StackingMode.StackAll:
                self.stacklayout.currentWidget().setMaximumSize(200, 100)
                # self.toggleCameraMapView()

        if index == StackingOptions.CAMERA_ON_TOP:
            if not self.camera_layout_btn.isSelected():
                self.camera_layout_btn.toggle_selected()
                self.map_layout_btn.toggle_selected()
                self.stacklayout.setCurrentIndex(1)
            if self.stacklayout.stackingMode() == QStackedLayout.StackingMode.StackAll:
                self.stacklayout.currentWidget().setMaximumSize(200, 100)
                # self.toggleCameraMapView()
            return

    def toggleCameraMapView(self):
        """Swap which of map/camera is the large principal view vs. the small overlay."""
        if self.mapAsPrincipalView:
            self.stacklayout.setCurrentIndex(1)
            self.robotcamera.raise_()
            # print(self.stacklayout.currentWidget())
            self.robotcamera.update()
            self.mapAsPrincipalView = False
            self.robotcamera.setMaximumSize(250, 190)
            # self.robotcamera.move(50, 50)
            self.rviz.setMaximumSize(2000, 1000)
            return
        else:
            self.stacklayout.setCurrentIndex(0)
            self.rviz.raise_()
            print(self.stacklayout.currentWidget())
            self.rviz.update()
            self.mapAsPrincipalView = True
            self.rviz.setMaximumSize(250, 250)
            self.rviz.move(50, 50)
            self.robotcamera.setMaximumSize(2000, 1000)

    def setView(self, index):
        """Bring ``index`` to the front (large) and shrink the other stacked widget."""
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
        """Keep the current stacked widget raised above the others on every repaint."""
        currentWidget = self.stacklayout.currentWidget()
        currentWidget.raise_()
        currentWidget.update()
        super().paintEvent(event)

    def toggleCreaeteSaveMap(self):
        """Toggle the create-map button between starting map creation and saving it."""
        if self.isCreateMap:
            if self.handleCreateMap():
                self.create_map_btn.toggle()
        else:
            if self.saveMapHandler():
                self.create_map_btn.toggle()

    def setFollowRobot(self, x):
        """Toggle RViz's camera-follows-robot setting."""
        self.rviz.setUp("followrobot", x)
        # if not x:
        #     self.rviz.setUp("scale", 60)
        #     self.rviz.setUp("globalframe", "map")

    def handleSaveInDatabase(self, data):
        """Forward a save-to-database request up to the parent panel."""
        self.save_in_database.emit(data)

    def add_recent_file(self, file):
        """Append ``file`` to the recent-maps list, keeping at most 4 entries."""
        if len(self.map_recent_files) > 3:
            self.map_recent_files.pop(0)
            self.map_recent_files.append(file)
        else:
            self.map_recent_files.append(file)

        self.setup_recent_files(self.map_recent_files)
        # self.user_configs.update_value("recent_map_files", self.map_recent_files)

    def setup_recent_files(self, files):
        """Rebuild the "recent files" menu (currently a no-op stub)."""
        print(files)
        # for file_path in files:
        #     print(file_path)
        #     name = file_path.split("/")[-1]
        #     new_action2 = QAction(name, self)
        #     new_action2.triggered.connect(lambda: self.handleLoadMap(file_path))
        #     self.recent_files_menu.addAction(new_action2)
        # return files

    def open_recent_file(self, file):
        """Load a map picked from the recent-files menu."""
        self.handleLoadMap(file)
        pass

    def handleLoadMap(self, file=None):
        """Drive the full map-loading flow: guard against Auto mode / an
        in-progress map creation / an already loaded map, prompt for a
        file if none was given, then start the ``map_server`` node for it."""
        if self.currentOperationMode == operationMode.AUTO:
            dg = CustomDialog(
                self.parent,
                "No se puede cargar mapa en el modo Auto",
                message="Para cargar un nuevo mapa cambie al modo manual",
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
                message="perderá todo el progreso que tiene hasta ahora!",
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
            file_path = file  # God, please help me!

            if not file:
                dialog = CustomFileDialog()
                if dialog.exec_():
                    file_path = dialog.selected_file
                    print("MAP FILE LOADED", file_path)

            if not file_path:
                return

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
                    title="Operacion realizada con exito!",
                    msg="EL mapa ha sido subido",
                    preset=ToastPreset.SUCCESS,
                    parent=self.parent,
                )
                ntf.show()

                self.global_state_holder.currentUserOperation = userOperation.LOADMAP
                self.selected_user_operation.emit(userOperation.LOADMAP)
                return

            self.rviz.setUp("reset", True)

            self.global_state_holder.currentUserOperation = userOperation.IDLE

            ntf = Notification(
                title="Operacion realizada sin exito!",
                msg="Seleccione un mapa valido!",
                preset=ToastPreset.ERROR,
                parent=self.parent,
            )
            ntf.show()

            # self.save_map_button.hide()

        except FileNotFoundError as error:
            print(error)

    def saveMapHandler(self) -> bool:
        """Prompt for a filename and save the currently built map via ``NodesManager``."""
        if not self.nodes_manager.nodeIsRunning("turtlebot3_slam_gmapping"):
            ntf = Notification(
                title="Fallo en la creación del mapa",
                msg="El node de mapao no esta disponible!",
                preset=ToastPreset.ERROR,
                parent=self.parent,
            )
            ntf.show()
            return False

        # if self.nodes_manager.topicHasPublisher("/scan"):
        # self.save_map_button.setEnabled(False)
        place_form = PlaceForm()
        dialog = InputDialog(
            self.parent,
            title="Guardar archivo de mapeo como:",
            child=place_form,
            msg="Nombre del archivo:",
        )
        dialog.exec_()
        print("VizPanel,dialog", dialog.filename)
        mapname = dialog.filename
        if mapname:
            self.nodes_manager.save_map(mapname)
            # self.nodes_manager.stopNodes(['turtlebot3_slam_gmapping'])
            ntf = Notification(
                title="Se guardo el mapa exitosamente",
                msg="Ahora el mapa está disponible para la navegación",
                preset=ToastPreset.SUCCESS,
                parent=self.parent,
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
        return True

    def setMapOperationState(self):
        """Reset the current user operation to IDLE after finishing a map operation."""
        icon = QApplication.style().standardIcon(QStyle.SP_MediaPlay)
        self.isCreateMap = True
        # self.nodes_manager.stopNodes(['turtlebot3_slam_gmapping'])
        self.global_state_holder.currentUserOperation = userOperation.IDLE
        self.selected_user_operation.emit(userOperation.IDLE)

        self.set_reference_btn.setEnabled(False)

    def handleSavePoints(self, points):
        """Forward newly placed checkpoints up to the patrol panel."""
        self.update_points.emit(points)
        pass

    def handleCreateMap(self) -> bool:
        """Start the gmapping SLAM node to begin building a new map
        (requires Manual mode and a live ``/scan`` publisher)."""
        if not self.currentOperationMode == operationMode.MANUAL:
            ntf = Notification(
                title="Solo se puede crear mapas en modo manual",
                msg="Seleccione el modo manual",
                preset=ToastPreset.WARNING,
                parent=self.parent,
            )
            ntf.show()

            return False

        if not self.nodes_manager.topicHasPublisher("/scan"):
            ntf = Notification(parent=self.parent, type=NotificationType.LIDAR_ERROR)
            ntf.show()
            return False

        self.action4.setEnabled(True)
        self.enable.emit("localization", False)
        self.global_state_holder.currentUserOperation = userOperation.CREATEMAP
        file_path = ""
        # self.map_loaded.emit(file_path)

        # self.save_map_button.show()

        icon = QApplication.style().standardIcon(QStyle.SP_DialogSaveButton)
        self.isCreateMap = False

        self.nodes_manager.bringUpStop()
        self.nodes_manager.stopNodes(["map_server", "amcl", "turtlebot3_slam_gmapping"])
        # self.create_map_btn.setEnabled(False)
        self.nodes_manager.bringUpStart()
        self.nodes_manager.startNodes(self.nodes_manager.initNodes(self.nodes))
        self.selected_user_operation.emit(userOperation.CREATEMAP)

        return True

    def show_points_window(self, checked):
        """Show the checkpoints/points window."""
        # self.parent.w = None
        # if self.parent.pointsWindow is None:
        # self.parent.pointsWindow = ImageViewer()
        # print('None ;; multiwindows')
        # if self.global_state_holder.currentUserOperation == userOperation.CREATEMAP:
        self.parent.pointsWindow.show_win()
        # self.parent.pointsWindow.save_points()

    def show_dreinage_window(self, checked):
        """Show the drainage reference-photos window (currently a no-op stub)."""
        # self.drainage_checkpoints_win.show()
        pass

    def update_operation_mode(self, mode):
        """Track the active operation mode and reset map state when switching to Auto."""
        self.currentOperationMode = mode
        if mode == operationMode.AUTO:
            # self.setCreatMapState
            self.setMapOperationState()
        self.rviz.setUp("reset", True)


class SelectModePanel(QGroupBox):
    """Auto/Manual/Localize mode switcher.

    Guards each transition with the right ROS node checks (``/scan``,
    ``/map_metadata``) and starts/stops ``amcl``/``move_base`` accordingly,
    emitting ``set_operation_mode`` so other panels can react.
    """

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
        """Dispatch to :meth:`handleAutoMode`/:meth:`handleManualMode` for ``mode``."""
        self.global_state_holder.visualization_panel.rviz.setUp("reset")

        if mode == operationMode.AUTO:
            self.handleAutoMode()
            return
        if mode == operationMode.MANUAL:
            self.handleManualMode()
            return

    def checkMap(self, map_filepath):
        """Remember the currently loaded map file (needed to enable Auto mode)."""
        self.map = map_filepath

    def handleAutoMode(self):
        """Switch to Auto mode: guard against map-creation-in-progress and
        missing lidar/map, then start ``amcl``/``move_base``."""
        if self.global_state_holder.currentUserOperation == userOperation.CREATEMAP:
            dg = CustomDialog(
                self.parent,
                "Creacion de mapa sigue activo!",
                message="perderá todo el progreso que tiene hasta ahora",
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
            ntf = Notification(type=NotificationType.LIDAR_ERROR, parent=self.parent)
            ntf.show()
            return

        if not self.nodes_manager.topicHasPublisher("/map_metadata"):
            ntf = Notification(
                title="No hay mapa disponible",
                msg="Cargue un mapa para continuar con la operación",
                preset=ToastPreset.ERROR,
                parent=self.parent,
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
                parent=self.parent,
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
            parent=self.parent,
        )
        ntf.show()

    def handleManualMode(self):
        """Switch to Manual mode, cancelling any running patrol first (with confirmation)."""
        patroslRunning = (
            self.patrol_panel.patrols_container.patrols_scheduler.patrolIsRunning
        )
        if patroslRunning:
            dg = CustomDialog(
                parent=self.parent,
                title="Patrullajes corriendo",
                message="Si pasa el modo manual, sera cancelada la programacion de los patrullajes.",
                positive_response="Continuar en modo Auto",
                negative_response="Cambiar a modo Manual",
                retries=0,
            )
            dg.exec_()
            if dg.response == "Positive":
                return
            self.patrol_panel.patrols_container.patrols_scheduler.cancel_task()
            self.global_state_holder.visualization_panel.rviz.setUp("reset")

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
            parent=self.parent,
        )
        ntf.show()

    def handleLocalization(self):
        """Restart just the ``amcl`` node to (re)localize the robot on the current map."""
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
        """Enable/disable one named sub-control (currently only "localization")."""
        if object == "localization":
            self.localize_button.setEnabled(state)

    # def update_operation_mode(self, mode):
    #     self.currentOperationMode = mode


class PatrolsPanel(QGroupBox):
    """"Cree, edite y programe Patrullajes" panel.

    Holds the create/delete/start-stop controls and pagination for the
    patrol list, delegating scheduling logic to
    :class:`~patrols_scheduler.PatrolsEscheduler` and rendering to
    :class:`PatrolsContainer`.
    """

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
        # Setup the controller

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
        self.patrols_scheduler.patrol_finished.connect(self.on_patrol_finished)
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

        self.delete_btn.setStyleSheet(border_button_style)
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
        """Enable/disable every patrol control (used during calibration)."""
        self.start_patrols_btn.setEnabled(enable)
        self.delete_btn.setEnabled(enable)
        self.create_btn.setEnabled(enable)
        self.right_btn.setEnabled(enable)
        self.left_btn.setEnabled(enable)
        self.patrols_container.setEnabled(enable)
        print("patrols calibration", enable)

    def handleLoadStoredPoints(self, data):
        """Forward checkpoints loaded from the database up to listeners."""
        self.set_stored_database_points.emit(data)

    def get_current_patrols(self, patrols):
        """Update the total-pages label from the current patrol count."""
        n = len(patrols.keys())
        if int(n % self.max_pag_count_elements):
            additinal_page = 1
        else:
            additinal_page = 0
        self.total_pages_label.setText(
            f"{int(n // self.max_pag_count_elements) + additinal_page}"
        )

    def add_patrol(self, check):
        """Open the :class:`~patrol_menu.PatrolsMenu` popup to create a new patrol."""
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
        """Warn the user that a new/edited patrol won't take effect until
        the scheduler is restarted, if one is currently running."""
        if self.patrols_scheduler.patrolIsRunning:
            dg = CustomDialog(
                self.parent,
                "Se detectaron algunos cambios",
                message="Se agregó un patrullaje con la ejecución  patrullajes activa, para que tus cambios surtan efecto <span style='font-weight: bold'>detén e inicia de nuevo los patrullajes</span>",
                interative=False,
            )
            dg.exec_()

    def toggleStartStop(self):
        """Start or stop the whole patrol schedule depending on current state."""
        if self.isStart:
            self.start_patrols()
            return
        self.stop_any_patrol()

    def start_patrols(self):
        """Validate preconditions (amcl/move_base running, points and
        patrols exist, nothing already running) then start scheduling."""
        if not self.nodes_manager.nodeIsRunning("amcl"):
            ntf = Notification(
                parent=self.parent,
                title="Fallo en inicio de patrullajes",
                msg="El nodo acml no esta activo",
                preset=ToastPreset.ERROR,
            )
            ntf.show()
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

        ntf = Notification(
            parent=self.parent,
            title="Programación iniciada con exito",
            msg="Continua con la operacion",
            preset=ToastPreset.SUCCESS,
        )
        ntf.show()

    def start_any_patrol(self, patrolid=None):
        """Flip the start button to "stop" and, if no single patrol id was
        given, start the full scheduler."""
        icon_stop = QApplication.style().standardIcon(QStyle.SP_MediaStop)
        self.start_patrols_btn.setIcon(icon_stop)
        self.start_patrols_btn.setText("Parar")
        self.isStart = False

        if not patrolid:
            self.patrols_scheduler.start_patrols()
        pass

    def stop_any_patrol(self, c=None):
        """Cancel the running patrol task and flip the start button back to "start"."""
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
            title="Los patrullajes se estan cancelando...",
            msg="Espere a que los patrullajes se cancelen",
            preset=ToastPreset.INFORMATION,
        )
        ntf.show()

    def delete_patrols(self):
        """Delete the checked patrols, warning if the scheduler is running."""
        toDelete = self.patrols_container.get_selected_patrolsid()
        self.patrols_scheduler.delete_patrols(toDelete)
        if self.patrols_scheduler.patrolIsRunning:
            dlg = CustomDialog(
                self,
                title="Se detectaron cambios",
                message="Se borro un patrullaje mientras el la ejecucion de patrullajes esta corriendo parata que tus cambios surtan efecto <span styele='font-weight: bold'>deten e inicia de nuevo los patrullajes</span>",
            )

            button = dlg.exec_()

        # max_pag, pag_index = self.patrols_container.move_page_index(0)
        # self.patrol_indexing.setText(f'{pag_index}/{max_pag}')

    def handleSinglepatrolExec(self, x=None):
        """Hook for single-patrol execution (currently a no-op stub)."""
        # self.start_patrols_btn.setEnabled(False)
        # self.create_btn.setEnabled(False)
        # self.delete_btn.setEnabled(False)
        pass

    def on_patrol_finished(self, message):
        """Show a toast for the scheduler's outcome and reset the start button."""
        # self.start_patrols_btn.setEnabled(False)
        # self.create_btn.setEnabled(False)
        # self.delete_btn.setEnabled(False)
        if message == "TaskSuccessfully":
            ntf = Notification(
                parent=self.parent,
                title="Todos los Patrullajes se Ejecutado con Exito",
                msg="Cree nuevos patrullajes y de Iniciar",
                preset=ToastPreset.INFORMATION,
            )
            ntf.show()

        if message == "ForcedTaskSuccessfully":
            ntf = Notification(
                parent=self.parent,
                title="Patrullaje forzado",
                msg="NA",
                preset=ToastPreset.INFORMATION,
            )
            ntf.show()

        if message == "TaskCancelled":
            ntf = Notification(
                parent=self.parent,
                title="La ejecucion ha sido cancelada",
                msg="Cree nuevos patrullajes y de Iniciar",
                preset=ToastPreset.INFORMATION,
            )
            ntf.show()

        icon_start = QApplication.style().standardIcon(QStyle.SP_MediaPlay)
        self.start_patrols_btn.setIcon(icon_start)
        self.start_patrols_btn.setText("Comenzar")
        self.isStart = True

        if "HomeDoesntExist" in message:
            dg = CustomDialog(
                None,
                "Home no encontrado",
                message="Establezca un púnto Home antes antes de iniciar la inspección",
                positive_response="Seguir",
                negative_response="Ignorar",
                retries=0,
            )
            dg.exec_()

    def set_next_page(self):
        """Move the patrol list one page forward."""
        max_pag, pag_index = self.patrols_container.move_page_index(1)
        self.current_index_label.setText(f"{1 + pag_index}")
        # print('nex page fuc', a)
        self.patrols_scheduler.get_current_patrols()

    def set_previous_page(self):
        """Move the patrol list one page back."""
        max_pag, pag_index = self.patrols_container.move_page_index(-1)
        self.current_index_label.setText(f"{1 + pag_index}")
        self.patrols_scheduler.get_current_patrols()

    def update_patrols_indexing_label(self, patrols):
        """Refresh the "page X/Y" label (currently unused/disconnected)."""
        max_pag_count, pag_index = self.patrols_container.move_page_index(0)
        if max_pag_count == 0:
            self.current_index_label.setText(f"{1}/{1}")
            return
        self.current_index_label.setText(
            f"{pag_index}/{len(patrols) // max_pag_count + len(patrols) % max_pag_count}"
        )

        # self.patrol_indexing.setText(f'{pag_index}/{max_pag}')
        pass

    def update_points(self, points):
        """Pass the latest checkpoint set down to the patrols scheduler."""
        self.patrols_scheduler.setPointsToVisit(points)
        # print("from patrolpanel "ToDelete, points)
        pass


class PatrolsContainer(QWidget):
    """Renders the current page of :class:`Patrol` widgets and keeps them
    wired to the scheduler's progress/weekday/running-patrol signals."""

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
        """Rebuild the visible :class:`Patrol` widgets for the current page."""
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
        """Stamp every tracked patrol with the newly current weekday."""
        for id, patrol in self.patrols_scheduler.patrols_data.items():
            patrol["current_weekday"] = current_weekday
        # pass

    def handlePatrolDelayed(self, patrolid):
        """Mark a patrol as delayed and show its "retrasado" label."""
        if self.patrols_scheduler.patrols_data.get(patrolid):
            self.patrols_scheduler.patrols_data.get(patrolid)["delayed"] = True
        for patrol in self.patrols_widgets:
            if str(patrol.id) == str(patrolid):
                patrol.patrol_delayed_label.show()

    def update_running_patrol(self, id):
        """Track which patrol is running and wire its progress updates."""
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
        """Forward a batch of checkpoints to the scheduler for persistence."""
        self.patrols_scheduler.handleSavePointsInDatabase(points_to_save)

    def update_patrol_model(self, id, points_left, total_points, patrol_status):
        """Record a patrol's progress/status into ``patrols_data``."""
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
        """Shift the current page by ``move`` pages (clamped to valid range)
        and return ``(max_pages_count, pageindex)``."""
        self.pageindex = self.pageindex + move
        patrols_items = list(self.patrols_data.items())

        if len(
            patrols_items[
                self.pageindex * self.max_element_view : self.max_element_view
                + self.pageindex * self.max_element_view
            ]
        ):  # ( self.pageindex * self.max_element_view + self.max_element_view - self.patrol_elements_count % self.max_element_view) > self.patrol_elements_count:
            self.pageindex = 0
        if self.pageindex < 0:
            self.pageindex = 0

        max_pages_count = (
            self.patrol_elements_count // self.max_element_view
            + self.patrol_elements_count % self.max_element_view
        )

        return (max_pages_count, self.pageindex)

    def get_selected_patrolsid(self):
        """Return the ids of all currently checked patrols."""
        return [patrol.selected for patrol in self.patrols_widgets if patrol.selected]

    def handleSinglepatrolExec(self, patrolid):
        """Wire progress signals for ``patrolid`` and force its execution."""
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
        """Forward which patrol became active for single-patrol execution."""
        self.exec_single_patrol.emit(str(patrolid))

    def patrol_finished(self):
        """Unused completion hook (no-op)."""
        pass

    def add_patrol(self, patrol):
        """Unused add-patrol hook (no-op; patrols are added via :meth:`update_patrols`)."""
        pass


class Patrol(QGroupBox):
    """One patrol card: schedule/progress labels, a selection checkbox and a
    menu to edit or force-execute it."""

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

        # from custom_tooltip import CustomToolTip

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
        """Recompute and show the "starts in N day(s)" schedule label."""
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
        """Parse an ``HHMM`` string into an ``(hour, minute)`` int tuple."""
        patrol_time = list(time)
        hour = "".join(patrol_time[:2])
        minute = "".join(patrol_time[2:])
        #             "Lun": {"day": "Lun", "time": 20205, "finished": False},
        return int(hour), int(minute)

    def edit_patrol_task(self):
        """Open the :class:`~patrol_menu.PatrolsMenu` popup pre-filled for this patrol."""
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
        """Menu action: warn if needed, then open the edit popup."""
        # print("Option 1 selected")
        self.show_edit_alert()
        self.edit_patrol_task()

    def show_edit_alert(self):
        """Warn that editing won't take effect until the scheduler restarts."""
        if self.patrols_scheduler.patrolIsRunning:
            dg = CustomDialog(
                self.parent,
                "Se detectaron algununos cambios",
                message="Par que sus  cambios surtan efecto reinicie la programacion de los patrullajes, Dando click en los botones de <spna style='font-weight: bold'>Detener y luego en Comenzar</span>",
                interative=False,
            )
            dg.exec_()

    def force_execution(self):
        """Menu action: force-run this patrol now, confirming it will
        preempt any other running patrol."""
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
            ntf = Notification(
                parent=self.parent,
                title="Fallo inicio de patrullajes",
                msg="No hay puntos de interés",
                preset=ToastPreset.ERROR,
            )
            ntf.show()
            return

        self.enable_single_patrol_exec.emit(str(self.id))
        # self.exec_botton.setEnabled(False)
        print("force executio  selected from Patrol")

    def select(self):
        """Sync the checked/selected visual state with the checkbox."""
        self.checked = self.checkbox.isChecked()
        if self.checked:
            self.setStyleSheet(patrol_selected_style)
        else:
            self.setStyleSheet(patrol_base_style)

    # @pyqtSlot(str, int, int)
    def update_patrol_progress(self, id, patrols_left, total_patrols, patrol_status):
        """Update this card's progress bar/labels if the update is for this patrol."""
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
        """This patrol's id if checked, else ``None``."""
        if self.checked:
            return self.id
        else:
            return None

    def animate_show(self):
        """Unused entrance animation (width + fade-in) for this card."""
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


class BatteryIndicator(QWidget):
    """Subscribes to ``/battery_state`` and renders a live icon/percentage,
    emitting ``battery_alert`` once when the level first drops critical."""

    battery_alert = pyqtSignal(str)

    def __init__(self):
        super().__init__(None)
        self.battery_state_sub = rospy.Subscriber(
            "/battery_state", BatteryState, self.battery_callback
        )
        self.percentage = 100
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_battery_state)
        self.timer.start(30)  # Update at ~30fps

        # Create UI elements
        self.layout = QHBoxLayout()

        self.layout.setContentsMargins(1, 1, 1, 1)
        self.showBatteryWarning = True

        # Battery percentage with dynamic colorSP_DialogCloseButton
        self.battery_icon = QPushButton()
        icon_size = QSize(30, 30)  # Set width and height in pixels
        self.battery_icon.setIconSize(icon_size)

        self.battery_icon.setIcon(QIcon("./public/battery-twotone-100-svgrepo-com.svg"))
        # self.battery_icon.setIconSize(50, 50)
        self.battery_icon.setText("100%")
        self.battery_icon.setStyleSheet("""
            font-size: 12px;
            font-weight: bold;
            color: #4CAF50;  /* Initial green color */
        """)

        self.status_label = QLabel("Cargada")
        self.status_label.setAlignment(Qt.AlignLeft)
        self.status_label.setStyleSheet("font-size: 14px; color: #666;")

        self.layout.addWidget(self.battery_icon)
        self.setLayout(self.layout)

        self.current_level = 100

    def map2percent(self, val):
        """Map a raw battery reading to a percentage (currently identity; the
        min/max rescale below is unreachable dead code)."""
        return val
        max = 1.1
        min = 0.99
        h = max - min
        return ((val - min) / h) * 100

    def battery_callback(self, msg):
        """ROS subscriber callback: update the stored battery percentage from a ``BatteryState`` msg."""
        self.percentage = self.map2percent(msg.percentage)
        # self.percentage = (self.percentage // 10) * 10
        self.percentage = (self.percentage) * 100
        self.percentage = (self.percentage // 10) * 10
        rospy.loginfo(f"Battery percentage: {self.percentage:.1f}%")

    def update_battery_state(self):
        """Timer tick: refresh the icon/label and color by battery level,
        triggering the one-shot critical-battery dialog/alert below 20%."""
        # 1. Round percentage to the nearest 10 for icon selection (e.g., 87% -> 80 or 90)
        # Assuming you have icons for 0, 10, 20... 100
        icon_level = int(round(self.percentage / 10.0)) * 10
        icon_level = max(0, min(100, icon_level))  # Clamp between 0-100

        # 2. Update Text and Icon
        self.battery_icon.setText(f"{self.percentage:.1f}%")
        icon_path = f"./public/battery-twotone-{icon_level}-svgrepo-com.svg"
        self.battery_icon.setIcon(QIcon(icon_path))

        # 3. Logic for Styles and Alerts
        if self.percentage < 20:
            # CRITICAL STATE: Red + Blinking effect (using the 30fps timer)
            # We use a simple modulo on a counter or timestamp to blink
            is_visible = (rospy.get_time() % 1.0) > 0.5
            color = "red" if is_visible else "transparent"

            self.battery_icon.setStyleSheet(
                f"font-size: 12px; font-weight: bold; color: {color};"
            )
            self.status_label.setText("CRÍTICO: Cargar ahora")
            self.status_label.setStyleSheet(
                "font-size: 14px; color: red; font-weight: bold;"
            )

            if self.showBatteryWarning:
                self.show_critical_dialog()
                self.showBatteryWarning = False
                self.battery_alert.emit("CRITICAL: Battery low!")

        elif self.percentage < 40:
            # LOW STATE: Orange/Yellow
            self.battery_icon.setStyleSheet(
                "font-size: 12px; font-weight: bold; color: #FFA500;"
            )
            self.status_label.setText("Batería Baja")
            self.status_label.setStyleSheet("font-size: 14px; color: #FFA500;")
            self.showBatteryWarning = True  # Reset for next drop

        else:
            # HEALTHY STATE: Green
            self.battery_icon.setStyleSheet(
                "font-size: 12px; font-weight: bold; color: #4CAF50;"
            )
            self.status_label.setText("Cargada")
            self.status_label.setStyleSheet("font-size: 14px; color: #666;")
            self.showBatteryWarning = True
            self.battery_alert.emit("")

    def show_critical_dialog(self):
        """Show the one-shot critical-battery confirmation dialog."""
        dg = CustomDialog(
            self,
            "Bateria en valor critico",
            message="Iniciando el modo de recuperación...",
            positive_response="Seguir en modo Auto",
            negative_response="Cambiar a Manual",
            retries=1,
        )
        dg.exec_()

    def odom_callback(self, msg):
        """Called every time a new odometry message arrives."""
        # Extract current position (x, y)
        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y

        # If this is the first message, just store the position and return
        if self.prev_x is None or self.prev_y is None:
            self.prev_x = curr_x
            self.prev_y = curr_y
            return

        # Compute displacement since last message
        dx = curr_x - self.prev_x
        dy = curr_y - self.prev_y
        displacement = math.sqrt(dx * dx + dy * dy)

        # Update accumulated distance
        self.accumulated_distance += displacement

        # Check if threshold has been reached (or exceeded)
        if self.accumulated_distance >= self.threshold:
            # Calculate how many full thresholds we passed
            # This handles large jumps (e.g., teleportation or fast movement)
            num_triggers = int(self.accumulated_distance // self.threshold)
            for _ in range(num_triggers):
                rospy.loginfo(
                    "Robot moved 10 cm (%.2f m total threshold crossed)", self.threshold
                )
            # Keep the remainder distance for next cycles
            self.accumulated_distance -= num_triggers * self.threshold

        # Update previous position for next callback
        self.prev_x = curr_x
        self.prev_y = curr_y


if __name__ == "__main__":
    app = QApplication(sys.argv)
    t = HomePanel()
    # t = PatrolsPanel()
    # t = VisualizationPanel()
    t.show()

    sys.exit(app.exec())
