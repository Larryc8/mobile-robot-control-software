import sys
import matplotlib
from collections import namedtuple
import numpy as np

matplotlib.use("Qt5Agg")
from matplotlib.backends.backend_qt5agg import (
    FigureCanvasQTAgg,
    NavigationToolbar2QT as NavigationToolbar,
)
from matplotlib.figure import Figure

from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

import rospy

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QGraphicsView,
    QGraphicsScene,
    QFileDialog,
    QVBoxLayout,
    QWidget,
    QPushButton,
    QGraphicsItem,
    QHBoxLayout,
    QStyle,
    QLabel,
    QGroupBox,
    QCheckBox,
    QGridLayout,
    QComboBox,
    QProgressBar,
    QSlider,
    QStackedLayout
)
from PyQt5.QtGui import (
    QPixmap,
    QImage,
    QPainter,
    QPainterPath,
    QPen,
    QBrush,
    QColor,
    QFont,
)

from PyQt5.QtCore import (
    Qt,
    QPropertyAnimation,
    QEasingCurve,
    QTimer,
    QThread,
    pyqtSignal,
)
from PyQt5.QtGui import QPixmap
import random

from rview import MyViz
from better_image_display import ImageViewer
from heatmap_generator import HeatmapGenerator
from database_manager import DataBase, InternalStorageManager, AlertStatus
from points_generator import CheckPointDisplay

from styles.labels import (
    error_label_style,
    warning_label_style,
    info_label_style,
    title_label_style,
    normal_label_style,
    muted_label_style,
    succes_label_style,
    muted_mini_label_style,
    section_header_label_style
)

from styles.buttons import (
    tag_button_style,
    tag_selected_button_style,
    tag_selected_button_style,
    colored_button_style,
    primary_button_style,
    border_button_style,
    secondary_button_style,
)


class MplCanvas(FigureCanvasQTAgg):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super().__init__(fig)


class PatrolStatsViewer(QMainWindow):
    def __init__(
        self,
        parent=None,
    ):
        super().__init__(parent)
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.Popup)
        self.parent = parent

        self.init_ui()

        # self.save_selected_points.connect(self.)
        self.setGeometry(
            self.parent.x(),
            self.parent.y(),
            self.parent.width() + 100,
            self.parent.height() + 200,
        )

    def init_ui(self):
        # Central widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout()
        sc = MplCanvas(self, width=5, height=4, dpi=100)
        data = np.random.randn(1000)
        sc.axes.hist([1, 2, 3, 1],  bins=30, alpha=0.7, color="blue", edgecolor="black", label='Sine')
        sc.axes.hist([1, 0, 7, 1],  bins=30, alpha=0.7, color="red", edgecolor="black", label='Sineaaa')
        sc.axes.hist([1, 4, 1, 1],  bins=30, alpha=0.7, color="green", edgecolor="black", label='Sineddd')
        sc.axes.legend(title='Functions', framealpha=1, shadow=False)
        sc.axes.set_title('Function Comparison with Custom Colors')

        layout.addWidget(sc)
        buttons_layout = QHBoxLayout()
        self.info_label = QLabel(
            "ℹ INFO: Para guardar los puntos de  manera permante da click en Guardar"
        )

        self.close_button = QPushButton("Cerrar")
        self.save_button = QPushButton("Guardar")

        self.close_button.setStyleSheet(primary_button_style)

        # buttons_layout.addWidget(self.save_button, 2)
        buttons_layout.addWidget(self.close_button, 1)
        layout.addLayout(buttons_layout)

        icon = QApplication.style().standardIcon(QStyle.SP_DialogCloseButton)
        self.close_button.setIcon(icon)

        central_widget.setLayout(layout)

    def show_win(self):
        popup_x = self.parent.x() + (self.parent.width() - self.width())
        popup_y = self.parent.y()
        self.move(popup_x, popup_y)
        self.show()


# import pyqtgraph as pg
class ImageViewer(QMainWindow):
    def __init__(
        self,
        ylm_map_filepath=None,
        parent=None,
        nodes_manager=None,
        patrols_scheduler=None,
    ):
        super().__init__(parent)
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.Popup)

        self.init_ui()

        self.zoom_factor = 1.0
        self.nodes_manager = nodes_manager
        self.parent = parent
        self.setGeometry(
            self.parent.x(),
            self.parent.y(),
            self.parent.width() + 100,
            self.parent.height() + 200,
        )
        self.setStyleSheet("QMainWindow { background-color: white;}")

    def init_ui(self):
        # Central widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout()
        layout.setSpacing(20)
        layout.setContentsMargins(50, 50, 50, 10)

        gradientbar_layout = QHBoxLayout()
        title_label = QLabel("Mapa de calor de incidencia de alertas")
        title_label.setStyleSheet(title_label_style)
        description_label = QLabel(
            "En este mapa de calor de reprsentan las alertas de tipo error asociadas a mal funcionamiento de los puntos de interes e inpedimeintos para la ejecucion normal de los patrullajes del robot"
        )

        label = QLabel("    ")
        gradient_min_label = QLabel("Mayor numero \nde incidencias")
        gradient_max_label = QLabel("Menor numero \nde incidencias")

        gradientbar_layout.addWidget(gradient_min_label, 1)
        gradientbar_layout.addWidget(label, 5)
        gradientbar_layout.addWidget(gradient_max_label, 1)
        data_info = QLabel("Los calculos se basan en los ultimos 30 dias")
        self.close_button = QPushButton("Cerrar")

        description_label.setStyleSheet(normal_label_style)
        gradient_max_label.setStyleSheet(muted_label_style)
        gradient_min_label.setStyleSheet(muted_label_style)
        data_info.setStyleSheet(muted_label_style)
        label.setStyleSheet(
            """
                  QLabel {
                            background: qlineargradient(x1:0, y1:0, x2:1, y2:0,stop:0 red, stop:1 green);
                            padding: 5px
                        }
                """
        )
        self.close_button.setStyleSheet(primary_button_style)

        self.close_button.clicked.connect(self.close_win)

        layout.addWidget(title_label)
        layout.addWidget(description_label)
        layout.addLayout(gradientbar_layout)
        layout.addWidget(data_info)
        buttons_layout = QHBoxLayout()
        self.graphics_view = Example(parent=self.parent)
        self.success_label = QLabel("✓ SUCCESS: File saved successfully")
        self.info_label = QLabel(
            "ℹ INFO: Para guardar los puntos de  manera permante da click en Guardar"
        )

        layout.addWidget(self.graphics_view)

        buttons_layout.addWidget(self.close_button, 1)
        layout.addLayout(buttons_layout)

        icon = QApplication.style().standardIcon(QStyle.SP_DialogCloseButton)
        self.close_button.setIcon(icon)

        central_widget.setLayout(layout)

    def show_win(self, file_path):
        self.graphics_view.display_image(file_path=file_path)
        popup_x = self.parent.x() + (self.parent.width() - self.width())
        popup_y = self.parent.y()
        self.move(popup_x, popup_y)
        self.show()

    def close_win(self, e):
        self.hide()


class Example(QGraphicsView):
    def __init__(self, parent):
        super().__init__()
        self.wheel_delta = 0
        self.zoom_factor = 1
        self.pixmap = None
        self.parent = parent
        self.setMouseTracking(True)
        self.setInteractive(True)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        self.setRenderHint(QPainter.Antialiasing)
        self.setRenderHint(QPainter.SmoothPixmapTransform)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setStyleSheet("QGraphicsView {border: 2px solid gray;}")

    def zoom_in(self):
        self.scale(1.1, 1.1)
        self.zoom_factor *= 1.1
        self.update()

    def zoom_out(self):
        self.scale(1 / 1.1, 1 / 1.1)
        self.zoom_factor /= 1.1
        self.update()

    def reset_view(self):
        # if self.scene.items():
        self.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)
        self.zoom_factor = 1.0

    def wheelEvent(self, event):
        # Get the wheel delta (positive for up, negative for down)
        delta = event.angleDelta().y()
        # Accumulate the total wheel movement
        self.wheel_delta += delta

        if delta > 0 and self.zoom_factor > 1:
            self.zoom_out()
        if delta < 0 and self.zoom_factor < 80:
            self.zoom_in()

        event.accept()

    def display_image(self, file_path):  # ghghghghhg
        self.scene = QGraphicsScene()
        self.setScene(self.scene)
        self.scene.clear()

        self.pixmap = QPixmap(file_path)
        if self.pixmap.isNull():
            print("Failed to load image")
            return

        self.scene.addPixmap(self.pixmap)
        self.reset_view()


class AlertItem(QWidget):
    show_details = pyqtSignal(dict)
    def __init__(self, text, parent=None, style=None):
        super().__init__(parent)
        self.parent = parent

        self.label = QLabel(text)

        self.label.setStyleSheet(style)
        layout = QHBoxLayout()
        layout.addWidget(self.label)

        self.setLayout(layout)

    # def mouseClickEvent(self, event):
    def mousePressEvent(self, event):
        print('CLicked')
        self.show_details.emit({})


class InfoPanel(QGroupBox):
    map_loaded = pyqtSignal(str)

    def __init__(self, parent) -> None:
        super().__init__()
        self.main_layout = QStackedLayout()
        self.stats_panel = QWidget()
        self.alert_details = AlertDetails()
        layout = QVBoxLayout(self)

        self.setStyleSheet("""
        QGroupBox {
            background-color: white;
        }""")
        self.generate_heatmap_task = None
        self.generate_alerts_stats_task = None
        msg3 = (
            '. Seleccione un mapa de la carpeta de "maps", y genere un \nestadisticas de inspección.'
        )
        msg1 = "Recopila estadísticas de diagnóstico relacionadas con la frecuencia y horarios  de los patrullajes del\nrobot asociadas a un mapa  "
        msg1 = msg1 + msg3
        msg2 = "Recopila estadísticas de diagnóstico relacionadas con los  lugares donde se generan alertas en los\npatrullajesdel robot asociadas a un mapa"
        msg2 = msg2 + msg3

        self.heatmap = ImageViewer(parent=parent)
        self.heatmap_img_filepath = None
        self.patrols_stats = PatrolStatsViewer(parent=parent)

        map_filepath_label = QLabel('x')
        map_filepath_label.hide()
        stats_title_image = QLabel("Estatdiaticas de Monitoreo")
        pixmap = QPixmap("./stats.png").scaled(320, 300)
        stats_title_image.setPixmap(pixmap)

        self.checkpoints_stats = StatContainer(
            title="Estadisticas de puntos de interés", msg=msg2
        )
        self.alerts_stats = StatContainer(
            title="Estadisticas  de Alertas", msg=msg1
        )

        layout.addWidget(map_filepath_label)
        layout.addWidget(stats_title_image, alignment=Qt.AlignCenter)
        layout.addWidget(self.checkpoints_stats, alignment=Qt.AlignCenter)
        layout.addWidget(self.alerts_stats, alignment=Qt.AlignCenter)

        self.checkpoints_stats.generate_stats.connect(self.get_checkpoints_stats)
        self.alerts_stats.map_loaded.connect(self.set_map)
        self.alerts_stats.generate_stats.connect(self.get_alerts_stats)
        self.alert_details.back2stats_button.clicked.connect(self.go2stats)

        self.checkpoints_stats.viewdata_button.clicked.connect(self.show_heatmap)

        self.stats_panel.setLayout(layout)
        self.main_layout.addWidget(self.stats_panel)
        self.main_layout.addWidget(self.alert_details)


        self.setLayout(self.main_layout)

    def go2stats(self):
        self.main_layout.setCurrentIndex(0)

    def go2alert_details(self):
        self.main_layout.setCurrentIndex(1)

    def handleShowAlertInfo(self, info_data):
        self.main_layout.setCurrentIndex(1)



    def get_checkpoints_stats(self, map_filepath):
        if self.generate_heatmap_task and self.generate_heatmap_task.isRunning():
            return

        self.checkpoints_stats.generate_stats_button.setText("Cargando...")
        self.checkpoints_stats.progress.show()
        self.checkpoints_stats.progress.setValue(0)
        self.checkpoints_stats.timer.start()

        self.generate_heatmap_task = TaskWorkerHeatmap(map=map_filepath)
        self.generate_heatmap_task.task_finished.connect(
            self.get_checkpoints_stats_finished
        )
        self.generate_heatmap_task.start()

    def get_checkpoints_stats_finished(self, filepath):
        self.checkpoints_stats.timer.stop()
        self.checkpoints_stats.progress.hide()
        self.checkpoints_stats.success_label.show()

        self.generate_heatmap_task.quit()
        self.generate_heatmap_task.wait()
        self.checkpoints_statcheckpoints_stats = None
        self.checkpoints_stats.generate_stats_button.setText("Cargar mapa")
        self.checkpoints_stats.isUploadMap = True
        self.heatmap_img_filepath = filepath


    def show_heatmap(self, m):
        if self.heatmap_img_filepath:
            self.heatmap.show_win(self.heatmap_img_filepath)

    def set_map(self, map_filepath):
        self.map_loaded.emit(map_filepath)
        print('map loaded Logsistem')
        self.patrols_stats.show_win()

    def get_alerts_stats(self,map_filepath):
        if self.generate_alerts_stats_task and self.generate_alerts_stats_task.isRunning():
            return

        self.alerts_stats.generate_stats_button.setText("Cargando...")
        self.alerts_stats.progress.show()
        self.alerts_stats.progress.setValue(0)
        self.alerts_stats.timer.start()

        self.generate_alerts_stats_task = TaskWorkerAlerts(map=map_filepath)
        self.generate_alerts_stats_task.task_finished.connect(
            self.get_checkpoints_stats_finished
        )
        self.generate_alerts_stats_task.start()

    def get_alerts_stats_finished(self):
        self.alerts_stats.timer.stop()
        self.alerts_stats.progress.hide()
        self.alerts_stats.success_label.show()

        # self.generate_heatmap_task.quit()
        # self.generate_heatmap_task.wait()
        # self.generate_heatmap_task = None
        # self.checkpoints_stats.generate_stats_button.setText("Cargar mapa")
        # self.checkpoints_stats.isUploadMap = True
        # self.heatmapimg_filepath = filepath




class AlertDetails(QGroupBox):
    def __init__(self) -> None:
        super().__init__()
        self.layout = QVBoxLayout()
        self.title_label = QLabel('ERROR: EL epepe etete')
        self.datetime_label = QLabel('1234-23-24')
        self.back2stats_button = QPushButton('Volver a Estadisticas')
        self.point =   CheckPointDisplay()

        self.title_label.setStyleSheet(title_label_style)
        self.datetime_label.setStyleSheet(section_header_label_style)

        self.layout.addWidget(self.title_label)
        self.layout.addWidget(self.datetime_label)
        self.layout.addWidget(self.point, 3)
        self.layout.addWidget(self.back2stats_button)
        self.setLayout(self.layout)


class StatContainer(QGroupBox):
    generate_stats = pyqtSignal(str)
    map_loaded = pyqtSignal(str)

    def __init__(self, title="Estatdiaticas", msg="nononono") -> None:
        super().__init__(title)
        self.setStyleSheet("""
            QGroupBox {
                font: 18px;
                color: gray;
                background-color: white;
            }
            
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
                color: black;
            }
        """)
        self.setFixedSize(800, 200)
        self.layout = QVBoxLayout()
        self.buttons_layout = QHBoxLayout()

        msg_label = QLabel(msg)
        msg_label.setStyleSheet(muted_label_style)
        self.layout.addWidget(msg_label)
        self.generate_stats_button = QPushButton("Cargar Mapa")
        self.viewdata_button = QPushButton("Ver resultados")
        self.progress = QProgressBar()
        self.success_label = QLabel("Resultados generados con exito")
        self.success_label.setStyleSheet(succes_label_style)
        self.timer = QTimer()
        self.timer.setInterval(2000)
        self.isUploadMap = True
        self.map_filepath = None

        self.timer.timeout.connect(self.updateProgress)

        self.generate_stats_button.setStyleSheet(colored_button_style)
        self.viewdata_button.setStyleSheet(secondary_button_style)
        self.progress.hide()
        self.success_label.hide()

        self.generate_stats_button.clicked.connect(self.toggle)

        self.layout.addWidget(self.progress)
        self.layout.addWidget(self.success_label)
        self.buttons_layout.addWidget(self.generate_stats_button)
        self.buttons_layout.addWidget(self.viewdata_button)
        self.layout.addLayout(self.buttons_layout)
        self.setLayout(self.layout)

    def toggle(self):
        if self.isUploadMap:
            self.map_filepath = self.handleLoadMap()
            self.isUploadMap = False
            self.generate_stats_button.setText("Generar estadísticas")
            self.success_label.hide()
            return

        self.generate_stats.emit(self.map_filepath)
        pass

    def updateProgress(self):
        self.progress.setValue(self.progress.value() + 1)

    def handleLoadMap(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Open Image File", "", "Image Files (*.yaml)"
        )
        print(file_path)
        self.map_loaded.emit(file_path)
        return file_path


class TaskWorkerHeatmap(QThread):
    task_finished = pyqtSignal(str)

    def __init__(self, data=[], map: str = "") -> None:
        super().__init__()
        self.map = map

    def run(self):
        ma = InternalStorageManager()
        data = ma.get_alerts_error()
        heatmapgenerator = HeatmapGenerator(file_path=self.map)
        heatmapimg_filepath = heatmapgenerator.generate(data=data)
        print(heatmapimg_filepath)
        self.task_finished.emit(heatmapimg_filepath)
        pass

class TaskWorkerAlerts(QThread):
    task_finished = pyqtSignal(str)

    def __init__(self, data=[], map: str = "") -> None:
        super().__init__()
        self.map = map

    def run(self):
        ma = InternalStorageManager()
        # data = ma.get_alerts_error()
        # self.task_finished.emit(heatmapimg_filepath)


class TaskWorkerFilterByTypeAlerts(QThread):
    filter_bytype_finished = pyqtSignal(list)

    def __init__(self, status, ascendant, page_number, page_size, map) -> None:
        super().__init__()
        self.page_number = page_number
        self.page_size = page_size
        self.ascendant = ascendant
        self.status = status
        self.map = map

    def run(self):
        ma = InternalStorageManager()
        data = ma.get_filtered_alerts(
            status=self.status,
            ascendant=self.ascendant,
            page_size=self.page_size,
            page_number=self.page_number,
            map=self.map
        )
        if data:
            self.filter_bytype_finished.emit(data)
            return

        self.filter_bytype_finished.emit([])


class LogPanel(QWidget):
    def __init__(self, node_manager=None, parent=None):
        super().__init__()
        self.layout = QGridLayout(self)
        self.main_layout = QHBoxLayout()
        self.setup_filter_buttons()
        self.setup_alert_list()
        self.info_panel = InfoPanel(parent=parent)
        self.layout.addLayout(self.main_layout, 2, 0)
        self.toggle = True
        self.node_manager = node_manager
        self.alerts_items = []
        self.alerts = []
        self.page_number = 0
        self.filter_bytype_task = None
        self.status = None
        self.ascendant = False
        self.PAGE_ZISE = 12
        self.map = None

        # self.getAlerts()

        # rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.poseMonitor)
        # rospy.Subscriber('/scan', LaserScan,  self.obstacleMonitor)
        # rospy.Subscriber('/imu', Imu,  self.imuMonitor)

        self.layout.addWidget(self.info_panel, 2, 1, 2, 2)
        info_label = QLabel("Filtras alertas por:")
        info_label.setStyleSheet(normal_label_style)
        self.layout.addWidget(info_label, 0, 0)

        self.info_panel.map_loaded.connect(self.set_map)

    def poseMonitor(self, msg):
        pass

    def obstacleMonitor(self, msg):
        pass

    def imuMonitor(self, msg):
        pass

    def setup_filter_buttons(self):
        input_layout = QHBoxLayout()

        self.filter_bydate_button = QPushButton("⬆ Ascendente")
        self.filter_bytAll_button = QPushButton("Todos")
        self.filter_byErrors_button = QPushButton("✗ Errores")
        self.filter_byWarnings_button = QPushButton("☢ Advertencia")
        self.filter_byInfos_button = QPushButton("✔ Informativas")

        self.filter_bytAll_button.setStyleSheet(tag_selected_button_style)
        self.filter_byErrors_button.setStyleSheet(tag_button_style)
        self.filter_byInfos_button.setStyleSheet(tag_button_style)
        self.filter_byWarnings_button.setStyleSheet(tag_button_style)
        self.filter_bydate_button.setStyleSheet(secondary_button_style)

        self.filter_bydate_button.clicked.connect(self.toggleAscendantDescendant)
        self.filter_byWarnings_button.clicked.connect(self.filterAlertsWarning)
        self.filter_byInfos_button.clicked.connect(self.filterAlertsInfo)
        self.filter_byErrors_button.clicked.connect(self.filterAlertsError)
        self.filter_bytAll_button.clicked.connect(self.filterAlertsAll)

        input_layout.addWidget(self.filter_bytAll_button)
        input_layout.addWidget(self.filter_byErrors_button)
        input_layout.addWidget(self.filter_byWarnings_button)
        input_layout.addWidget(self.filter_byInfos_button)
        order_label = QLabel("Ordenar alertas de forma:")
        order_label.setStyleSheet(muted_mini_label_style)
        input_layout.addWidget(order_label)
        input_layout.addWidget(self.filter_bydate_button)


        self.info_label = QLabel('Carge un mapa desde Estadisticas de alertas.\n  Para ver las alertas generadas en el mapa')
        self.layout.addWidget(self.info_label, 2, 0, alignment=Qt.AlignCenter)
        self.info_label.setStyleSheet(muted_label_style)

        self.layout.addLayout(input_layout, 1, 0, 1, 3)

    def setup_alert_list(self):
        self.alerts_container = QWidget()
        self.alerts_layout = QVBoxLayout(self.alerts_container)
        self.alerts_layout.setSpacing(5)
        self.alerts_layout.addStretch()
        # self.info_label = QLabel('Carge un mapa desde Estadisticas de alertas')
        # self.alerts_layout.addWidget(self.info_label, alignment=Qt.AlignCenter)
        self.showmore_button = QPushButton("Mostrar mas")

        self.showmore_button.setStyleSheet(primary_button_style)
        self.showmore_button.clicked.connect(self.handleShowMoreAlerts)
        self.scroll_slider = QSlider(Qt.Vertical)

        self.main_layout.addWidget(self.alerts_container)
        self.main_layout.addWidget(self.scroll_slider)
        self.layout.addWidget(self.showmore_button, 3, 0)

    def add_alerts(self, alerts=[]):
        for alert in alerts:
            x_position, y_position, status, date, time = alert
            text = f"ERROR: Obstaculo encontrado {date} {time}"
            if text:
                # style = random.choice(
                #     [error_label_style, warning_label_style, info_label_style]
                # )
                if status == AlertStatus.ERROR.value:
                    style = error_label_style
                elif status == AlertStatus.WARNING.value:
                    style = warning_label_style
                else:
                    style = info_label_style

                todo_item = AlertItem(text, self, style=style)
                self.alerts_layout.insertWidget(
                    self.alerts_layout.count() - 1, todo_item
                )
                todo_item.show_details.connect(self.info_panel.handleShowAlertInfo)
                self.alerts_items.append(todo_item)

    def remove_alert_item(self):
        for item in self.alerts_items:
            self.alerts_layout.removeWidget(item)
            item.deleteLater()
        self.alerts_items = []

    def handleShowMoreAlerts(self, u):
        # self.remove_todo_item()
        # self.add_todo()
        if not self.map:
            return

        self.showmore_button.setText("Mostrar mas")

        self.page_number = self.page_number + 1
        self.getAlerts()

    def filterAlertsAll(self, filter):
        if not self.map:
            return

        self.status = None
        self.getAlerts()
        self.filter_bytAll_button.setStyleSheet(tag_selected_button_style)
        self.filter_byErrors_button.setStyleSheet(tag_button_style)
        self.filter_byInfos_button.setStyleSheet(tag_button_style)
        self.filter_byWarnings_button.setStyleSheet(tag_button_style)
        # self.filter_bydate_button.setStyleSheet(tag_button_style)

    def filterAlertsError(self):
        if not self.map:
            return
        self.status = AlertStatus.ERROR.value
        self.getAlerts()
        self.filter_bytAll_button.setStyleSheet(tag_button_style)
        self.filter_byErrors_button.setStyleSheet(tag_selected_button_style)
        self.filter_byInfos_button.setStyleSheet(tag_button_style)
        self.filter_byWarnings_button.setStyleSheet(tag_button_style)
        # self.filter_bydate_button.setStyleSheet(tag_button_style)

    def filterAlertsWarning(self):
        if not self.map:
            return
        self.status = AlertStatus.WARNING.value
        self.getAlerts()
        self.filter_bytAll_button.setStyleSheet(tag_button_style)
        self.filter_byErrors_button.setStyleSheet(tag_button_style)
        self.filter_byInfos_button.setStyleSheet(tag_button_style)
        self.filter_byWarnings_button.setStyleSheet(tag_selected_button_style)
        # self.filter_bydate_button.setStyleSheet(tag_button_style)

    def filterAlertsInfo(self):
        if not self.map:
            return
        self.status = AlertStatus.INFO.value
        self.getAlerts()
        self.filter_bytAll_button.setStyleSheet(tag_button_style)
        self.filter_byErrors_button.setStyleSheet(tag_button_style)
        self.filter_byInfos_button.setStyleSheet(tag_selected_button_style)
        self.filter_byWarnings_button.setStyleSheet(tag_button_style)
        # self.filter_bydate_button.setStyleSheet(tag_button_style)

    def handleTaskFinish(self, data):
        self.filter_bytype_task.quit()
        self.filter_bytype_task.wait()
        self.filter_bytype_task = None
        self.remove_alert_item()
        self.add_alerts(data)

        if self.ascendant:
            self.filter_bydate_button.setText("⬆ Ascendente")
        else:
            self.filter_bydate_button.setText("⬇ Descendente")

        if len(data) == 0:
            self.page_number = 0
            self.showmore_button.setText("Volver al principio")

    def getAlerts(self):
        if self.filter_bytype_task and self.filter_bytype_task.isRunning():
            return

        self.filter_bytype_task = TaskWorkerFilterByTypeAlerts(
            status=self.status,
            ascendant=self.ascendant,
            page_number=self.page_number,
            page_size=self.PAGE_ZISE,
            map=self.map
        )

        self.filter_bytype_task.filter_bytype_finished.connect(self.handleTaskFinish)
        self.filter_bytype_task.start()

    def toggleAscendantDescendant(self):
        if not self.map:
            return

        if self.ascendant:
            self.ascendant = False
            self.getAlerts()
        else:
            self.ascendant = True
            self.getAlerts()

    def set_map(self, map_filepath):
        self.info_label.hide()
        self.map = map_filepath
        print('Log system set map ')
        self.getAlerts()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = LogPanel()
    window.show()
    sys.exit(app.exec_())
