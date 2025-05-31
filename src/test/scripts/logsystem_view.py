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
from database_manager import DataBase, InternalStorageManager

from styles.labels import (
    error_label_style,
    warning_label_style,
    info_label_style,
    title_label_style,
)

from styles.buttons import (
    tag_button_style,
    tag_selected_button_style,
    colored_button_style,
    primary_button_style,
    border_button_style
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
        nodes_manager=None,
    ):
        super().__init__(parent)
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.Popup)

        self.init_ui()

        self.zoom_factor = 1.0
        self.nodes_manager = nodes_manager
        self.parent = parent
        self.resolution = 0
        self.botton_left_map = (999, 999)
        self.pointsInMap = {}
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
        sc.axes.hist(data, bins=30, alpha=0.7, color="blue", edgecolor="black")

        layout.addWidget(sc)
        buttons_layout = QHBoxLayout()
        self.info_label = QLabel(
            "ℹ INFO: Para guardar los puntos de  manera permante da click en Guardar"
        )

        self.close_button = QPushButton("Cerrar")
        self.save_button = QPushButton("Guardar")

        buttons_layout.addWidget(self.save_button, 2)
        buttons_layout.addWidget(self.close_button, 1)
        layout.addLayout(buttons_layout)

        icon = QApplication.style().standardIcon(QStyle.SP_DialogCloseButton)
        self.close_button.setIcon(icon)

        central_widget.setLayout(layout)

    def show_win(self):
        # popup_x = self.parent.x() + (self.parent.width() - self.width()) // 2
        # popup_y = self.parent.y() + (self.parent.height() - self.height()) // 2
        popup_x = self.parent.x() + (self.parent.width() - self.width())
        # popup_y = self.parent.y() + (self.parent.height() - self.height() - 30)
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

    def init_ui(self):
        # Central widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout()
        gradientbar_layout = QHBoxLayout()
        title_label = QLabel("Mapa de calor de  indidencia de alertas")
        title_label.setStyleSheet(title_label_style)
        description_label = QLabel(
            "Mucho texto aburrido lloeknoknlenrr erokgorgne goergnerong"
        )

        label = QLabel("   hola gradinte jjjj ")
        gradientbar_layout.addWidget(QLabel("Mayor numero de incidencias"), 1)
        gradientbar_layout.addWidget(label, 5)
        gradientbar_layout.addWidget(QLabel("Menor numero de incidencias"), 1)
        label.setStyleSheet(
            """
                  QLabel {
                            background: qlineargradient(x1:0, y1:0, x2:1, y2:0,stop:0 red, stop:1 green);
                        }
                """
        )

        layout.addWidget(title_label)
        layout.addWidget(description_label)
        layout.addLayout(gradientbar_layout)
        buttons_layout = QHBoxLayout()
        self.graphics_view = Example(parent=self.parent)
        self.success_label = QLabel("✓ SUCCESS: File saved successfully")
        self.info_label = QLabel(
            "ℹ INFO: Para guardar los puntos de  manera permante da click en Guardar"
        )

        layout.addWidget(self.graphics_view)

        self.close_button = QPushButton("Cerrar")

        buttons_layout.addWidget(self.close_button, 1)
        layout.addLayout(buttons_layout)

        icon = QApplication.style().standardIcon(QStyle.SP_DialogCloseButton)
        self.close_button.setIcon(icon)

        central_widget.setLayout(layout)

    def show_win(self, file_path):
        self.graphics_view.display_image(file_path=file_path)
        # popup_x = self.parent.x() + (self.parent.width() - self.width()) // 2
        # popup_y = self.parent.y() + (self.parent.height() - self.height()) // 2
        popup_x = self.parent.x() + (self.parent.width() - self.width())
        # popup_y = self.parent.y() + (self.parent.height() - self.height() - 30)
        popup_y = self.parent.y()
        self.move(popup_x, popup_y)
        self.show()


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
    def __init__(self, text, parent=None, style=None):
        super().__init__(parent)
        self.parent = parent

        self.label = QLabel(text)

        self.label.setStyleSheet(style)
        layout = QHBoxLayout()
        layout.addWidget(self.label)

        self.setLayout(layout)


class DisplayInfo(QGroupBox):
    def __init__(self, parent) -> None:
        super().__init__()
        layout = QVBoxLayout(self)
        self.generate_heatmap_task = None 
        msg1 = " recopila estadísticas de diagnóstico relacionadas con la frecuencia y horarios  de los patrullajes del robot "
        msg2 = " recopila estadísticas de diagnóstico relacionadas con los  lugares donde se generan alertas en los patrullajes del robot "

        self.heatmap = ImageViewer(parent=parent)
        self.patrols_stats = PatrolStatsViewer(parent=parent)

        stats_title = QLabel("Estatdiaticas de Monitoreo")
        stats_title.setStyleSheet(title_label_style)
        stats_title.setFixedSize(400, 80)
        layout.addWidget(stats_title)

        self.checkpoints_stats = DescriptionContainer(
            title="Estatdiaticas de puntos de interes", msg=msg2
        )
        self.alerts_stats = DescriptionContainer(title="Estadisticas  de Alertas", msg=msg1)

        layout.addWidget(self.checkpoints_stats)
        layout.addWidget(self.alerts_stats)
        self.setLayout(layout)

        self.checkpoints_stats.load_button.clicked.connect(self.get_checkpoints_stats)
        self.alerts_stats.load_button.clicked.connect(self.task2)

    def get_checkpoints_stats(self, y):
        if self.generate_heatmap_task and self.generate_heatmap_task.isRunning():
            return

        self.checkpoints_stats.load_button.setText('Cargando...')
        self.checkpoints_stats.progress.show()
        self.checkpoints_stats.progress.setValue(0)
        self.checkpoints_stats.timer.start()

        self.generate_heatmap_task = TaskWorkerHeatmap()
        self.generate_heatmap_task.task_finished.connect(self.get_checkpoints_stats_finished)
        self.generate_heatmap_task.start()

    def get_checkpoints_stats_finished(self, filepath):
        self.checkpoints_stats.timer.stop()
        self.checkpoints_stats.progress.hide()

        self.generate_heatmap_task.quit()
        self.generate_heatmap_task.wait()
        self.generate_heatmap_task = None
        self.checkpoints_stats.load_button.setText('Generar Diagnostico')

        self.heatmap.show_win(filepath)


    def task2(self, y):
        self.patrols_stats.show_win()


class DescriptionContainer(QGroupBox):
    def __init__(self, title="Estatdiaticas", msg="nononono") -> None:
        super().__init__(title)
        self.setStyleSheet("""
            QGroupBox {
                font: 18px;
                color: gray;
            }
            
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
                color: black;
            }
        """)
        self.layout = QVBoxLayout()
        self.buttons_layout = QHBoxLayout()

        self.layout.addWidget(QLabel(msg))
        self.load_map_button = QPushButton("Seleccionar mapa")
        self.load_button = QPushButton("Generar Diagnostico")
        self.viewdata_button = QPushButton("Ver resultados")
        self.progress = QProgressBar()
        self.timer = QTimer()
        self.timer.setInterval(2000)

        self.timer.timeout.connect(self.updateProgress)
        self.load_map_button.clicked.connect(self.handleLoadMap)

        self.load_button.setStyleSheet(colored_button_style)
        self.load_map_button.setStyleSheet(border_button_style)
        self.progress.hide()

        self.layout.addWidget(self.progress)
        self.layout.addWidget(self.load_map_button)
        self.buttons_layout.addWidget(self.load_button)
        self.buttons_layout.addWidget(self.viewdata_button)
        self.layout.addLayout(self.buttons_layout)
        self.setLayout(self.layout)

    def updateProgress(self):
        self.progress.setValue(self.progress.value() + 1)

    def handleLoadMap(self, k):
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Open Image File", "", "Image Files (*.yaml)"
        )
        print(file_path)

class TaskWorkerHeatmap(QThread):
    task_finished = pyqtSignal(str)

    def __init__(self, data=[]) -> None:
        super().__init__()

    def run(self):
        ma = InternalStorageManager()
        data = ma.get_alerts_error()
        heatmapgenerator = HeatmapGenerator()
        filepath = heatmapgenerator.generate(data=data)
        print(filepath)
        self.task_finished.emit(filepath)
        pass


class LogPanel(QWidget):
    def __init__(self, node_manager=None, parent=None):
        super().__init__()
        self.layout = QGridLayout(self)
        self.main_layout = QVBoxLayout()
        self.setup_filter_buttons()
        self.setup_todo_list()
        self.panel = DisplayInfo(parent=parent)
        self.layout.addLayout(self.main_layout, 2, 0)
        self.toggle = True
        self.node_manager = node_manager
        self.alerts_items = []
        self.filters_state = {
            "error": (False, 20),
            "warning": (False, 20),
            "info": (False, 20),
            "all": (True, 60),
        }
        # rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.poseMonitor)
        # rospy.Subscriber('/scan', LaserScan,  self.obstacleMonitor)
        # rospy.Subscriber('/imu', Imu,  self.imuMonitor)

        self.layout.addWidget(self.panel, 2, 1, 1, 2)
        self.layout.addWidget(QLabel("Filtras las alestar como prefieras"), 0, 0)

    def poseMonitor(self, msg):
        pass

    def obstacleMonitor(self, msg):
        pass

    def imuMonitor(self, msg):
        pass

    def setup_filter_buttons(self):
        input_layout = QHBoxLayout()

        filter_bydate_button = QPushButton("Recinetes-Antiguos")
        filter_bytAll_button = QPushButton("All 200")
        filter_byErrors_button = QPushButton("Errors 100")
        filter_byWarnings_button = QPushButton("warning 50")
        filter_byInfos_button = QPushButton("Info 50")

        filter_bytAll_button.setStyleSheet(tag_button_style)
        filter_byErrors_button.setStyleSheet(tag_button_style)
        filter_byInfos_button.setStyleSheet(tag_button_style)
        filter_byWarnings_button.setStyleSheet(tag_button_style)
        filter_bydate_button.setStyleSheet(tag_button_style)

        filter_bydate_button.clicked.connect(self.filterAlerts)
        filter_byWarnings_button.clicked.connect(self.filterAlerts)
        filter_byInfos_button.clicked.connect(self.filterAlerts)
        filter_byErrors_button.clicked.connect(self.filterAlerts)
        filter_bytAll_button.clicked.connect(self.filterAlerts)

        input_layout.addWidget(filter_bytAll_button)
        input_layout.addWidget(filter_byErrors_button)
        input_layout.addWidget(filter_byWarnings_button)
        input_layout.addWidget(filter_byInfos_button)
        input_layout.addWidget(filter_bydate_button)

        self.layout.addLayout(input_layout, 1, 0, 1, 3)

    def setup_todo_list(self):
        self.alerts_container = QWidget()
        self.alerts_layout = QVBoxLayout(self.alerts_container)
        self.alerts_layout.setSpacing(5)
        self.alerts_layout.addStretch()
        btn = QPushButton("Mostrar mas")
        btn.setStyleSheet(primary_button_style)
        btn.clicked.connect(self.handleShowMoreAlerts)

        self.main_layout.addWidget(self.alerts_container)
        self.main_layout.addWidget(btn)

    def add_todo(self):
        for i in range(13):
            text = "ERROR: Obstaculo encontrado 1/23/2025 23:23"
            if text:
                style = random.choice(
                    [error_label_style, warning_label_style, info_label_style]
                )
                todo_item = AlertItem(text, self, style=style)
                self.alerts_layout.insertWidget(
                    self.alerts_layout.count() - 1, todo_item
                )
                self.alerts_items.append(todo_item)

    def remove_todo_item(self):
        for item in self.alerts_items:
            self.alerts_layout.removeWidget(item)
            item.deleteLater()
        self.alerts_items = []

    def handleShowMoreAlerts(self, u):
        self.remove_todo_item()
        self.add_todo()

    def filterAlerts(self, filter):
        print("ffffffffffffffffffffffffffffffff")
        pass


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = LogPanel()
    window.show()
    sys.exit(app.exec_())
