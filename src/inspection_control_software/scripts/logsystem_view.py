import sys
from collections import namedtuple

import matplotlib
import numpy as np

matplotlib.use("Qt5Agg")
import random

import rospy

# from better_image_display import ImageViewer
from database_manager import AlertStatus, DataBase, InternalStorageManager
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from heatmap_generator import HeatmapGenerator
from matplotlib.backends.backend_qt5agg import (
    FigureCanvasQTAgg,
)
from matplotlib.backends.backend_qt5agg import (
    NavigationToolbar2QT as NavigationToolbar,
)
from matplotlib.figure import Figure
from PyQt5.QtCore import (
    QDate,
    QEasingCurve,
    QPropertyAnimation,
    Qt,
    QThread,
    QTimer,
    pyqtSignal,
)
from PyQt5.QtGui import (
    QBrush,
    QColor,
    QFont,
    QImage,
    QPainter,
    QPainterPath,
    QPen,
    QPixmap,
)
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDateEdit,
    QFileDialog,
    QGraphicsItem,
    QGraphicsScene,
    QGraphicsView,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QProgressBar,
    QPushButton,
    QSlider,
    QStackedLayout,
    QStyle,
    QVBoxLayout,
    QWidget,
)
from rview import MyViz
from sensor_msgs.msg import Imu, LaserScan
from styles.buttons import (
    border_button_style,
    colored_button_style,
    primary_button_style,
    secondary_button_style,
    tag_button_style,
    tag_selected_button_style,
)
from styles.labels import (
    error_label_style,
    info_label_style,
    muted_label_style,
    muted_mini_label_style,
    normal_label_style,
    section_header_label_style,
    succes_label_style,
    title_label_style,
    warning_label_style,
)


class MplCanvas(FigureCanvasQTAgg):
    def __init__(self, parent=None, width=5, height=2, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super().__init__(fig)


class AlertHistogram(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.layout = QVBoxLayout(self)
        self.canvas = MplCanvas(self, width=10, height=3, dpi=100)
        self.layout.addWidget(self.canvas)
        # self.canvas.axes.set_title("Frecuencia de Alertas")
        # self.canvas.axes.set_xlabel("Tiempo")
        # self.canvas.axes.set_ylabel("Cantidad")
        self.setFixedHeight(210)

    def update_plot(self, labels, values):
        self.canvas.axes.clear()
        self.canvas.axes.bar(labels, values, color="#3498db", alpha=0.7)
        # self.canvas.axes.set_title("Frecuencia de Alertas")
        # self.canvas.axes.set_xlabel("Tiempo")
        # self.canvas.axes.set_ylabel("Cantidad")
        # self.canvas.axes.tick_params(axis="x", rotation=45)
        # self.canvas.figure.tight_layout()
        self.canvas.draw()
        self.hide()
        self.show()


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
        sc.axes.hist(
            [1, 2, 3, 1],
            bins=30,
            alpha=0.7,
            color="blue",
            edgecolor="black",
            label="Sine",
        )
        sc.axes.hist(
            [1, 0, 7, 1],
            bins=30,
            alpha=0.7,
            color="red",
            edgecolor="black",
            label="Sineaaa",
        )
        sc.axes.hist(
            [1, 4, 1, 1],
            bins=30,
            alpha=0.7,
            color="green",
            edgecolor="black",
            label="Sineddd",
        )
        sc.axes.legend(title="Functions", framealpha=1, shadow=False)
        sc.axes.set_title("Function Comparison with Custom Colors")

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
class ImageViewer(QWidget):
    on_close = pyqtSignal(bool)

    def __init__(
        self,
        ylm_map_filepath=None,
        parent=None,
        nodes_manager=None,
        patrols_scheduler=None,
    ):
        super().__init__(parent)
        # self.setWindowFlags(Qt.FramelessWindowHint | Qt.Popup)

        self.init_ui()

        self.zoom_factor = 1.0
        self.nodes_manager = nodes_manager
        self.parent = parent
        # self.setGeometry(
        #     self.parent.x(),
        #     self.parent.y(),
        #     self.parent.width() + 100,
        #     self.parent.height() + 200,
        # )
        self.setStyleSheet("QMainWindow { background-color: white;}")

    def init_ui(self):
        # Central widget and layout
        # central_widget = QWidget()
        # self.setCentralWidget(central_widget)
        layout = QVBoxLayout()
        # layout.setSpacing(20)
        # layout.setContentsMargins(50, 50, 50, 10)

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

        # layout.addWidget(title_label)
        # layout.addWidget(description_label)
        layout.addLayout(gradientbar_layout)
        # layout.addWidget(data_info)
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

        self.setLayout(layout)

    def show_win(self, file_path):
        self.graphics_view.display_image(file_path=file_path)
        # popup_x = self.parent.x() + (self.parent.width() - self.width())
        # popup_y = self.parent.y()
        # self.move(popup_x, popup_y)
        # self.show()

    def close_win(self, e):
        print("Close Heatmap")
        self.on_close.emit(True)


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

    def __init__(self, text, alert_data, parent=None, style=None):
        super().__init__(parent)
        self.parent = parent
        self.text = text
        self.alert_data = alert_data

        self.label = QLabel(text)

        self.label.setStyleSheet(style)
        layout = QHBoxLayout()
        layout.addWidget(self.label)

        self.setLayout(layout)

    def mousePressEvent(self, event):
        (
            x_position,
            y_position,
            status,
            date,
            time,
            message,
            image_bytes,
            image_filepath,
        ) = self.alert_data
        self.show_details.emit(
            {
                "message": message,
                "date": str(date),
                "time": str(time),
                "status": status,
                "image_filepath": image_filepath,
                "image_bytes": image_bytes,
            }
        )


class InfoPanel(QGroupBox):
    map_loaded = pyqtSignal(str)

    def __init__(self, parent) -> None:
        super().__init__()
        self.main_layout = QStackedLayout()
        self.stats_panel = QWidget()
        self.alert_details = AlertDetails()
        layout = QVBoxLayout(self)
        self.stacked_layout = QStackedLayout()
        self.container = QWidget()

        self.setStyleSheet("""
        QGroupBox {
            background-color: white;
        }""")
        self.generate_heatmap_task = None
        self.generate_alerts_stats_task = None
        msg3 = '. Seleccione un mapa de la carpeta de "maps", y genere un \nestadisticas de inspección.'
        msg1 = "Recopila estadísticas de diagnóstico relacionadas con la frecuencia y horarios  de los patrullajes del\nrobot asociadas a un mapa  "
        msg1 = msg1 + msg3
        msg2 = "Recopila estadísticas de diagnóstico relacionadas con los  lugares donde se generan alertas en los\npatrullajesdel robot asociadas a un mapa"
        msg2 = msg2 + msg3

        self.heatmap = ImageViewer(parent=parent)
        self.heatmap_img_filepath = None
        self.patrols_stats = PatrolStatsViewer(parent=parent)

        map_filepath_label = QLabel("x")
        map_filepath_label.hide()
        stats_title_image = QLabel("Estatdiaticas de Monitoreo")
        pixmap = QPixmap("./stats.png").scaled(320, 300)
        stats_title_image.setPixmap(pixmap)

        self.checkpoints_stats = StatContainer(
            title="Estadisticas de puntos de interés", msg=msg2
        )
        self.alerts_stats = StatContainer(title="Estadisticas  de Alertas", msg=msg1)

        layout.addWidget(map_filepath_label)
        layout.addWidget(stats_title_image, alignment=Qt.AlignCenter)
        layout.addWidget(self.checkpoints_stats, alignment=Qt.AlignCenter)
        # layout.addWidget(self.alerts_stats, alignment=Qt.AlignCenter)

        self.checkpoints_stats.generate_stats.connect(self.get_checkpoints_stats)
        self.checkpoints_stats.viewdata_button.clicked.connect(self.show_heatmap)

        self.alerts_stats.map_loaded.connect(self.set_map)
        self.alerts_stats.generate_stats.connect(self.get_alerts_stats)
        self.alert_details.back2stats_button.clicked.connect(self.go2stats)

        self.stats_panel.setLayout(layout)
        self.main_layout.addWidget(self.stats_panel)
        self.main_layout.addWidget(self.alert_details)

        self.container.setLayout(self.main_layout)
        self.stacked_layout.addWidget(self.container)
        self.stacked_layout.addWidget(self.heatmap)

        self.heatmap.on_close.connect(self.go2stats)

        self.setLayout(self.stacked_layout)

    def go2stats(self):
        self.main_layout.setCurrentIndex(0)
        self.stacked_layout.setCurrentIndex(0)
        print("Go to stats")

    def go2alert_details(self):
        self.main_layout.setCurrentIndex(1)

    def handleShowAlertInfo(self, info_data):
        self.alert_details.set_data(info_data)
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
            self.stacked_layout.setCurrentIndex(1)

    def set_map(self, map_filepath):
        self.map_loaded.emit(map_filepath)
        print("map loaded Logsistem")
        self.patrols_stats.show_win()

    def get_alerts_stats(self, map_filepath):
        if (
            self.generate_alerts_stats_task
            and self.generate_alerts_stats_task.isRunning()
        ):
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


class AlertDetails(QGroupBox):
    def __init__(self) -> None:
        super().__init__()
        self.layout = QVBoxLayout()
        self.info_layout = QVBoxLayout()

        self.image_label = QLabel("No Image")
        self.image_label.setFixedSize(800, 400)
        self.image_label.setScaledContents(True)
        self.image_label.setStyleSheet(
            "border: 1px solid #ddd; background-color: #f0f0f0;"
        )

        self.title_label = QLabel("ERROR: --")
        self.datetime_label = QLabel("----/--/-- --:--:--")
        self.message_label = QLabel("Message details...")

        self.title_label.setStyleSheet(title_label_style)
        self.datetime_label.setStyleSheet(section_header_label_style)
        self.message_label.setStyleSheet(normal_label_style)
        self.message_label.setWordWrap(True)

        self.back2stats_button = QPushButton("< Volver")
        self.playdata_button = QPushButton("Play recorded data")
        # self.back2stats_button.setStyleSheet(secondary_button_style)
        self.playdata_button.setStyleSheet(primary_button_style)

        self.layout.addWidget(self.image_label, alignment=Qt.AlignCenter)
        # self.info_layout.addWidget(self.title_label)
        # self.info_layout.addWidget(self.datetime_label)
        self.info_layout.addWidget(self.message_label)
        self.layout.addLayout(self.info_layout)

        buttons_layout = QHBoxLayout()
        buttons_layout.addWidget(self.datetime_label)
        buttons_layout.addWidget(self.back2stats_button)
        self.layout.addLayout(buttons_layout)

        self.setLayout(self.layout)

    def set_data(self, data):
        message = data.get("message", "N/A")
        date = data.get("date", "N/A")
        time = data.get("time", "N/A")
        status = data.get("status", 0)
        image_bytes = data.get("image_bytes")
        image_filepath = data.get("image_filepath")

        status_text = "INFO"
        if status == AlertStatus.ERROR.value:
            status_text = "ERROR"
        elif status == AlertStatus.WARNING.value:
            status_text = "WARNING"

        self.title_label.setText(f"{status_text}: {message}")
        self.datetime_label.setText(f"{date} {time}")
        self.message_label.setText(f"{message}")

        if image_filepath:
            pixmap = QPixmap(image_filepath)
            # pixmap.loadFromData(image_bytes)
            if not pixmap.isNull():
                self.image_label.setPixmap(pixmap)
            else:
                self.image_label.setText("Invalid Image Data")
        else:
            self.image_label.setText("No Image Available")


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
        self.map_filepath = file_path.split('.')[0]
        return file_path


class TaskWorkerHeatmap(QThread):
    task_finished = pyqtSignal(str)

    def __init__(self, data=[], map: str = "") -> None:
        super().__init__()
        self.map = map

    def run(self):
        ma = InternalStorageManager()
        data = ma.get_alerts_error(self.map)
        print(f"Esto es la data {__name__}: {data}")
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
            map_file=self.map,
        )
        if data:
            self.filter_bytype_finished.emit(data)
            return

        self.filter_bytype_finished.emit([])


class LogPanel(QWidget):
    def __init__(self, node_manager=None, parent=None):
        super().__init__()
        self._database = None
        self._database1 = None

        self.layout = QGridLayout(self)
        self.main_layout = QHBoxLayout()

        # Core Pagination Variables
        self.page_number = 0
        self.PAGE_SIZE = 9  # Fixed typo from PAGE_ZISES
        self.status = None
        self.ascendant = False
        self.map = None
        self.alerts_items = []
        self._database = None

        # Setup the dynamic pagination UI component

        self.info_panel = InfoPanel(parent=parent)
        self.toggle = True
        self.node_manager = node_manager

        self.stacked_layout = QStackedLayout()
        self.main_layout.addLayout(self.stacked_layout)
        self.update_data_btn = QPushButton('Actualizar datos')

        self.start_date_edit = QDateEdit(QDate.currentDate().addDays(-30))
        self.end_date_edit = QDateEdit(QDate.currentDate())
        self.input_layout = QVBoxLayout()
        btn = QPushButton("Cargar mapa")
        self.input_layout.addWidget(btn)

        # Histogram Section
        self.histogram_container = QWidget()
        self.histogram_layout = QVBoxLayout(self.histogram_container)

        self.granularity_layout = QHBoxLayout()
        self.granularity_label = QLabel("Granularidad:")
        self.granularity_label.setStyleSheet(muted_label_style)
        self.granularity_combo = QComboBox()
        self.granularity_combo.addItems(["Mensual", "Anual", "Por hora"])
        self.granularity_combo.currentIndexChanged.connect(self.getHistogramData)

        self.granularity_layout.addWidget(self.granularity_label)
        self.granularity_layout.addWidget(self.granularity_combo)
        self.granularity_layout.addStretch()

        self.alert_histogram = AlertHistogram()
        self.histogram_layout.addLayout(self.granularity_layout)
        self.histogram_layout.addWidget(self.alert_histogram)

        self.maps_selection = QComboBox()
        self.get_maps()
        self.maps_selection.currentIndexChanged.connect(self.handleMapSelection)

        self.layout.addLayout(self.main_layout, 4, 0, 1, 2)
        # self.layout.addLayout(self.input_layout, 3, 0)
        self.layout.addWidget(self.info_panel, 4, 2)
        self.layout.addWidget(self.histogram_container, 0, 0, 2, 4)
        self.layout.addWidget(self.maps_selection, 2, 0)
        self.layout.addWidget(self.update_data_btn, 2, 1)

        self.info_panel.map_loaded.connect(self.set_map)
        self.update_data_btn.clicked.connect(self.updateData)

    def updateData(self):
        self.page_number = 0
        self.getAlerts()

    def handleMapSelection(self, index):
        self.page_number = 0
        self.getAlerts()
        self.getHistogramData()

    def getHistogramData(self):
        index = self.granularity_combo.currentIndex()
        if index == 0:
            granularity = "monthly"
        elif index == 1:
            granularity = "yearly"
        else:
            granularity = "hourly"

        map_file = self.maps_selection.currentText()

        self._db_hist = DataBase(
            action="get_alert_histogram_data",
            data={"granularity": granularity, "map_file": map_file},
        )
        self._db_hist.action_completed.connect(self.handleHistogramFinish)
        self._db_hist.start()

    def handleHistogramFinish(self, msg, data):
        if msg == "SuccessGetHistogramData":
            hist_data = data.get("histogram", [])
            labels = [row[0] for row in hist_data]
            values = [row[1] for row in hist_data]
            self.alert_histogram.update_plot(labels, values)
            self.histogram_container.update()

    def setup_alert_list(self):
        """Creates a dedicated UI container for alerts along with pagination controls."""
        # Main layout container for the alert module section
        self.alert_section_layout = QVBoxLayout()

        # Container specifically holding alert item cards
        self.alerts_container = QWidget()
        self.alerts_layout = QVBoxLayout(self.alerts_container)
        self.alerts_layout.setSpacing(5)
        self.alerts_layout.addStretch()

        # Pagination Control Layout
        self.pagination_layout = QHBoxLayout()

        self.prev_button = QPushButton("Anterior")
        self.next_button = QPushButton("Siguiente")
        self.page_label = QLabel("Página 1")

        # Apply styling rules matching your design tokens
        # self.prev_button.setStyleSheet(secondary_button_style)
        # self.next_button.setStyleSheet(secondary_button_style)
        self.page_label.setStyleSheet(muted_label_style)
        self.page_label.setAlignment(Qt.AlignCenter)

        # Pagination Action Connections
        self.prev_button.clicked.connect(self.goToPrevPage)
        self.next_button.clicked.connect(self.goToNextPage)

        # Assemble Pagination Control Bar
        self.pagination_layout.addWidget(self.prev_button)
        self.pagination_layout.addWidget(self.page_label, 1)
        self.pagination_layout.addWidget(self.next_button)

        # Date Filtering UI
        self.filter_layout = QHBoxLayout()
        self.start_date_edit.setCalendarPopup(True)
        self.end_date_edit.setCalendarPopup(True)

        self.start_date_edit.dateChanged.connect(lambda: self.getAlerts())
        self.end_date_edit.dateChanged.connect(lambda: self.getAlerts())

        start_label = QLabel("Desde:")
        start_label.setStyleSheet(muted_label_style)
        end_label = QLabel("Hasta:")
        end_label.setStyleSheet(muted_label_style)

        self.filter_layout.addWidget(start_label)
        self.filter_layout.addWidget(self.start_date_edit)
        self.filter_layout.addWidget(end_label)
        self.filter_layout.addWidget(self.end_date_edit)

        # Combine items view and controls into the main section
        self.alert_section_layout.addLayout(self.filter_layout)
        self.alert_section_layout.addWidget(self.alerts_container, 1)
        self.alert_section_layout.addLayout(self.pagination_layout)

        # Inject into the primary window layout hierarchy
        self.main_layout.addLayout(self.alert_section_layout, 2)
        self.getAlerts()

    def add_alerts(self, alerts=[]):
        for alert in alerts:
            x_position, y_position, status, date, time, message, image_bytes, _aaa = (
                alert
            )
            text = f"{message} // {date} {time}"
            if text:
                if status == AlertStatus.ERROR.value:
                    style = error_label_style
                elif status == AlertStatus.WARNING.value:
                    style = warning_label_style
                else:
                    style = info_label_style

                alert_item = AlertItem(text, alert, self, style=style)
                self.alerts_layout.insertWidget(
                    self.alerts_layout.count() - 1, alert_item
                )
                alert_item.show_details.connect(self.info_panel.handleShowAlertInfo)
                self.alerts_items.append(alert_item)

    def remove_alert_item(self):
        for item in self.alerts_items:
            self.alerts_layout.removeWidget(item)
            item.deleteLater()
        self.alerts_items = []

    # --- Database Driven Pagination Controls ---

    def goToPrevPage(self):
        """Triggers a database reload for the previous page window."""
        if self.page_number > 0:
            self.page_number -= 1
            self.getAlerts()

    def goToNextPage(self):
        """Triggers a database reload for the next page window."""
        self.page_number += 1
        print(f"next page: {self.page_number}")
        self.getAlerts()

    def getAlerts(self):
        """Dispatches an asynchronous thread task fetching specific page limits."""
        if self._database and self._database.isRunning():
            return

        self._database = DataBase(
            action="get_filtered_alerts",
            data={
                "status": self.status,
                "ascendant": self.ascendant,
                "page_size": self.PAGE_SIZE,
                "page_number": self.page_number,
                "map_file": self.maps_selection.currentText(),
                "start_date": self.start_date_edit.date().toPyDate(),
                "end_date": self.end_date_edit.date().toPyDate(),
            },
        )
        self._database.action_completed.connect(self.handleTaskFinish)
        self._database.start()

    def handleTaskFinish(self, msg, data):
        """Processes async data return and enforces edge-case pagination states."""
        if self._database:
            self._database.quit()
            self._database.wait()
            self._database = None

        # UI Refresh
        results = data.get("alerts")
        has_more = data.get("has_more", False)
        self.remove_alert_item()
        self.add_alerts(results)

        self.page_label.setText(f"Página {self.page_number + 1}")

        # # Edge Case 1: Disable previous button if we are back on page 0
        self.prev_button.setEnabled(self.page_number > 0)

        # # Edge Case 2: Disable next button if returned rows are less than a full PAGE_SIZE payload
        self.next_button.setEnabled(has_more)

    def filterAlertsAll(self, filter):
        if not self.map:
            return
        self.status = None
        self.page_number = 0  # Always reset viewport to initial page on filter changes
        self.getAlerts()
        self.filter_bytAll_button.setStyleSheet(tag_selected_button_style)
        self.filter_byErrors_button.setStyleSheet(tag_button_style)
        self.filter_byInfos_button.setStyleSheet(tag_button_style)
        self.filter_byWarnings_button.setStyleSheet(tag_button_style)

    def filterAlertsError(self):
        if not self.map:
            return
        self.status = AlertStatus.ERROR.value
        self.page_number = 0  # Always reset viewport to initial page on filter changes
        self.getAlerts()
        self.filter_bytAll_button.setStyleSheet(tag_button_style)
        self.filter_byErrors_button.setStyleSheet(tag_selected_button_style)
        self.filter_byInfos_button.setStyleSheet(tag_button_style)
        self.filter_byWarnings_button.setStyleSheet(tag_button_style)

    def filterAlertsWarning(self):
        if not self.map:
            return
        self.status = AlertStatus.WARNING.value
        self.page_number = 0  # Always reset viewport to initial page on filter changes
        self.getAlerts()
        self.filter_bytAll_button.setStyleSheet(tag_button_style)
        self.filter_byErrors_button.setStyleSheet(tag_button_style)
        self.filter_byInfos_button.setStyleSheet(tag_button_style)
        self.filter_byWarnings_button.setStyleSheet(tag_selected_button_style)

    def filterAlertsInfo(self):
        if not self.map:
            return
        self.status = AlertStatus.INFO.value
        self.page_number = 0  # Always reset viewport to initial page on filter changes
        self.getAlerts()
        self.filter_bytAll_button.setStyleSheet(tag_button_style)
        self.filter_byErrors_button.setStyleSheet(tag_button_style)
        self.filter_byInfos_button.setStyleSheet(tag_selected_button_style)
        self.filter_byWarnings_button.setStyleSheet(tag_button_style)

    def toggleAscendantDescendant(self):
        if not self.map:
            return
        self.ascendant = not self.ascendant
        self.page_number = 0  # Reset pagination context window upon re-sorting
        self.getAlerts()

    def set_map(self, map_filepath):
        # Prevent runtime AttributeError if info_label isn't declared
        if hasattr(self, "info_label"):
            self.info_label.hide()
        self.map = map_filepath
        self.page_number = 0  # Clean slate pagination window context
        print("Log system set map")
        self.getAlerts()

    def get_maps(self):
        if self._database1 and self._database1.isRunning():
            return

        self._database1 = DataBase(action="get_available_maps", data={})
        self._database1.action_completed.connect(self.get_maps_finished)
        self._database1.start()

    def get_maps_finished(self, msg, data):
        maps = data.get("maps")
        for map in maps:
            file_path = map["file_path"]
            self.maps_selection.addItem(file_path)

        self.setup_alert_list()
        self.getHistogramData()
        self._database1.quit()
        self._database1.wait()
        self._database1 = None


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = LogPanel()
    window.show()
    sys.exit(app.exec_())
