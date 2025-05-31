import sys
import logging

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
    QGraphicsOpacityEffect,
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
from PyQt5.QtCore import Qt, QRectF, QSize, QPropertyAnimation
import random
import yaml
from datetime import datetime


from styles.buttons import (
    border_button_style,
    border_button_style_danger,
    primary_button_style,
    secondary_button_style,
    colored_button_style,
    tertiary_button_style,
    toggle_button_style,
    minimal_button_style,
)

from styles.labels import (
    inactive_label_style,
    minimal_label_style,
    succes_label_style,
    info_label_style,
    warning_label_style,
    code_label_style,
)

from PyQt5.QtCore import (
    Qt,
    QRectF,
    QSizeF,
    QPoint,
    QRect,
    QSize,
    QThread,
    pyqtSignal,
    QObject,
    QTimer,
)  # , pyqtSlot


class ImageViewer(QMainWindow):
    save_selected_points = pyqtSignal(dict)
    save_in_database = pyqtSignal(dict)

    def __init__(
        self,
        ylm_map_filepath=None,
        parent=None,
        nodes_manager=None,
        patrols_scheduler=None,
    ):
        super().__init__()
        # self.setWindowTitle("Image Viewer with QGraphicsView")
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.Popup)
        self.setGeometry(100, 100, 700, 600)

        self.init_ui()

        # Variables
        # self.current_image_path = "./map2.pgm"
        self.zoom_factor = 1.0
        self.nodes_manager = nodes_manager
        self.parent = parent
        self.resolution = 0
        self.botton_left_map = (999, 999)
        self.pointsInMap = {}
        # self.save_selected_points.connect(self.)

    def init_ui(self):
        # Central widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout()
        buttons_layout = QHBoxLayout()
        top_buttons_layout = QHBoxLayout()
        checkpoints_info_layout = QHBoxLayout()
        self.timer = QTimer()
        self.timer.timeout.connect(self.hide_alerts)
        self.setBiggerSize = True

        # Create graphics view and scene
        self.graphics_view = Example()
        self.success_label = QLabel("✓ SUCCESS: File saved successfully")
        self.info_label = QLabel(
            "ℹ INFO: Para guardar los puntos de  manera permante da click en Guardar"
        )
        self.warning_label = QLabel(
            "⚠ WARNING: Necesitas cargar un mapa para agrgar puntos interes"
        )
        self.checkpoints_labels_checked = QLabel("▣ Visistado con exito")
        self.checkpoints_labels_nochecked = QLabel("▣ No revisado")
        self.checkpoints_labels_next = QLabel("▣ Siguiente para revision")

        self.success_label.hide()
        self.info_label.hide()
        self.success_label.setStyleSheet(succes_label_style)
        self.info_label.setStyleSheet(info_label_style)
        self.warning_label.setStyleSheet(warning_label_style)
        self.toggle_size_button = QPushButton("◰ Expandir")
        self.import_points_button = QPushButton('Importar puntos de interes de otro mapa')
        self.toggle_size_button.setMaximumSize(110, 30)
        self.toggle_size_button.setStyleSheet(border_button_style)
        # self.toggle_size_button.setStyleSheet('font-size: 20px')

        top_buttons_layout.addWidget(self.import_points_button, alignment=Qt.AlignRight)
        top_buttons_layout.addWidget(self.toggle_size_button, alignment=Qt.AlignRight)
        layout.addLayout(top_buttons_layout)
        layout.addWidget(self.info_label, alignment=Qt.AlignTop)
        layout.addWidget(self.warning_label)
        layout.addWidget(self.graphics_view)
        layout.addWidget(self.success_label)

        # Buttons
        # self.load_button = QPushButton("Load Image")
        self.close_button = QPushButton("Cerrar")
        self.save_button = QPushButton("Guardar")
        # self.close_button.clicked.connect(self.load_image)
        self.save_button.clicked.connect(self.saveInDatabase)
        self.graphics_view.send_points.connect(self.save_points)
        self.close_button.clicked.connect(self.close_win)
        self.graphics_view.pointsChanged.connect(self.handlePointsChanged)
        self.toggle_size_button.clicked.connect(self.toggleSize)
        # layout.addWidget(self.load_button)
        [
            checkpoints_info_layout.addWidget(b)
            for b in (
                self.checkpoints_labels_next,
                self.checkpoints_labels_checked,
                self.checkpoints_labels_nochecked,
            )
        ]
        buttons_layout.addWidget(self.save_button, 2)
        buttons_layout.addWidget(self.close_button, 1)
        layout.addLayout(checkpoints_info_layout)
        layout.addLayout(buttons_layout)

        self.checkpoints_labels_nochecked.setStyleSheet(code_label_style)
        self.checkpoints_labels_checked.setStyleSheet(code_label_style)
        self.checkpoints_labels_next.setStyleSheet(code_label_style)
        self.close_button.setStyleSheet(border_button_style)
        self.save_button.setStyleSheet(primary_button_style)
        self.import_points_button.setStyleSheet(tertiary_button_style)

        icon = QApplication.style().standardIcon(QStyle.SP_DriveNetIcon)
        self.save_button.setIcon(icon)
        icon = QApplication.style().standardIcon(QStyle.SP_DialogCloseButton)
        self.close_button.setIcon(icon)

        # self.zoom_in_button = QPushButton("Zoom In (+)")
        # self.zoom_in_button.clicked.connect(self.zoom_in)
        # layout.addWidget(self.zoom_in_button)

        # self.zoom_out_button = QPushButton("Zoom Out (-)")
        # self.zoom_out_button.clicked.connect(self.zoom_out)
        # layout.addWidget(self.zoom_out_button)

        # self.reset_button = QPushButton("Reset View")
        # self.reset_button.clicked.connect(self.reset_view)
        # layout.addWidget(self.reset_button)
        # self.save_button.setEnabled(False)

        central_widget.setLayout(layout)

    def handlePointsChanged(self, state):
        if state == "added":
            self.success_label.setText(
                "✓ SUCCESS: Se agrego un nuevo punto al patrullaje"
            )
        if state == "deleted":
            self.success_label.setText("✓ SUCCESS: Se elimino un  punto al patrullaje")
        self.success_label.show()
        self.timer.start(4000)

    def hide_alerts(self):
        self.timer.stop()
        self.success_label.hide()
        pass

    def toggleSize(self):
        if self.setBiggerSize:
            self.setGeometry(
                self.parent.x(),
                self.parent.y(),
                self.parent.width(),
                self.parent.height() - 100,
            )
            self.toggle_size_button.setText("◲ Disminuir")
            # self.reset_view()
            self.setBiggerSize = False
        else:
            self.setGeometry(0, 0, 700, 600)
            self.move_win()
            self.toggle_size_button.setText("◰ Expandir")
            # self.reset_view()
            self.setBiggerSize = True

    def show_win(self):
        # popup_x = self.parent.x() + (self.parent.width() - self.width()) // 2
        # popup_y = self.parent.y() + (self.parent.height() - self.height()) // 2
        if self.setBiggerSize:
            self.move_win()
        self.show()

    def move_win(self):
        popup_x = self.parent.x() + (self.parent.width() - self.width() - 10)
        # popup_y = self.parent.y() + (self.parent.height() - self.height() - 30)
        popup_y = self.parent.y() + 100
        self.move(popup_x, popup_y)

    def close_win(self):
        self.hide()

    def save_points(self, a):
        # a = self.getPointsInMap()
        self.pointsInMap = a
        self.save_selected_points.emit(a)

        logging.info("from ImageViewer POINT SAVED")

    def saveInDatabase(self, x):
        self.save_in_database.emit(self.pointsInMap)


    def load_map(self, file_path, stored_database_points=[]):
        try:
            with open(file_path, "r") as file:
                data = yaml.load(file, Loader=yaml.SafeLoader)

            print(file_path)
            [x, y, z] = data["origin"]
            image = data["image"]
            resolution = data["resolution"]

            file_path_array = file_path.split("/")
            root_file_path = "/".join(file_path_array[:-1])
            print(root_file_path)
            self.graphics_view.display_image(f"{root_file_path}/{image}")
            self.resolution = resolution
            self.botton_left_map = (x, y)
            self.graphics_view.botton_left_map = (x, y)
            self.graphics_view.map_resolution = resolution
            self.graphics_view.mapfile = file_path
            # self.save_button.setEnabled(True)
            self.info_label.show()
            self.warning_label.hide()
            self.graphics_view.clear_points()
            # self.load_stored_points.emit(stored_database_points)

        except Exception as e:
            print(e)

    def load_stored_points(self, data):
        print('ImageViewer', data)
        self.graphics_view.load_stored_points(data)

    def update_points_state(self, current_poin_id, next_point_id, point_state):
        self.graphics_view.update_points_state(
            current_poin_id, next_point_id, point_state
        )

    def zoom_in(self):
        # self.graphics_view.scale(1.2, 1.2)
        # self.zoom_factor *= 1.2
        # self.update()
        pass

    def zoom_out(self):
        # self.graphics_view.scale(1 / 1.2, 1 / 1.2)
        # self.zoom_factor /= 1.2
        # self.update()
        pass

    def reset_view(self):
        # if self.graphics_view.scene.items():
        if self.graphics_view.pixmap:
            self.graphics_view.fitInView(
                self.graphics_view.scene.sceneRect(), Qt.KeepAspectRatio
            )
        self.zoom_factor = 1.0

    # def resizeEvent(self, event):
    #     # if self.graphics_view.scene.items():
    #     #     self.graphics_view.fitInView(
    #     #         self.graphics_view.scene.sceneRect(), Qt.KeepAspectRatio
    #     #     )
    #     super().resizeEvent(event)


class Example(QGraphicsView):
    send_points = pyqtSignal(dict)
    pointsChanged = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.squares = {}  # Store square positions and sizes
        self.pointsToCheck = {}
        self.square_size = 10  # Size of the square to draw
        self.wheel_delta = 0
        self.points_path = []
        self.zoom_factor = 1
        self.pixmap = None
        self.map_resolution = 0
        self.mapfile = None
        self.mapfile_database = None

        # self.setGeometry(300, 300, 350, 300)
        # self.setWindowTitle("Shapes")
        self.setMouseTracking(True)
        self.setInteractive(True)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        # self.loadImage("./map2.pgm")

        self.setRenderHint(QPainter.Antialiasing)
        self.setRenderHint(QPainter.SmoothPixmapTransform)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        # self.scene = QGraphicsScene()
        # self.setScene(self.scene)

        # self.load_stored_points()

    def getPointsPath(self) -> dict:
        print("Points to Check:", self.pointsToCheck)
        # if self.pixmap:
        #     width = self.pixmap.width()
        #     height = self.pixmap.height()
        #     # path = [((point['x'] - width/2)*0.05, (point['y'] - height/2) )]

        # for id, point in self.pointsToCheck.items():
        #     x, y = point["x"] - width / 2, point["y"] - height / 2
        #     print(f"new point x={x*0.05} m y={-y*0.05} m")
        return self.pointsToCheck

    def getPointsInMap(self) -> dict:
        path = {}
        if self.pixmap:
            width = self.pixmap.width()
            height = self.pixmap.height()
            # self.resolution = 0.05000000074505806

            print("image viwer", width, height, self.map_resolution)
            # x_adjust_factor = abs(self.botton_left_map[0]) - width * self.resolution / 2
            # y_adjust_factor = (
            #     abs(self.botton_left_map[1]) - height * self.resolution / 2
            # )
            x0_meters, y0_meters = (
                abs(self.botton_left_map[0]),
                height * self.map_resolution - abs(self.botton_left_map[1]),
            )

            # print("IMAGE VIWER adjets factors", x_adjust_factor, y_adjust_factor)
            path = {
                str(id): {
                    "x_meters": (
                        (point["x"]) * self.map_resolution - x0_meters
                    ),  # * self.resolution,
                    "y_meters": -(
                        (point["y"]) * self.map_resolution - y0_meters
                    ),  # * self.resolution,
                    "yaw_degrees": 0,
                    "checked": False,
                    "mapfile": self.mapfile,
                }
                for id, point in self.getPointsPath().items()
            }
            print("Image viwer map points", path)
        return path

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
        # self.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)

    def wheelEvent(self, event):
        # Get the wheel delta (positive for up, negative for down)
        delta = event.angleDelta().y()

        # Accumulate the total wheel movement
        self.wheel_delta += delta

        if delta > 0 and self.zoom_factor > 1:
            self.zoom_out()
        if delta < 0 and self.zoom_factor < 80:
            self.zoom_in()

        # You can also print the raw delta for debugging
        print(f"Wheel moved: {delta} (total: {self.wheel_delta})")
        event.accept()

    def mouseDoubleClickEvent(self, event):
        if not self.pixmap:
            return

        x = event.pos().x()
        y = event.pos().y()
        point_mapped2scene = self.mapToScene(x, y)
        self.points_path.append(point_mapped2scene)
        print(
            f"PUNTOS CON RESPECTO A LA VISTA X:{point_mapped2scene.x()} Y:{point_mapped2scene.y()}"
        )
        print(
            f"PUNTOS CON RESPECTO A LA VISTA X:{point_mapped2scene.x()*0.05} X:{point_mapped2scene.y()*0.05} en metros"
        )

        colors = [Qt.red, Qt.yellow, Qt.green, Qt.blue]
        if event.button() == Qt.LeftButton:
            print(f"Mouse Position: ({x}, {y})")  # Update label with mouse position
            square_pos = QPoint(
                x
                - self.square_size
                * self.zoom_factor
                * (224 / max(self.pixmap.width(), self.pixmap.height()))
                // 2,
                y
                - self.square_size
                * self.zoom_factor
                * (224 / max(self.pixmap.width(), self.pixmap.height()))
                // 2,
            )

            # point_map = QPoint(
            #     x - self.square_size // 2,
            #     y - self.square_size // 2,
            # )
            # point_mapped2scene = self.mapToScene(
            #     x - self.square_size // 2, y - self.square_size // 2
            # )
            point_mapped2scene = self.mapToScene(x, y)

            # Add the square to our list
            obj = self.mapToScene(
                QRect(
                    square_pos.x(),
                    square_pos.y(),
                    self.square_size
                    * self.zoom_factor
                    * (224 / max(self.pixmap.width(), self.pixmap.height())),
                    self.square_size
                    * self.zoom_factor
                    * (224 / max(self.pixmap.width(), self.pixmap.height())),
                )
            ).boundingRect()

            sync_id = str(datetime.now().timestamp())

            self.pointsToCheck.update(
                {
                    str(sync_id): {
                        "color": Qt.red,
                        "x": point_mapped2scene.x(),
                        "y": point_mapped2scene.y(),
                    }
                }
            )

            self.squares.update(
                {
                    str(sync_id): {
                        "color": Qt.red,
                        "object": obj,
                    }
                }
            )
            self.send_points.emit(self.getPointsInMap())
            self.pointsChanged.emit("added")

            self.scale(1.2, 1.2)
            self.scale(1 / 1.2, 1 / 1.2)  # force a repaint
            self.update()  # Repaint the widget to reflect the updated label
            # self.updateScene()
        else:
            temp_squares = {}
            temp_points = {}
            print("squares")
            print(self.squares)
            print("points")
            print(self.pointsToCheck)

            for id, square in self.squares.items():
                if not square["object"].contains(
                    point_mapped2scene.x(), point_mapped2scene.y()
                ):
                    temp_squares.update({id: square})
                    temp_points.update({id: self.pointsToCheck[id]})
                    self.send_points.emit(self.getPointsInMap())
                else:
                    self.pointsChanged.emit("deleted")

            self.squares = temp_squares
            self.pointsToCheck = temp_points

            # self.pointsToCheck = [point for point in self.pointsToCheck if point in ]

            print(f" image viewer FUNCIONA AHORA SI, {len(self.squares)}")
            self.update()
            # self.updateScene()

    def clear_points(self):
        self.pointsToCheck = {}
        self.squares = {}
        

    def load_stored_points(self, stored_points):
        # print('Example', stored_points)
        if stored_points:
            for point in  stored_points.get('points'):
                x_meters, y_meters, map_file  = point
                if map_file == self.mapfile:
                    x_pix = int(
                        (x_meters - self.botton_left_map[0]) / self.map_resolution
                    )  # Change coordinates from real to map's
                    y_pix = (self.pixmap.height()) - int(
                        (y_meters - self.botton_left_map[1]) / self.map_resolution
                    )  # Origin is set at left-bottom corner, so subtraction from map size is needed

                    size =  self.mapToScene(self.square_size, self.square_size)

                    obj = QRectF(
                        x_pix - size.y()*(224 / max(self.pixmap.width(), self.pixmap.height()))*1.3//2,
                        y_pix - size.y()*(224 / max(self.pixmap.width(), self.pixmap.height()))*1.3//2,
                        size.y()* (224 / max(self.pixmap.width(), self.pixmap.height()))*1.3,
                        size.y()* (224 / max(self.pixmap.width(), self.pixmap.height()))*1.3,
                    )
                    # point = QPointF(x_pix, y_pix)


                    id_sync = datetime.now().timestamp()
                    self.squares.update({str(id_sync): {"color": Qt.red, "object": obj}})
                    self.pointsToCheck.update(
                        {str(id_sync): {"color": Qt.red, "x": x_pix, "y": y_pix}}
                    )
        self.update()
        self.send_points.emit(self.getPointsInMap())
        self.pointsChanged.emit("added")

    def display_image(self, file_path):  # ghghghghhg
        self.scene = QGraphicsScene()
        self.setScene(self.scene)
        self.scene.clear()
        # self.reset_view()

        self.pixmap = QPixmap(file_path)
        if self.pixmap.isNull():
            print("Failed to load image")
            return

        self.scene.addPixmap(self.pixmap)
        # self.scale(1.2, 1.2)
        # self.scene.setSceneRect(pixmap.rect().x(), pixmap.rect().y(), pixmap.rect().width(), pixmap.rect().height())
        self.reset_view()
        # self.load_stored_points()

    def update_points_state(self, current_poin_id, next_point_id, point_state):
        # if not current_poin_id and not next_point_id:
        #     print("Image viwer, SQUARE", self.squares)
        #     for id, square in self.squares.items():
        #         print("Image viwer SQuare FOR", square)
        #         square["color"] = Qt.red
        #         self.update()
        #     return

        if point_state < 4:
            if self.squares.get(current_poin_id):
                self.squares[current_poin_id]["color"] = Qt.green
        else:
            if self.squares.get(current_poin_id):
                self.squares[current_poin_id]["color"] = Qt.red

        if next_point_id and self.squares.get(next_point_id):
            self.squares[next_point_id]["color"] = Qt.yellow

    def drawForeground(self, painter, rect):
        # painter.drawLine(0, 0, 20, 20)
        # painter.drawEllipse(20, 20, 20, 20)
        # self.setBackgroundBrush(Qt.gray)
        for id, square in self.squares.items():
            self.setForegroundBrush(square["color"])  #     for square in self.squares:
            # painter.setBrush(square['color'])
            painter.setFont(QFont('Arial', 2))
            painter.setPen(QPen(square['color'], 1, Qt.SolidLine))
            # painter.drawRect(square['object'])

            text = 'No revisado'
            if square['color'] == Qt.red:
                text = 'No revisado'
            if square['color'] == Qt.green:
                text = 'Revisado'
            if square['color'] == Qt.yellow:
                text = 'Siguiente...'
            painter.drawText(square['object'].x(),square['object'].y()+8, text)

            super().drawForeground(painter, square["object"])
            # super().drawForeground(painter, rect)
        self.update()

    def keyPressEvent(self, event):
        move_step = 50
        center = self.mapToScene(self.viewport().rect().center())

        if event.key() == Qt.Key_Left:
            self.centerOn(center.x() - move_step, center.y())
        elif event.key() == Qt.Key_Right:
            self.centerOn(center.x() + move_step, center.y())
        elif event.key() == Qt.Key_Up:
            self.centerOn(center.x(), center.y() - move_step)
        elif event.key() == Qt.Key_Down:
            self.centerOn(center.x(), center.y() + move_step)
        else:
            super().keyPressEvent(event)


class CustomGraphicsItem(QGraphicsItem):
    def __init__(self, x, y, width, height):
        super().__init__()
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.setFlag(QGraphicsItem.ItemIsMovable)
        self.setFlag(QGraphicsItem.ItemIsSelectable)

        # Custom properties
        self.color = QColor(Qt.blue)
        self.selected_color = QColor(Qt.red)
        self.pen_width = 2

    def boundingRect(self):
        # Define the bounding rectangle of the item
        return QRectF(self.x, self.y, self.width, self.height)

    def paint(self, painter, option, widget=None):
        # Custom painting of the item
        pen = QPen(self.selected_color if self.isSelected() else self.color)
        pen.setWidth(self.pen_width)
        painter.setPen(pen)

        brush = QBrush(self.color.lighter(130))
        painter.setBrush(brush)

        painter.drawRect(self.x, self.y, self.width, self.height)

        # Draw some custom decoration
        painter.drawText(self.x + 5, self.y + 15, "Custom Item")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    viewer = ImageViewer()
    viewer.show()
    sys.exit(app.exec_())
