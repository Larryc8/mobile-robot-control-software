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
)
from PyQt5.QtGui import QPixmap, QImage, QPainter, QPainterPath
from PyQt5.QtCore import Qt, QRectF, QSize
import random
import yaml
from datetime import datetime

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
)  # , pyqtSlot


class ImageViewer(QMainWindow):
    save_selected_points = pyqtSignal(dict)

    def __init__(self, ylm_map_filepath=None, parent=None, nodes_manager=None, patrols_scheduler=None):
        super().__init__()
        # self.setWindowTitle("Image Viewer with QGraphicsView")
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.Popup)
        self.setGeometry(100, 100, 384, 384)

        # Initialize UI
        self.init_ui()

        # Variables
        # self.current_image_path = "./map2.pgm"
        self.zoom_factor = 1.0
        self.nodes_manager = nodes_manager
        self.parent  = parent
        self.resolution = 0
        self.botton_left_map = (999,999)
        # self.save_selected_points.connect(self.)
        # file_path = "./map2.pgm"

        # if file_path:
        # self.current_image_path = file_path
        # self.display_image(file_path)

    def init_ui(self):
        # Central widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout()

        # Create graphics view and scene
        self.graphics_view = Example()

        # self.graphics_view.setRenderHint(QPainter.Antialiasing)
        # self.graphics_view.setRenderHint(QPainter.SmoothPixmapTransform)
        # self.graphics_view.setDragMode(QGraphicsView.ScrollHandDrag)
        # self.scene = QGraphicsScene()
        # self.graphics_view.setScene(self.scene)
        # pScene = QGraphicsScthis)
        layout.addWidget(self.graphics_view)

        # Buttons
        # self.load_button = QPushButton("Load Image")
        self.close_button = QPushButton("close")
        self.save_button = QPushButton("save")
        # self.close_button.clicked.connect(self.load_image)
        self.save_button.clicked.connect(self.save_points)
        self.close_button.clicked.connect(self.close_win)
        # layout.addWidget(self.load_button)
        layout.addWidget(self.save_button)
        layout.addWidget(self.close_button)

        # self.zoom_in_button = QPushButton("Zoom In (+)")
        # self.zoom_in_button.clicked.connect(self.zoom_in)
        # layout.addWidget(self.zoom_in_button)

        # self.zoom_out_button = QPushButton("Zoom Out (-)")
        # self.zoom_out_button.clicked.connect(self.zoom_out)
        # layout.addWidget(self.zoom_out_button)

        # self.reset_button = QPushButton("Reset View")
        # self.reset_button.clicked.connect(self.reset_view)
        # layout.addWidget(self.reset_button)
        self.save_button.setEnabled(False)

        central_widget.setLayout(layout)

    def show_win(self):
        # popup_x = self.parent.x() + (self.parent.width() - self.width()) // 2
        # popup_y = self.parent.y() + (self.parent.height() - self.height()) // 2
        popup_x = self.parent.x() + 30 
        popup_y = self.parent.y() + 60
        self.move(popup_x, popup_y)
        self.show()

    def close_win(self):
        self.hide()


    def save_points(self):
        a = self.getPointsInMap()
        self.save_selected_points.emit(a)

        logging.info("from ImageViewer POINT SAVED")

    def getPointsInMap(self) -> dict:
        path = {}
        if self.graphics_view.pixmap:
            width = self.graphics_view.pixmap.width()
            height = self.graphics_view.pixmap.height()
        
            print('image viwer', width, height, self.resolution)
            x_adjust_factor = abs(self.botton_left_map[0]) - width*self.resolution/2 
            y_adjust_factor = abs(self.botton_left_map[1]) - height*self.resolution/2
            print("IMAGE VIWER adjets factors", x_adjust_factor, y_adjust_factor)
            path = {
                str(id): {
                    "x_meters": (point["x"] - width / 2 ) * self.resolution - x_adjust_factor/2,
                    "y_meters": -(point["y"] - height / 2) * self.resolution - y_adjust_factor,
                    "yaw_degrees": 0,
                    "checked": False,
                }
                for id, point in self.graphics_view.getPointsPath().items()
            }
            print("Image viwer map points", path)
        return path

    # def display_image(self, file_path):
    #     # Clear previous items from scene
    #     self.scene.clear()

    #     # Load image
    #     pixmap = QPixmap(file_path)
    #     if pixmap.isNull():
    #         print("Failed to load image")
    #         return

    #     # Add pixmap to scene
    #     self.scene.addPixmap(pixmap)
    #     self.graphics_view.scale(1.2, 1.2)

    #     # Set scene rect to image size
    #     # self.scene.setSceneRect(pixmap.rect().x(), pixmap.rect().y(), pixmap.rect().width(), pixmap.rect().height())

    #     # Reset view
    # self.reset_view()

    def load_map(self, file_path):
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
            self.botton_left_map = (x,y)
            self.save_button.setEnabled(True)

        except Exception as e:
            print(e)

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
        self.graphics_view.fitInView(
            self.graphics_view.scene.sceneRect(), Qt.KeepAspectRatio
        )
        self.zoom_factor = 1.0

    def resizeEvent(self, event):
        # if self.graphics_view.scene.items():
        #     self.graphics_view.fitInView(
        #         self.graphics_view.scene.sceneRect(), Qt.KeepAspectRatio
        #     )
        super().resizeEvent(event)


class Example(QGraphicsView):
    def __init__(self):
        super().__init__()
        self.squares = {}  # Store square positions and sizes
        self.pointsToCheck = {}
        self.square_size = 10  # Size of the square to draw
        self.wheel_delta = 0
        self.points_path = []
        self.zoom_factor = 1
        self.pixmap = None
        self.resolution = 0

        self.setGeometry(300, 300, 350, 300)
        self.setWindowTitle("Shapes")
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

    def getPointsPath(self):
        print(self.pointsToCheck)
        if self.pixmap:
            width = self.pixmap.width()
            height = self.pixmap.height()
            # path = [((point['x'] - width/2)*0.05, (point['y'] - height/2) )]

        for id, point in self.pointsToCheck.items():
            x, y = point["x"] - width / 2, point["y"] - height / 2
            print(f"new point x={x*0.05} m y={-y*0.05} m")
        return self.pointsToCheck

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
        if delta < 0 and self.zoom_factor < 30:
            self.zoom_in()

        # You can also print the raw delta for debugging
        print(f"Wheel moved: {delta} (total: {self.wheel_delta})")
        event.accept()

    def mouseDoubleClickEvent(self, event):
        x = event.pos().x()
        y = event.pos().y()
        point = self.mapToScene(x, y)
        self.points_path.append(point)
        print(f"PUNTOS CON RESPECTO A LA VISTA X:{point.x()} Y:{point.y()}")
        print(
            f"PUNTOS CON RESPECTO A LA VISTA X:{point.x()*0.05} X:{point.y()*0.05} en metros"
        )

        colors = [Qt.red, Qt.yellow, Qt.green, Qt.blue]
        if event.button() == Qt.LeftButton:
            print(f"Mouse Position: ({x}, {y})")  # Update label with mouse position
            square_pos = QPoint(
                x - self.square_size * self.zoom_factor // 2,
                y - self.square_size * self.zoom_factor // 2,
            )

            point = QPoint(
                x - self.square_size // 2,
                y - self.square_size // 2,
            )

            # Add the square to our list
            obj = self.mapToScene(
                QRect(
                    square_pos.x(),
                    square_pos.y(),
                    self.square_size * self.zoom_factor,
                    self.square_size * self.zoom_factor,
                )
            ).boundingRect()

            sync_id = str(datetime.now().timestamp())

            self.pointsToCheck.update(
                {str(sync_id): {"color": Qt.red, "x": obj.x(), "y": obj.y()}}
            )

            self.squares.update(
                {
                    str(sync_id): {
                        "color": random.choice(colors),
                        "object": obj,
                    }
                }
            )
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
                if not square["object"].contains(point.x(), point.y()):
                    temp_squares.update({id: square})
                    temp_points.update({id: self.pointsToCheck[id]})

            self.squares = temp_squares
            self.pointsToCheck = temp_points

            # self.pointsToCheck = [point for point in self.pointsToCheck if point in ]

            print(f" image viewer FUNCIONA AHORA SI, {len(self.squares)}")
            self.update()
            # self.updateScene()

    def load_stored_points(self):
        obj = QRectF(
            10,
            10,
            self.square_size * self.zoom_factor,
            self.square_size * self.zoom_factor,
        )

        id_sync = datetime.now().timestamp()
        self.squares.update({str(id_sync): {"color": Qt.red, "object": obj}})
        self.pointsToCheck.update(
            {str(id_sync): {"color": Qt.red, "x": obj.x(), "y": obj.y()}}
        )

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

    def drawForeground(self, painter, rect):
        painter.drawLine(0, 0, 20, 20)
        for id, square in self.squares.items():
            self.setForegroundBrush(square["color"])  #     for square in self.squares:
            # transform = self.transform()
            # scale = transform.m11()
            # width = square["object"].width()
            # height = square['object'].height()
            # size = QSizeF((width * scale), (height * scale))
            # square["object"].setSize(size)
            super().drawForeground(painter, square["object"])
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


if __name__ == "__main__":
    app = QApplication(sys.argv)
    viewer = ImageViewer()
    viewer.show()
    sys.exit(app.exec_())
