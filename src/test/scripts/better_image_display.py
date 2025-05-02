import sys
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
from PyQt5.QtGui import QPixmap, QImage, QPainter
from PyQt5.QtCore import Qt, QRectF
import random
from PyQt5.QtCore import Qt, QPoint, QRect, QSize


class ImageViewer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Image Viewer with QGraphicsView")
        self.setGeometry(100, 100, 384, 384)

        # Initialize UI
        self.init_ui()

        # Variables
        self.current_image_path = "./map2.pgm"
        self.zoom_factor = 1.0
        file_path = "./map2.pgm"

        if file_path:
            self.current_image_path = file_path
            self.display_image(file_path)

    def init_ui(self):
        # Central widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout()

        # Create graphics view and scene
        self.graphics_view = Example()
        # self.graphics_view.setRenderHint(QPainter.Antialiasing)
        # self.graphics_view.setRenderHint(QPainter.SmoothPixmapTransform)
        self.graphics_view.setDragMode(QGraphicsView.ScrollHandDrag)
        self.scene = QGraphicsScene()
        self.graphics_view.setScene(self.scene)
        # pScene = QGraphicsScthis)
        layout.addWidget(self.graphics_view)

        # Buttons
        self.load_button = QPushButton("Load Image")
        self.load_button.clicked.connect(self.load_image)
        layout.addWidget(self.load_button)

        self.zoom_in_button = QPushButton("Zoom In (+)")
        self.zoom_in_button.clicked.connect(self.zoom_in)
        layout.addWidget(self.zoom_in_button)

        self.zoom_out_button = QPushButton("Zoom Out (-)")
        self.zoom_out_button.clicked.connect(self.zoom_out)
        layout.addWidget(self.zoom_out_button)

        self.reset_button = QPushButton("Reset View")
        self.reset_button.clicked.connect(self.reset_view)
        layout.addWidget(self.reset_button)

        central_widget.setLayout(layout)

    def load_image(self):
        # Open file dialog to select image
        # file_path, _ = QFileDialog.getOpenFileName(
        #     self, "Open Image File", "",
        #     "Image Files ( *.pgm)"
        # )
        file_path = "./map2.pgm"

        if file_path:
            self.current_image_path = file_path
            self.display_image(file_path)

    def display_image(self, file_path):
        # Clear previous items from scene
        self.scene.clear()

        # Load image
        pixmap = QPixmap(file_path)
        if pixmap.isNull():
            print("Failed to load image")
            return

        # Add pixmap to scene
        self.scene.addPixmap(pixmap)
        self.graphics_view.scale(1.2, 1.2)


        # Set scene rect to image size
        # self.scene.setSceneRect(pixmap.rect())

        # Reset view
        self.reset_view()

    def zoom_in(self):
        self.graphics_view.scale(1.2, 1.2)
        self.zoom_factor *= 1.2
        self.update()

    def zoom_out(self):
        self.graphics_view.scale(1 / 1.2, 1 / 1.2)
        self.zoom_factor /= 1.2
        self.update()

    def reset_view(self):
        if self.scene.items():
            self.graphics_view.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)
            self.zoom_factor = 1.0

    def resizeEvent(self, event):
        if self.scene.items():
            self.graphics_view.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)
        super().resizeEvent(event)


class Example(QGraphicsView):
    def __init__(self):
        super().__init__()
        self.squares = []  # Store square positions and sizes
        self.square_size = 10  # Size of the square to draw
        self.wheel_delta = 0

        self.setGeometry(300, 300, 350, 300)
        self.setWindowTitle("Shapes")
        self.setMouseTracking(True)
        self.setInteractive(True)
        # self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        # self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        # self.loadImage("./map2.pgm")
        

    def wheelEvent(self, event):
        # Get the wheel delta (positive for up, negative for down)
        delta = event.angleDelta().y()
        
        # Accumulate the total wheel movement
        self.wheel_delta += delta
        
        # Update the label
                
        # You can also print the raw delta for debugging
        print(f"Wheel moved: {delta} (total: {self.wheel_delta})")
        
        # Accept the event
        event.accept()

    def mouseDoubleClickEvent(self, event):
        x = event.pos().x()
        y = event.pos().y()
        point = self.mapToScene(x, y)
        print(f"PUNTOS CON RESPECTO A LA VISTA X:{point.x()} X:{point.y()}")
        print(f"PUNTOS CON RESPECTO A LA VISTA X:{point.x()*0.05} X:{point.y()*0.05} en metros")

        colors = [Qt.red, Qt.yellow, Qt.green, Qt.blue]
        if event.button() == Qt.LeftButton:
            print(f"Mouse Position: ({x}, {y})")  # Update label with mouse position
            square_pos = QPoint(x - self.square_size // 2, y - self.square_size // 2)

            # Add the square to our list
            obj = self.mapToScene(
                QRect(
                    square_pos.x(), square_pos.y(), self.square_size, self.square_size
                )
            ).boundingRect()
            self.squares.append({"color": random.choice(colors), "object": obj})
            self.scale(1.2, 1.2)
            self.scale(1/1.2, 1/1.2) #force a repaint
            self.update()  # Repaint the widget to reflect the updated label
            # self.updateScene()
        else:
            # x_escene =
            # y_scene =
            self.squares = [
                square
                for square in self.squares
                if not square["object"].contains(point.x(), point.y())
            ]
            print(f"FUNCIONA AHORA SI, {len(self.squares)}")
            self.update()
            # self.updateScene()

    def drawForeground(self, painter, rect):
        for square in self.squares:
            self.setForegroundBrush(square['color'])  #     for square in self.squares:
            super().drawForeground(painter, square["object"])
            self.update()


class SimpleItem(QGraphicsItem):
    def __init__(self, left, top, width, height) -> None:
        super().__init__()
        self.top = top
        self.left = left
        self.width = width
        self.height = height

    def boundingRect(self):
        penWidth = 1.0
        return QRectF(
            self.left - penWidth / 2,
            self.top - penWidth / 2,
            self.width + penWidth,
            self.height + penWidth,
        )

    def paint(self, painter, option, widget):
        painter.drawRoundedRect(self.left, self.top, self.width, self.height, 5, 5)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    viewer = ImageViewer()
    viewer.show()
    sys.exit(app.exec_())
