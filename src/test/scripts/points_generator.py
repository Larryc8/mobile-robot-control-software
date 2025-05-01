from PyQt5.QtWidgets import QWidget, QApplication, QLabel
from PyQt5.QtGui import QPainter, QPen, QColor
from PyQt5.QtCore import Qt, QPoint, QRect, QSize
from PyQt5.QtGui import QPixmap
import sys
import random


class PointsGenerator(QLabel):
    def __init__(self):
        super().__init__()
        self.squares = []  # Store square positions and sizes
        self.square_size = 10  # Size of the square to draw

        self.setGeometry(300, 300, 350, 300)
        self.setWindowTitle('Shapes')
        self.setMouseTracking(True)
        self.loadImage("./map2.pgm")

    def mousePressEvent(self, event):
        x = event.pos().x()
        y = event.pos().y()
        colors = [Qt.red, Qt.yellow, Qt.green, Qt.blue]
        if event.button() == Qt.LeftButton:
            print(f"Mouse Position: ({x}, {y})")  # Update label with mouse position
            square_pos = QPoint(x - self.square_size // 2, y - self.square_size // 2)

            # Add the square to our list
            self.squares.append(
                    {
                    'color': random.choice(colors), 
                    'object': QRect(square_pos.x(), square_pos.y(), self.square_size, self.square_size) 
                    }
            )
            self.update()  # Repaint the widget to reflect the updated label
        else:
            self.squares = [
                square for square in self.squares if not square['object'].contains(x, y)
            ]
            print(f"FUNCIONA AHORA SI, {len(self.squares)}")
            self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setBrush(Qt.red)

        for square in self.squares:
            painter.setPen(square['color'])
            painter.drawRect(square['object'])

    def loadImage(self, image_path: str):
        pixmap = QPixmap(image_path)
        if pixmap.isNull():
            print('LA IMAGEN ES NULA')
        pixmap.scroll(10, 10, pixmap.rect())
        print(pixmap.size())
        # pixmap.setDevicePixelRatio(0.5)
        # self.scroll(50, 50)
        # pixmap = pixmap.scaled(QSize(500, 500), Qt.KeepAspectRatioByExpanding )
        self.setPixmap(pixmap)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    ex = PointsGenerator()
    ex.show()
    sys.exit(app.exec_())
