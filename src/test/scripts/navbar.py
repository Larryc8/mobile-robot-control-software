import sys
from PyQt5.QtWidgets import (
    QMainWindow,
    QApplication,
    QPushButton,
    QWidget,
    QTabWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QLayout,
    QGridLayout,
    QGroupBox,
    QGraphicsWidget,
    QMenuBar,
)
from PyQt5.QtGui import QPixmap, QImage, QPicture
from PyQt5.QtCore import QSize, Qt


class TopBar(QGroupBox):
    def __init__(self):
        super().__init__()
        self.layout = QGridLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        images_filenames = ["PSI_LOGO.png", "APPLOGO.png", "UnivalleLogo.jpg"]
        image = [
            CustomImage(filename, label)
            for filename, label in zip(images_filenames, ["", "BOT INSPECTOR", ""])
        ]

        self.layout.addWidget(image[0], 0, 0, alignment=Qt.AlignLeft)
        self.layout.addWidget(image[1], 0, 1, alignment=Qt.AlignHCenter)
        self.layout.addWidget(image[2], 0, 2, alignment=Qt.AlignRight)
        self.setLayout(self.layout)


class CustomImage(QWidget):
    def __init__(self, file_name: str, label_text: str) -> None:
        super().__init__()
        absolute_path = "/pico-sdk/mobile-robot-control-software/src/test/images/"
        # self.setStyleSheet("background-color: blue")
        self.layout = QHBoxLayout()
        image = QLabel()
        label = QLabel()

        pixmap = QPixmap()
        pixmap.load(absolute_path + file_name)
        image.setPixmap(pixmap)

        if label_text:
            label.setText(label_text)

        self.layout.addWidget(image)
        self.layout.addWidget(label)
        self.setLayout(self.layout)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    ex = TopBar()
    ex.show()
    sys.exit(app.exec())
