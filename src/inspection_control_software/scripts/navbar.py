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
from PyQt5.QtGui import QPixmap, QImage, QPicture, QIcon
from PyQt5.QtCore import QSize, Qt


class TopBar(QWidget):
    def __init__(self):
        super().__init__()
        self.layout = QGridLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        images_filenames = ["PSI_LOGO.png", "./public/APPLOGO.png", "UnivalleLogo.jpg"]

        image = [
            CustomImage(filename, label)
            for filename, label in zip(images_filenames, ["./public/APPLOGO.png", "BOT INSPECTOR", ""])
        ]

        button1 = QPushButton('BOT INSPECTOR')
        icon1 = QIcon("./public/APPLOGO.png")  # Load from file
        button1.setIcon(icon1)
        # button1.setFixedSize(48, 48)
        button1.setIconSize(QSize(150, 52))
        button1.setStyleSheet("""
            QPushButton {
            background-color: blue;
            border: none;
            }""")

        button2 = QPushButton()
        icon2 = QIcon("./public/UNIVALLE_LOGO.png")  # Load from file
        button2.setIcon(icon2)
        # button1.setFixedSize(48, 48)
        button2.setIconSize(QSize(250, 52))
        button2.setStyleSheet("""
            QPushButton {
            background-color: blue;
            border: none;
            }""")


        button3 = QPushButton()
        icon3 = QIcon("./public/PSI_LOGO.png")  # Load from file
        button3.setIcon(icon3)
        # button1.setFixedSize(48, 48)
        button3.setIconSize(QSize(152, 52))
        button3.setStyleSheet("""
            QPushButton {
            background-color: blue;
            border: none;
            }""")

        self.layout.addWidget(button2, 0, 0, alignment=Qt.AlignLeft)
        self.layout.addWidget(button1, 0, 1, alignment=Qt.AlignHCenter)
        self.layout.addWidget(button3, 0, 2, alignment=Qt.AlignRight)
        # self.layout.addWidget(image[1], 0, 1, alignment=Qt.AlignHCenter)
        # self.layout.addWidget(image[2], 0, 2, alignment=Qt.AlignRight)
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
        pixmap.load(file_name)
        image.setPixmap(pixmap)

        # if label_text:
        #     label.setText(label_text)

        self.layout.addWidget(image)
        self.layout.addWidget(label)
        self.setLayout(self.layout)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    ex = TopBar()
    ex.show()
    sys.exit(app.exec())
