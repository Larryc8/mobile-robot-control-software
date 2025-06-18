import sys
import typing
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

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
    QGraphicsRectItem,
    QGroupBox,
)

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap

from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap

from rview import MyViz
from robot_vision import ImageMatcheChecker


class RobotCamera(QGroupBox):
    def __init__(self) -> None:
        super().__init__("Camara del robot")
        self.layout = QVBoxLayout()
        # self.container.setLayout(self.layout)
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.image_label)
        # self.load_image('./mora1.png')
        self.close_btn = QPushButton("Menu")
        self.close_btn.setMaximumWidth(100)
        self.layout.addWidget(self.close_btn)
        self.cv_image = None

        self.close_btn.clicked.connect(self.hide_camera)

        # Create CV bridge
        self.bridge = CvBridge()

        # Subscribe to image topic
        self.image_sub = rospy.Subscriber("image_topic", Image, self.image_callback)

        # Timer to check for new images
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_display)
        self.timer.start(60)  # Update at ~30fps
        # Store the latest image
        self.current_image = None
        self.setLayout(self.layout)

    def paintEvent(self, event) -> None:
        # self.pixmap = QPixmap('./mora1.png')
        self.update_display()
        super().paintEvent(event)

    # def load_image(self, image_path):
    #     self.pixmap = QPixmap(image_path)

    #     if self.pixmap.isNull():
    #         print(f"Error: Could not load image {image_path}")
    #         return

    #     self.resize_image()

    def resize_image(self):
        self.image_label.setPixmap(self.pixmap)
        self.image_label.setScaledContents(True)

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_image = self.cv_image
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")

    def update_display(self):
        if self.current_image is not None:
            # Convert OpenCV image to QImage
            height, width, channel = self.current_image.shape
            bytes_per_line = 3 * width
            q_img = QImage(
                self.current_image.data,
                width,
                height,
                bytes_per_line,
                QImage.Format_RGB888,
            ).rgbSwapped()

            # Convert QImage to QPixmap and display
            self.pixmap = QPixmap.fromImage(q_img)
            self.resize_image()

    def hide_camera(self):
        # self.image_label.hide()
        # self.setFixedSize(140, 80)
        print('camer button cliked')
        pass


if __name__ == "__main__":
    app = QApplication(sys.argv)
    viewer = RobotCamera(None)
    viewer.show()

    sys.exit(app.exec_())
