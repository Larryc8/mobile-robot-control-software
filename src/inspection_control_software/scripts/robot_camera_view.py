import math
import sys
from datetime import datetime
import typing
import rospy
import cv2 as cv
import tf

from pyqttoast import Toast, ToastPreset

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

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
    QMenu,
    QAction,
)

from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QPixmap

from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget
from PyQt5.QtCore import QTimer, QRect
from PyQt5.QtGui import QImage, QPixmap, QBitmap, QPainter, QPen, QColor

from robot_vision import ImageMatcheChecker
from database_manager import DataBase
from config_model import NodesManager


from styles.buttons import border_button_style, button_with_menu_style, tertiary_button_style, menu_style


groupbox_style = """
            QGroupBox {
                background-color: #f8f9fa;
                border: 1px solid lightgray;
                margin-top: 2ex;
                padding: 1px;
                color: black;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                padding: 0 10px;
                background-color: blue;
                left: -1px;
                right: -1px;
            }
        """

class TaskWorker(QThread):
    task_completed = pyqtSignal(tuple)

    def __init__(self) -> None:
        super().__init__()

    def run(self):
        pass
        try:
            listener = tf.TransformListener()
            # rospy.loginfo('waiting for map frame')
            listener.waitForTransform(
                target_frame="map",
                source_frame="base_link",
                time=rospy.Time(0),
                timeout=rospy.Duration(4),
            )
            trans, rotation_qua = listener.lookupTransform(
                "map", "base_link", rospy.Time(0)
            )

            yaw = tf.transformations.euler_from_quaternion(rotation_qua)
            print("translation", trans, yaw)

            self.task_completed.emit(tuple(trans[:2]) + tuple([yaw[2]]))
        except Exception as e:
            print(e)
            self.task_completed.emit(tuple([0, 0]))


class RobotCamera(QGroupBox):
    send_buffered_data = pyqtSignal(list)
    custom_option_clicked = pyqtSignal()

    def __init__(self, buffer, parent) -> None:
        super().__init__("Camara del robot", parent)
        self.layout = QVBoxLayout()
        # self.container.setLayout(self.layout)
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.image_label)
        self.setStyleSheet(groupbox_style)
        # self.load_image('./mora1.png')
        self.menu_btn = QPushButton("menu")
        self.menu_btn.setMaximumWidth(100)
        self.menu_btn.setStyleSheet(tertiary_button_style + button_with_menu_style )
        self.menu_btn.hide()
        self.setup_submenu()
        self.layout.addWidget(self.menu_btn)
        self.cv_image = None
        self.database = None
        self.data_buffer = buffer
        self.parent = parent
        self.pose_getter = None
        # self.data_buffer2 = buffer
        self.nodes_manager = NodesManager()

        # Create CV bridge
        self.bridge = CvBridge()

        # Subscribe to image topic
        self.image_sub = rospy.Subscriber("/camera/image", Image, self.image_callback)

        # Timer to check for new images
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_display)
        self.timer.start(30)  # Update at ~30fps
        # Store the latest image
        self.current_image = None
        self.setLayout(self.layout)

    def enterEvent(self, event):
        self.menu_btn.show()

        super().enterEvent(event)
        
    def leaveEvent(self, event):
        self.menu_btn.hide()

        super().leaveEvent(event)

    def add_visual_aid(self, pixmap):
        if pixmap.isNull():
            print("Failed to load image")
            return -1

        masked_pixmap = pixmap.copy()
        result = pixmap

        painter = QPainter(result)
        painter.drawPixmap(0, 0, masked_pixmap)

        pen = QPen(QColor(0, 0, 255))  # Red color
        pen.setWidth(3)  # Border thickness
        painter.setPen(pen)
        painter.setBrush(Qt.NoBrush)  # No fill
        painter.drawRect(QRect(60, 60, 170, 150))
        painter.end()

        return result

    def setup_submenu(self):
        menu = QMenu("Transform", self)
        menu.setStyleSheet(menu_style)

        self.save_image_action = QAction("guardar referencia", self)
        self.save_image_action.triggered.connect(self.buffer_reference_image)
        menu.addAction(self.save_image_action)

        lowercase_action = QAction("maximizar/minimizar", self)
        lowercase_action.triggered.connect(self.toggleSize)
        menu.addAction(lowercase_action)
        self.menu_btn.setMenu(menu)

    def save_buffered_data(self, mapfile):
        if self.database and self.database.isRunning():
            return

        print(f"{__name__} map: {mapfile}")
        print(f"{__name__} number of points: {len(self.data_buffer)}")
        points = {}
        try:
            for data in self.data_buffer:
                id = str(datetime.now().timestamp())
                print(f"{__name__} {data[1]}")
                x, y, yaw = data[2]
                image_filepath = data[1]
                cv.imwrite(data[1], data[0])
                _x, _y = math.cos(yaw), math.sin(yaw)
                gui_yaw = math.atan2(_y, -_x)
                points.update(
                    {
                        str(id): {
                            "x_meters": x,  # * self.resolution,
                            "y_meters": y,  # * self.resolution,
                            "yaw_degrees": 0,
                            "yaw": yaw,
                            "checked": False,
                            "mapfile": mapfile,
                            "type": 1,
                            "gui_yaw": gui_yaw,
                            "image": image_filepath,
                        }
                    }
                )
            print(f"{__name__} number of _points: {len(points)} {points}")
            self.database = DataBase(action="add_points", data=points, mapfile=mapfile)
            self.database.action_completed.connect(self.database_task_completed)
            self.database.start()

        except Exception as identifier:
            pass

    def database_task_completed(self, x, y):
        print(x)
        pass

    def buffer_reference_image(self):
        print("SAVE IMAGE REFERENCE")
        if self.pose_getter and self.pose_getter.isRunning():
            return

        self.pose_getter = TaskWorker()
        self.pose_getter.task_completed.connect(self.set_pose_and_image)
        self.pose_getter.start()

    def set_pose_and_image(self, pose):
        if len(self.data_buffer) < 10:
            id = str(datetime.now().timestamp())
            img_file_path = f"./reference_images/reference_image{id}.jpg"
            self.data_buffer.append((self.current_image, img_file_path, pose))

            # self.data_buffer2.append(pixmap)
            # print(f'{__name__} curent image shape', self.current_image.shape)
            self.send_buffered_data.emit(self.data_buffer)

        print(f"{__name__} pose", pose)

        toast = Toast(self.parent)
        toast.setDuration(2000)  # Hide after 5 seconds
        toast.setTitle("Exito!")
        toast.setText("Se agrego punto de referencia!")
        toast.applyPreset(ToastPreset.SUCCESS)  # Apply style preset
        Toast.setPositionRelativeToWidget(self.parent)
        toast.show()

        if self.pose_getter:
            self.pose_getter.quit()
            self.pose_getter.wait()
            self.pose_getter = None

    def paintEvent(self, event) -> None:
        # self.pixmap = QPixmap('./mora1.png')
        self.update_display()
        super().paintEvent(event)

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
            self.pixmap = self.add_visual_aid(self.pixmap)
            self.resize_image()

    def toggleSize(self):
        self.custom_option_clicked.emit()
        # self.setFixedSize(140, 80)
        print("camer button cliked")
        pass


if __name__ == "__main__":
    app = QApplication(sys.argv)
    viewer = RobotCamera(None)
    viewer.show()

    sys.exit(app.exec_())
