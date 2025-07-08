import math
import sys
from datetime import datetime
import typing
import rospy
import cv2 as cv
import tf

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

from PyQt5.QtCore import Qt, QThread,pyqtSignal
from PyQt5.QtGui import QPixmap

from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap

from rview import MyViz
from robot_vision import ImageMatcheChecker
from database_manager import DataBase
from config_model import NodesManager


from pyqttoast import Toast, ToastPreset

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
            trans, rotation_qua = listener.lookupTransform("map", "base_link", rospy.Time(0))

            yaw =  tf.transformations.euler_from_quaternion(rotation_qua) 
            print("translation", trans, yaw)

            self.task_completed.emit(tuple(trans[:2]) + tuple([yaw[2]]))
        except Exception as e:
            print(e)
            self.task_completed.emit(tuple([0, 0]))


class RobotCamera(QGroupBox):
    send_buffered_data = pyqtSignal(list)
    def __init__(self, buffer, parent) -> None:
        super().__init__("Camara del robot")
        self.layout = QVBoxLayout()
        # self.container.setLayout(self.layout)
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.image_label)
        # self.load_image('./mora1.png')
        self.menu_btn = QPushButton("Menu")
        self.menu_btn.setMaximumWidth(100)
        self.setup_submenu()
        self.layout.addWidget(self.menu_btn)
        self.cv_image = None
        self.database = None
        self.data_buffer = []
        self.parent = parent
        self.pose_getter = None
        self.data_buffer2 = buffer
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

    def setup_submenu(self):
        menu = QMenu("Transform", self)

        save_image_action = QAction("guardar referencia", self)
        save_image_action.triggered.connect(self.buffer_reference_image)
        menu.addAction(save_image_action)

        lowercase_action = QAction("option 2", self)
        lowercase_action.triggered.connect(self.hide_camera)
        menu.addAction(lowercase_action)
        self.menu_btn.setMenu(menu)

    def save_buffered_data(self, mapfile):
        if self.database and self.database.isRunning():
            return

        print(f'{__name__} map: {mapfile}')
        print(f'{__name__} number of points: {len(self.data_buffer)}')
        points = {}
        try:
            for data in self.data_buffer:
                id = str(datetime.now().timestamp())
                print(f'{__name__} {data[1]}')
                x, y, yaw = data[2]
                image= data[1]
                cv.imwrite(data[1], data[0])
                _x, _y = math.cos(yaw), math.sin(yaw)
                gui_yaw = math.atan2(_y, -_x)
                points.update({
                    str(id): {
                    "x_meters": x,  # * self.resolution,
                    "y_meters": y,  # * self.resolution,
                    "yaw_degrees": 0,
                    'yaw': yaw,
                    "checked": False,
                    "mapfile": mapfile,
                    'type': 1,
                    'gui_yaw': gui_yaw, 
                    'image': image
                }})
            print(f'{__name__} number of _points: {len(points)} {points}')
            self.database = DataBase(action='add_points', data=points, mapfile=mapfile)
            self.database.action_completed.connect(self.database_task_completed)
            self.database.start()

        except Exception as identifier:
            pass

    def database_task_completed(self, x, y):
        print(x)
        pass

    def buffer_reference_image(self):
        print('SAVE IMAGE REFERENCE')
        if self.pose_getter and self.pose_getter.isRunning():
            return

        self.pose_getter = TaskWorker()
        self.pose_getter.task_completed.connect(self.set_pose_and_image)
        self.pose_getter.start()

    def set_pose_and_image(self, pose):
        if len(self.data_buffer) < 10:
            id = str(datetime.now().timestamp())
            img_file_path = f'./reference_images/reference_image{id}.jpg'
            self.data_buffer.append((self.current_image, img_file_path, pose))

            if self.current_image is not None:
                height, width, channel = self.current_image.shape
                bytes_per_line = 3 * width
                q_img = QImage(
                    self.current_image.data,
                    width,
                    height,
                    bytes_per_line,
                    QImage.Format_RGB888,
                ).rgbSwapped()

                pixmap = QPixmap.fromImage(q_img)
                self.data_buffer2.append(pixmap)
                print(f'{__name__} curent image shape', self.current_image.shape)
                self.send_buffered_data.emit(self.data_buffer2)

        print(f'{__name__} pose', pose)

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
            self.resize_image()

    def hide_camera(self):
        self.image_label.hide()
        # self.setFixedSize(140, 80)
        print("camer button cliked")
        pass


if __name__ == "__main__":
    app = QApplication(sys.argv)
    viewer = RobotCamera(None)
    viewer.show()

    sys.exit(app.exec_())
