import math
import sys
import typing
from datetime import datetime

import cv2 as cv
import rospy
import tf
from config_model import NodesManager, UserConfigFileManager
from cv_bridge import CvBridge
from database_manager import DataBase
from geometry_msgs.msg import (
    Pose,
    Pose2D,
    PoseStamped,
    Quaternion,
    TransformStamped,
    Twist,
)
from input_textdialog import CustomDialog
from nav_msgs.msg import Odometry
from notification import Notification
from PyQt5.QtCore import QRect, QSize, Qt, QThread, QTimer, pyqtSignal
from PyQt5.QtGui import QBitmap, QColor, QImage, QPainter, QPen, QPixmap
from PyQt5.QtWidgets import (
    QAction,
    QApplication,
    QFileDialog,
    QGraphicsItem,
    QGraphicsOpacityEffect,
    QGraphicsRectItem,
    QGraphicsScene,
    QGraphicsView,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMenu,
    QPushButton,
    QStyle,
    QVBoxLayout,
    QWidget,
)
from pyqttoast import Toast, ToastPosition, ToastPreset
from robot_vision import ImageMatcheChecker
from sensor_msgs.msg import Image
from styles.buttons import (
    border_button_style,
    button_with_menu_style,
    menu_style,
    tertiary_button_style,
)
from styles.labels import inactive_label_style, title_label_style
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf2_msgs.msg import TFMessage

groupbox_style = """
            QWidget {
            background-color: gray;
            padding: 0px 40px;
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
            listener = tf.TransformListener(
                interpolate=True, cache_time=rospy.Duration(11)
            )
            # rospy.loginfo('waiting for map frame')
            # m = TransformStamped()
            # m.header.frame_id = 'map'
            # m.child_frame_id = 'base_link'
            # m.transform.translation.x = 2.71828183
            # m.transform.translation.y = 2.71828183
            # m.transform.rotation.w = 1.0
            # listener.setTransform(m)
            #
            listener.waitForTransform(
                target_frame="map",
                source_frame="base_link",
                time=rospy.Time(),
                timeout=rospy.Duration(10),
            )
            trans, rotation_qua = listener.lookupTransform(
                "map", "base_link", rospy.Time()
            )

            yaw = tf.transformations.euler_from_quaternion(rotation_qua)
            print("translation", trans, yaw)

            self.task_completed.emit(tuple(trans[:2]) + tuple([yaw[2]]))
        except Exception as e:
            print(e)
            self.task_completed.emit(tuple([0, 0]))


class RobotCamera(QWidget):
    send_buffered_data = pyqtSignal(list)
    custom_option_clicked = pyqtSignal()

    def __init__(self, buffer, parent) -> None:
        super().__init__(parent)
        self.layout = QVBoxLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)
        # self.container.setLayout(self.layout)
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setWordWrap(True)
        self.layout.addWidget(self.image_label)
        self.setStyleSheet(groupbox_style)
        # self.load_image('./mora1.png')
        self.menu_btn = QPushButton("menu")
        self.menu_btn.setMaximumWidth(100)
        self.menu_btn.setStyleSheet(tertiary_button_style + button_with_menu_style)
        self.menu_btn.hide()
        self.setup_submenu()
        self.layout.addWidget(self.menu_btn)
        self.cv_image = None
        self.database = None
        self.data_buffer = buffer
        self.parent = parent
        self.pose_getter = None
        # self.data_buffer2 = buffer
        #
        self.user_config = UserConfigFileManager()
        self.image_label.setText("No hay datos de la camara")
        self.image_label.setStyleSheet(
            inactive_label_style + title_label_style + "font-famly: Helvetica"
        )

        self.current_aruco_pose = Pose2D()

        self.buffered_data_pub = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=2
        )
        self.nodes_manager = NodesManager()
        self.tf_sub = rospy.Subscriber("/tf", TFMessage, self.get_transforms)
        self.aruco_odom_sub = rospy.Subscriber(
            "/aruco/odom", Odometry, self.aruco_odom_callback
        )

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

    def get_transforms(self, msg):
        pass

    def enterEvent(self, event):
        # self.menu_btn.show()

        super().enterEvent(event)

    def leaveEvent(self, event):
        self.menu_btn.hide()

        super().leaveEvent(event)

    def add_visual_aid(self, pixmap):
        if pixmap.isNull():
            print("Failed to load image")
            return -1

        masked_pixmap = pixmap.copy()
        size = masked_pixmap.size()
        result = pixmap

        config = self.user_config.read_data()
        w, h = config["roi"]

        rect_size = QSize(w, h)

        x0 = size.width() // 2 - rect_size.width() // 2
        y0 = size.height() // 2 - rect_size.height() // 2

        painter = QPainter(result)
        painter.drawPixmap(0, 0, masked_pixmap)

        pen = QPen(QColor(0, 0, 255))  # Red color
        pen.setWidth(1)  # Border thickness
        painter.setPen(pen)
        painter.setBrush(Qt.NoBrush)  # No fill
        painter.drawRect(QRect(x0, y0, rect_size.width(), rect_size.height()))
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

                with open(image_filepath, "rb") as f:
                    image_content = f.read()

                points.update(
                    {
                        str(id): {
                            "x_meters": x,  # * self.resolution,
                            "y_meters": y,  # * self.resolution,
                            "yaw": yaw,
                            "mapfile": mapfile,
                            "image": image_filepath,
                            "image_bytes": image_content,
                        }
                    }
                )
            # print(f"{__name__} number of _points: {len(points)} {points}")
            self.database = DataBase(action="add_points", data=points, map_file=mapfile)
            self.database.action_completed.connect(self.database_task_completed)
            self.database.start()

        except Exception as identifier:
            pass

    def database_task_completed(self, x, y):
        print(x)
        pass

    def buffer_reference_image(self):
        print("SAVE IMAGE REFERENCE", f"aruco pose {self.current_aruco_pose}")
        # ntf = Notification(
        #     title="Accion en proceso... Guardando referencia",
        #     msg="Por favor no mueva el robot hasta guardar la referencia",
        #     preset=ToastPreset.INFORMATION,
        #     parent=self.parent,
        #     duration=8000,
        # )
        # ntf.show()
        dlg = CustomDialog(
            parent=self.parent,
            title="Accion en proceso... Guardando referencia",
            message="Por favor no mueva el robot hasta guardar la referencia",
            interative=False,
        )
        dlg.exec_()
        if self.pose_getter and self.pose_getter.isRunning():
            return

        self.pose_getter = TaskWorker()
        self.pose_getter.task_completed.connect(self.set_pose_and_image)
        self.pose_getter.start()

    def aruco_odom_callback(self, odom_msg):
        # Extract position
        self.current_aruco_pose.x = odom_msg.pose.pose.position.x
        self.current_aruco_pose.y = odom_msg.pose.pose.position.y

        # Extract orientation (convert quaternion to Euler angles)
        orientation_q = odom_msg.pose.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        ]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        self.current_aruco_pose.theta = yaw

    def set_pose_and_image(self, pose):
        if len(self.data_buffer) < 10:
            id = str(datetime.now().timestamp())
            img_file_path = f"./reference_images/reference_image{id}.jpg"
            aruco_pose = [
                self.current_aruco_pose.x,
                self.current_aruco_pose.y,
                self.current_aruco_pose.theta,
            ]
            self.data_buffer.append(
                (self.current_image, img_file_path, pose, aruco_pose)
            )

            # self.data_buffer2.append(pixmap)
            # print(f'{__name__} curent image shape', self.current_image.shape)
            self.send_buffered_data.emit(self.data_buffer)
            yaw = pose[2]
            quat_tuple = quaternion_from_euler(0, 0, yaw)
            data = PoseStamped()
            data.header.stamp = rospy.Time.now()
            data.header.frame_id = "map1"
            data.pose.position.x = pose[0]
            data.pose.position.y = pose[1]
            data.pose.position.z = 0
            data.pose.orientation.x = quat_tuple[0]
            data.pose.orientation.y = quat_tuple[1]
            data.pose.orientation.z = quat_tuple[2]
            data.pose.orientation.w = quat_tuple[3]
            self.buffered_data_pub.publish(data)

        print(f"{__name__} pose", pose)

        ntf = Notification(
            title="Accion completada con exito",
            msg="Se guardo foto de referencia exitosamente",
            preset=ToastPreset.SUCCESS,
            parent=self.parent,
        )

        ntf.show()

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
            return

        self.image_label.setText("No hay datos de la camara")

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
