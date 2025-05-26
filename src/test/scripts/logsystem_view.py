from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLineEdit,
    QPushButton,
    QCheckBox,
    QGroupBox,
    QLabel,
    QMessageBox,
)
from PyQt5.QtCore import Qt, QPropertyAnimation, QEasingCurve, QTimer
from PyQt5.QtGui import QPixmap
import random
import sys
from io import StringIO
from rview import MyViz

# import pyqtgraph as pg


class TodoItem(QWidget):
    def __init__(self, text, parent=None):
        super().__init__(parent)
        self.parent = parent

        # Create widgets for the todo item
        self.checkbox = QCheckBox()
        self.label = QLabel(text)
        self.delete_button = QPushButton("×")
        self.info_button = QPushButton("info")
        self.delete_button.setFixedSize(30, 30)
        self.info_button.setFixedSize(60, 30)
        errorstyle = """
            QLabel {
                color: #c92a2a;
                background-color: #ffc9c9;
                border: 1px solid #ff8787;
                border-radius: 4px;
                padding: 8px;
                font-size: 14px;
            }
        """
        infostyle = """
            QLabel {
                color: #1864ab;
                background-color: #d0ebff;
                border: 1px solid #74c0fc;
                border-radius: 4px;
                padding: 8px;
                font-size: 14px;
            }
        """

        # Warning message style (orange/yellow)
        warningstyle = """
            QLabel {
                color: #e67700;
                background-color: #fff3bf;
                border: 1px solid #ffd43b;
                border-radius: 4px;
                padding: 8px;
                font-size: 14px;
            }
        """

        # Layout for the todo item
        self.label.setStyleSheet(random.choice([warningstyle, errorstyle, infostyle]))
        layout = QHBoxLayout()
        layout.addWidget(self.checkbox)
        layout.addWidget(self.label)
        layout.addStretch()
        layout.addWidget(self.info_button)
        layout.addWidget(self.delete_button)

        self.setLayout(layout)

        # Connect signals
        self.checkbox.stateChanged.connect(self.toggle_completed)
        self.delete_button.clicked.connect(self.delete_item)

    def toggle_completed(self, state):
        if state == Qt.Checked:
            pass
        else:
            pass

    def delete_item(self):
        if self.parent:
            self.parent.remove_todo_item(self)


class DisplayInfo(QGroupBox):
    def __init__(self) -> None:
        super().__init__()
        pixmap = QPixmap("./mora1.png")
        self.image_label = QLabel()
        layout = QVBoxLayout(self)
        # layout.setAligment(Qt.AlignTop)
        load_button = QPushButton("Load Image")
        # self.setStyleSheet("background-color: gray")
        # load_button.clicked.connect(self.load_image)
        # pg.plot([1, 2, 3, 4])   # data can be a list of values or a numpy array

        # Status label
        self.status_label = QLabel(
            "No image loaded, mucho que ddkebkjbfkebfw ekfjwekfbwefkwebfe jkjkbhbjhjhjhv ejdbkwebfkwjbefkwebf"
        )

        if pixmap.isNull():
            self.status_label.setText("Failed to load image")
            return

        # Scale the image to fit the label while maintaining aspect ratio
        self.image_label.setPixmap(
            pixmap.scaled(
                self.image_label.width(),
                self.image_label.height(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
        )
        # self.image_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.rviz = MyViz(configfile='./config_navigation.rviz')
        self.rviz.setUp('globalframe', 'base_footprint')
        layout.addWidget(self.rviz)
        layout.addWidget(self.status_label, alignment=Qt.AlignTop)
        layout.addWidget(load_button)


class LogPanel(QWidget):
    def __init__(self, node_manager=None):
        super().__init__()
        self.layout = QVBoxLayout(self)
        self.main_layout = QHBoxLayout()
        self.setup_input_area()
        self.setup_todo_list()
        self.panel = DisplayInfo()
        self.layout.addLayout(self.main_layout)
        self.toggle = True
        self.node_manager = node_manager
        # rosbag record -O subset /turtle1/cmd_vel /turtle1/pose
        # <node pkg="rosbag" type="play" name="rosbag" required="true" args="$(find image_view)/test.bag"/>
        # self.nodes = [{"name": "rosbag", "package": "rosbag", "exec": "record", 'args': '/scan'}]
        self.node_manager.bringUpStart()
        # self.node_manager.startNodes(self.node_manager.initNodes(self.nodes))
        # self.node_manager.recordSensordata(['/scan'])

        self.timer = QTimer()
        self.timer.setSingleShot(True)
        self.timer.timeout.connect(self.checkAlerts)
        self.timer.start(7000)

        self.main_layout.addWidget(self.panel, 2)
        self.panel.hide()

    def checkAlerts(self):
        # self.node_manager.stopNodes(['rosbag'])
        # self.node_manager.stopRecordSensorData()
        # old_stderr = sys.stderr
        # print(old_stderr)
        # captured_stderr = StringIO()
        # error_message = captured_stderr.getvalue()
        # print(f"Captured error jiji: {error_message}")
        pass

    def setup_input_area(self):
        input_layout = QHBoxLayout()

        add_button = QPushButton("Add")
        add_button.clicked.connect(self.add_todo)

        input_layout.addWidget(add_button)
        self.layout.addLayout(input_layout)

    def setup_todo_list(self):
        self.todo_container = QWidget()
        self.todo_layout = QVBoxLayout(self.todo_container)
        self.todo_layout.setSpacing(5)

        # Add stretch to push items to the top
        self.todo_layout.addStretch()

        self.main_layout.addWidget(self.todo_container)

    def add_todo(self):
        if self.toggle:
            self.panel.show()
        else:
            self.panel.hide()
        self.toggle = not self.toggle

        text = "ERROR: Obstaculo encontrado 1/23/2025 23:23"
        if text:
            todo_item = TodoItem(text, self)
            self.todo_layout.insertWidget(self.todo_layout.count() - 1, todo_item)

    def remove_todo_item(self, item):
        self.todo_layout.removeWidget(item)
        item.deleteLater()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = LogPanel()
    window.show()
    sys.exit(app.exec_())
