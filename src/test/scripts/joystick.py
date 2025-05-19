import sys
import asyncio
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QGroupBox, QPushButton
from PyQt5.QtCore import Qt, QPoint, QPointF
from PyQt5.QtGui import QPainter, QPen, QBrush, QColor, QPainterPath

from manual_control_node import ControlDynamicPose


class Joypad(QGroupBox):
    def __init__(self, parent=None):
        super().__init__("Control")
        size = 220
        self.setMaximumHeight(size)
        self.setMinimumHeight(size)
        self.setMaximumWidth(size)
        self.setMinimumWidth(size)
        self.joypad_radius = size*0.4
        self.thumb_radius = size*0.15
        self.center = QPointF(self.width()/2, self.height()/2 + 15)
        self.thumb_pos = self.center
        self.max_distance = self.joypad_radius - self.thumb_radius
        self.currentOperationMode = 'manual'
        self.x_value = 0.0
        self.y_value = 0.0
        self.pressed = False
        self.robot_vel_controller = ControlDynamicPose()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        painter.setPen(QPen(Qt.gray, 2))
        painter.setBrush(QBrush(QColor(220, 220, 220)))
        painter.drawEllipse(self.center, self.joypad_radius, self.joypad_radius)

        # Draw thumb stick
        if self.pressed:
            painter.setBrush(QBrush(QColor(100, 150, 255)))
        else:
            # self.robot_vel_controller.setRobotDynamicPose(0,0)
            painter.setBrush(QBrush(QColor(150, 150, 150)))
        painter.drawEllipse(self.thumb_pos, self.thumb_radius, self.thumb_radius)
        super().paintEvent(event)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.pressed = True
            self.moveThumb(event.pos())
            self.update()
            super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self.pressed:
            self.moveThumb(event.pos())
            self.update()
            super().mouseMoveEvent(event)
            # print('size: ',self.width(), self.height())
            # print('center: ', self.center.x(), self.center.y())

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.pressed = False
            self.thumb_pos = self.center
            self.x_value = 0.0
            self.y_value = 0.0
            self.update()
            self.valueChanged(0.0, 0.0)
            self.robot_vel_controller.setRobotDynamicPose(0,0)
            super().mouseReleaseEvent(event)

    def moveThumb(self, pos):
        # Calculate vector from center to mouse position
        vector = QPointF(pos) - self.center
        distance = (vector.x() ** 2 + vector.y() ** 2) ** 0.5

        # If mouse is outside max distance, limit the thumb position
        if distance > self.max_distance:
            scale = self.max_distance / distance
            vector = QPointF(vector.x() * scale, vector.y() * scale)

        self.thumb_pos = self.center + vector

        # Calculate normalized values (-1.0 to 1.0)
        self.x_value = vector.x() / self.max_distance
        self.y_value = (
            -vector.y() / self.max_distance
        )  # Invert Y for more intuitive up=positive

        # self.valueChanged(self.x_value, self.y_value)
        self.robot_vel_controller.setRobotDynamicPose(self.x_value, self.y_value)

    def valueChanged(self, x, y):
        pass

    def update_operation_mode(self, mode):
        if mode == 'auto':
            print('auto joystick')
        pass


class JoypadDemo(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle("Qt Joypad Demo")
        # self.setGeometry(100, 100, 300, 350)

        layout = QVBoxLayout()

        self.joypad = Joypad()
        self.joypad.valueChanged = self.onJoypadChanged

        self.label = QLabel("Joypad Values: X: 0.00, Y: 0.00")
        self.label.setAlignment(Qt.AlignCenter)

        layout.addWidget(self.joypad)
        layout.addWidget(self.label)
        self.setLayout(layout)

    def onJoypadChanged(self, x, y):
        self.label.setText(f"Joypad Values: X: {x:.2f}, Y: {y:.2f}")


# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     demo = JoypadDemo()
#     demo.show()
#     sys.exit(app.exec_())
