import sys

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QHBoxLayout,
    QLabel,
    QLCDNumber,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)


class RobotVelocityController(QWidget):
    def __init__(self):
        super().__init__()

        # Initialize variables
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.path_tolerance = 0.10

        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("Robot Teleop Controller")
        self.setGeometry(100, 100, 400, 600)  # Increased height to fit new elements

        # Main Layout
        main_layout = QVBoxLayout()

        # Title
        title_label = QLabel("Velocity Control Panel")
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet(
            "font-size: 16px; font-weight: bold; margin-bottom: 10px;"
        )
        main_layout.addWidget(title_label)

        # --- Linear Velocity Section ---
        linear_layout = QVBoxLayout()
        linear_label = QLabel("Linear Velocity (m/s)")
        linear_layout.addWidget(linear_label)

        # LCD Display for Linear
        self.lcd_linear = QLCDNumber()
        self.lcd_linear.setSegmentStyle(QLCDNumber.Flat)
        linear_layout.addWidget(self.lcd_linear)

        # Slider for Linear
        self.slider_linear = QSlider(Qt.Horizontal)
        self.slider_linear.setMinimum(-100)  # Represents -1.0 m/s
        self.slider_linear.setMaximum(100)  # Represents 1.0 m/s
        self.slider_linear.setValue(0)
        self.slider_linear.valueChanged.connect(self.update_linear_vel)
        linear_layout.addWidget(self.slider_linear)

        # main_layout.addLayout(linear_layout)
        # main_layout.addSpacing(15)

        # --- Angular Velocity Section ---
        angular_layout = QVBoxLayout()
        angular_label = QLabel("Angular Velocity (rad/s)")
        angular_layout.addWidget(angular_label)

        # LCD Display for Angular
        self.lcd_angular = QLCDNumber()
        self.lcd_angular.setSegmentStyle(QLCDNumber.Flat)
        angular_layout.addWidget(self.lcd_angular)

        # Slider for Angular
        self.slider_angular = QSlider(Qt.Horizontal)
        self.slider_angular.setMinimum(-200)  # Represents -2.0 rad/s
        self.slider_angular.setMaximum(200)  # Represents 2.0 rad/s
        self.slider_angular.setValue(0)
        self.slider_angular.valueChanged.connect(self.update_angular_vel)
        angular_layout.addWidget(self.slider_angular)

        # main_layout.addLayout(angular_layout)
        # main_layout.addSpacing(15)

        # --- Conduction Mode Section ---
        mode_layout = QVBoxLayout()
        mode_title = QLabel("Conduction Mode")
        mode_title.setStyleSheet("font-weight: bold;")
        mode_title.setAlignment(Qt.AlignCenter)
        mode_layout.addWidget(mode_title)

        # Buttons layout
        buttons_layout = QHBoxLayout()

        self.btn_soft = QPushButton("Soft")
        self.btn_soft.clicked.connect(lambda: self.update_mode("Soft"))
        buttons_layout.addWidget(self.btn_soft)

        self.btn_medium = QPushButton("Medium")
        self.btn_medium.clicked.connect(lambda: self.update_mode("Medium"))
        buttons_layout.addWidget(self.btn_medium)

        self.btn_aggressive = QPushButton("Aggressive")
        self.btn_aggressive.setStyleSheet("color: red;")
        self.btn_aggressive.clicked.connect(lambda: self.update_mode("Aggressive"))
        buttons_layout.addWidget(self.btn_aggressive)

        mode_layout.addLayout(buttons_layout)

        # Description Label
        self.mode_description = QLabel("Select a mode to see impact.")
        self.mode_description.setAlignment(Qt.AlignCenter)
        self.mode_description.setWordWrap(True)
        self.mode_description.setStyleSheet(
            "color: gray; font-style: italic; margin-top: 5px;"
        )
        mode_layout.addWidget(self.mode_description)

        main_layout.addLayout(mode_layout)
        main_layout.addSpacing(15)

        # --- Navigation Parameters Section (New) ---
        nav_layout = QVBoxLayout()
        nav_title = QLabel("Navigation Parameters")
        nav_title.setStyleSheet(
            "font-weight: bold; border-top: 1px solid #ccc; padding-top: 10px;"
        )
        nav_title.setAlignment(Qt.AlignCenter)
        nav_layout.addWidget(nav_title)

        # Stuck Timeout Dropdown
        timeout_layout = QHBoxLayout()
        timeout_label = QLabel("Stuck Timeout:")
        self.timeout_combo = QComboBox()
        self.timeout_combo.addItems(
            ["5 seconds", "10 seconds", "20 seconds", "30 seconds", "60 seconds"]
        )
        self.timeout_combo.setCurrentIndex(1)  # Default to 10s
        self.timeout_combo.currentTextChanged.connect(self.update_timeout)

        timeout_layout.addWidget(timeout_label)
        timeout_layout.addWidget(self.timeout_combo)
        nav_layout.addLayout(timeout_layout)

        # Path Tolerance Slider
        tolerance_layout = QVBoxLayout()
        tolerance_header = QHBoxLayout()
        tolerance_label = QLabel("Path Tolerance (m):")

        self.lcd_tolerance = QLCDNumber()
        self.lcd_tolerance.setSegmentStyle(QLCDNumber.Flat)
        self.lcd_tolerance.display(self.path_tolerance)

        tolerance_header.addWidget(tolerance_label)
        tolerance_header.addWidget(self.lcd_tolerance)
        tolerance_layout.addLayout(tolerance_header)

        self.slider_tolerance = QSlider(Qt.Horizontal)
        self.slider_tolerance.setMinimum(1)  # Represents 0.01 m
        self.slider_tolerance.setMaximum(100)  # Represents 1.00 m
        self.slider_tolerance.setValue(10)  # Represents 0.10 m
        self.slider_tolerance.valueChanged.connect(self.update_tolerance)
        tolerance_layout.addWidget(self.slider_tolerance)

        nav_layout.addLayout(tolerance_layout)
        main_layout.addLayout(nav_layout)
        main_layout.addSpacing(20)

        # --- Emergency Stop Button ---
        self.stop_btn = QPushButton("EMERGENCY STOP")
        self.stop_btn.setStyleSheet(
            "background-color: red; color: white; font-weight: bold; height: 40px;"
        )
        self.stop_btn.clicked.connect(self.stop_robot)
        # main_layout.addWidget(self.stop_btn)

        self.setLayout(main_layout)

    def update_linear_vel(self, value):
        self.linear_vel = value / 100.0
        self.lcd_linear.display(self.linear_vel)
        self.publish_cmd_vel()

    def update_angular_vel(self, value):
        self.angular_vel = value / 100.0
        self.lcd_angular.display(self.angular_vel)
        self.publish_cmd_vel()

    def update_mode(self, mode):
        if mode == "Soft":
            text = "Impact: Low acceleration/speed. Safe for crowded areas."
            style = "color: green;"
        elif mode == "Medium":
            text = "Impact: Balanced speed. Standard behavior."
            style = "color: blue;"
        elif mode == "Aggressive":
            text = "Impact: High torque/speed. Faster response."
            style = "color: darkred; font-weight: bold;"

        self.mode_description.setText(text)
        self.mode_description.setStyleSheet(style)
        print(f"Mode changed to: {mode}")

    def update_timeout(self, text):
        print(f"Stuck Timeout updated to: {text}")
        # Add logic here to update the robot's navigation parameter

    def update_tolerance(self, value):
        self.path_tolerance = value / 100.0
        self.lcd_tolerance.display(self.path_tolerance)
        print(f"Path Tolerance updated to: {self.path_tolerance} m")
        # Add logic here to update the robot's navigation parameter

    def stop_robot(self):
        self.slider_linear.setValue(0)
        self.slider_angular.setValue(0)

    def publish_cmd_vel(self):
        print(
            f"Publishing -> Linear: {self.linear_vel:.2f} m/s | Angular: {self.angular_vel:.2f} rad/s"
        )


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    controller = RobotVelocityController()
    controller.show()
    sys.exit(app.exec_())
