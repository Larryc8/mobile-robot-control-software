import sys

import rospy
from config_model import UserConfigFileManager
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
from styles.buttons import dropdown_style, secondary_button_style, tertiary_button_style
from utils.custom_toolbutton import CustomToolButtom

# Dios mio dame m*a


class RobotVelocityController(QWidget):
    def __init__(self):
        super().__init__()

        # Initialize variables
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.PATH_TOLERANCE = 0.10
        self.user_config_hanler = UserConfigFileManager()
        config = self.user_config_hanler.read_data()

        self.init_ui()

        self.update_mode(config["conduction_mode"])
        self.init_tolerance(config["path_tolerance"] * 100)
        self.timeout_combo.setCurrentIndex(config["stuck_timeout_index"])

    def init_ui(self):
        # Main Layout
        main_layout = QVBoxLayout()
        info_label_style = "color: gray; font-style: italic; margin-top: 5px;"

        # Title
        title_label = QLabel("Panel de Control")
        title_label.setStyleSheet("font-size: 16px; font-weight: bold; ")
        main_layout.addWidget(title_label)

        # --- Linear Velocity Section ---

        # main_layout.addLayout(linear_layout)
        # main_layout.addSpacing(15)

        # main_layout.addLayout(angular_layout)
        # main_layout.addSpacing(15)

        # --- Conduction Mode Section ---
        mode_layout = QVBoxLayout()
        mode_title = QLabel("Modo de Conducción")
        mode_title.setStyleSheet("font-weight: bold;")
        mode_layout.addWidget(mode_title)

        # Buttons layout
        buttons_layout = QHBoxLayout()
        buttons_layout.setSpacing(40)
        # buttons_layout.setContentsMargins(0, 0, 0, 0)
        buttons_layout.setAlignment(Qt.AlignLeft)

        self.btn_soft = CustomToolButtom(
            "Suave", icon="./public/speed_low.svg", size=60
        )
        self.btn_soft.clicked.connect(lambda: self.update_mode("Soft"))
        buttons_layout.addWidget(self.btn_soft, 2)

        self.btn_medium = CustomToolButtom(
            "Media", icon="./public/speed_medium.svg", size=60
        )
        self.btn_medium.clicked.connect(lambda: self.update_mode("Medium"))
        buttons_layout.addWidget(self.btn_medium)

        self.btn_aggressive = CustomToolButtom(
            "Agresivo", icon="./public/speed_high.svg", size=60
        )
        self.btn_aggressive.setStyleSheet("color: red;")
        self.btn_aggressive.clicked.connect(lambda: self.update_mode("Aggressive"))
        buttons_layout.addWidget(self.btn_aggressive)

        mode_layout.addLayout(buttons_layout)

        # Description Label
        self.mode_description = QLabel("Seleccione un modo para ver su impacto.")
        self.mode_description.setWordWrap(True)
        self.mode_description.setStyleSheet(info_label_style)
        mode_layout.addWidget(self.mode_description)

        main_layout.addLayout(mode_layout)
        # main_layout.addSpacing(15)

        # --- Navigation Parameters Section (New) ---
        nav_layout = QVBoxLayout()
        nav_title = QLabel("Parámetros de Navegación")
        nav_title.setStyleSheet(
            "font-weight: bold; border-top: 1px solid #ccc; padding-top: 10px;"
        )
        nav_layout.addWidget(nav_title)

        self.timeout_items = [
            ("5 segundos", 5),
            ("10 segundos", 10),
            ("20 segundos", 20),
            ("30 segundos", 30),
            ("60 segundos", 60),
        ]

        # Stuck Timeout Dropdown
        timeout_layout = QHBoxLayout()
        timeout_label = QLabel("Tiempo límite de bloqueo:")
        self.timeout_combo = QComboBox()
        # self.timeout_combo.setStyleSheet(dropdown_style)

        for text, code in self.timeout_items:
            self.timeout_combo.addItem(text)

        self.timeout_combo.setCurrentIndex(1)  # Default to 10s
        self.timeout_combo.currentTextChanged.connect(self.update_timeout)

        timeout_layout.addWidget(timeout_label)
        timeout_layout.addWidget(self.timeout_combo)
        nav_layout.addLayout(timeout_layout)

        # Path Tolerance Slider
        tolerance_layout = QVBoxLayout()
        tolerance_header = QHBoxLayout()
        tolerance_label = QLabel("Tolerancia de ruta (m):")

        self.lcd_tolerance = QLCDNumber()
        self.lcd_tolerance.setSegmentStyle(QLCDNumber.Flat)
        self.lcd_tolerance.display(self.PATH_TOLERANCE)

        tolerance_header.addWidget(tolerance_label)
        tolerance_header.addWidget(self.lcd_tolerance)
        tolerance_layout.addLayout(tolerance_header)

        self.slider_tolerance = QSlider(Qt.Horizontal)
        self.slider_tolerance.setMinimum(1)  # Represents 0.01 m
        self.slider_tolerance.setMaximum(100)  # Represents 1.00 m
        self.slider_tolerance.setValue(10)  # Represents 0.10 m
        self.slider_tolerance.valueChanged.connect(self.update_tolerance)
        tolerance_layout.addWidget(self.slider_tolerance)
        x = QLabel("Valores alto implican una mayor sensidbiliada")
        tolerance_layout.addWidget(x)

        nav_layout.addLayout(tolerance_layout)
        main_layout.addLayout(nav_layout)
        main_layout.addSpacing(20)

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
            text = "Impacto: Baja aceleración/velocidad. Seguro para áreas concurridas."
            if not self.btn_soft.isSelected():
                self.btn_soft.toggle_selected()
                rospy.set_param("/max_linear_velocity", 0.1)
                rospy.set_param("/max_angular_velocity", 0.3)
            if self.btn_medium.isSelected():
                self.btn_medium.toggle_selected()
            if self.btn_aggressive.isSelected():
                self.btn_aggressive.toggle_selected()
        elif mode == "Medium":
            text = "Impacto: Velocidad equilibrada. Comportamiento estándar."
            if not self.btn_medium.isSelected():
                self.btn_medium.toggle_selected()
                rospy.set_param("/max_linear_velocity", 0.2)
                rospy.set_param("/max_angular_velocity", 0.5)
            if self.btn_soft.isSelected():
                self.btn_soft.toggle_selected()
            if self.btn_aggressive.isSelected():
                self.btn_aggressive.toggle_selected()
        elif mode == "Aggressive":
            text = "Impacto: Alto torque/velocidad. Respuesta más rápida."
            if not self.btn_aggressive.isSelected():
                self.btn_aggressive.toggle_selected()
                rospy.set_param("/max_linear_velocity", 0.4)
                rospy.set_param("/max_angular_velocity", 0.8)
            if self.btn_medium.isSelected():
                self.btn_medium.toggle_selected()
            if self.btn_soft.isSelected():
                self.btn_soft.toggle_selected()

        self.user_config_hanler.update_value("conduction_mode", mode)

        self.mode_description.setText(text)
        print(f"Mode changed to: {mode}")

    def update_timeout(self, text):
        print(f"Stuck Timeout updated to: {text}")
        index = self.timeout_combo.findText(text)
        self.user_config_hanler.update_value("stuck_timeout_index", index)

        # Add logic here to update the robot's navigation parameter

    def update_tolerance(self, value):
        self.PATH_TOLERANCE = value / 100.0
        self.lcd_tolerance.display(self.PATH_TOLERANCE)
        print(f"Path Tolerance updated to: {self.PATH_TOLERANCE} m")
        rospy.set_param("/path_tolerance", self.PATH_TOLERANCE)
        self.user_config_hanler.update_value("path_tolerance", self.PATH_TOLERANCE)
        # Add logic here to update the robot's navigation parameter

    def init_tolerance(self, value):
        self.slider_tolerance.setValue(value)
        self.update_tolerance(value)

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
