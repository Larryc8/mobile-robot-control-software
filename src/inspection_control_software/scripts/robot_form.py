import sys

import rospy
from config_model import UserConfigFileManager
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QFileDialog,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QStyle,
    QVBoxLayout,
    QWidget,
)
from styles.buttons import (
    border_button_style,
    border_button_style_danger,
    secondary_button_style,
)


class RobotConfigForm(QWidget):
    def __init__(self):
        super().__init__()
        self.user_config = UserConfigFileManager()
        self.init_ui()

    def init_ui(self):
        # Window settings
        self.setWindowTitle("Robot Configuration Setup")
        # self.setGeometry(100, 100, 500, 300)  # x, y, width, height

        # Main Layout
        main_layout = QVBoxLayout()
        buttons_layout = QHBoxLayout()
        delete_option_btn = QPushButton("Borrar robot")
        add_option_btn = QPushButton("+ Agregar Nuevo")
        delete_option_btn.setIcon(
            QApplication.style().standardIcon(QStyle.SP_DialogDiscardButton)
        )

        delete_option_btn.setStyleSheet(border_button_style_danger)
        add_option_btn.setStyleSheet(border_button_style)

        buttons_layout.addWidget(delete_option_btn)
        buttons_layout.addWidget(add_option_btn)

        self.country_dropdown = QComboBox()
        # self.country_dropdown.currentIndexChanged.connect(self.on_country_changed)

        # Add countries with custom data (country codes)
        robots_options = [
            ("Turtlebot3", "US"),
            ("Canada", "CA"),
        ]

        for country, code in robots_options:
            self.country_dropdown.addItem(country, code)

        # Title
        title_label = QLabel("Enter Robot Details")
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("font-size: 18px; font-weight: bold;")

        # main_layout.addWidget(title_label)

        main_layout.addWidget(self.country_dropdown)
        main_layout.addLayout(buttons_layout)
        # Form Layout for inputs
        # main_layout.addStretch(0)

        form_layout = QFormLayout()

        # 1. Robot Name
        self.name_input = QLineEdit()
        self.name_input.setPlaceholderText("e.g., TurtleBot3")
        form_layout.addRow("Robot Name:", self.name_input)

        # 2. URDF File Path (Custom layout for line edit + browse button)
        self.urdf_input = QLineEdit()
        self.urdf_input.setPlaceholderText("/path/to/model.urdf")

        browse_btn = QPushButton("Browse...")
        browse_btn.clicked.connect(self.browse_file)

        urdf_layout = QHBoxLayout()
        urdf_layout.addWidget(self.urdf_input)
        urdf_layout.addWidget(browse_btn)

        form_layout.addRow("URDF File Path:", urdf_layout)

        # 3. Odom Topic
        self.odom_input = QLineEdit()
        self.odom_input.setPlaceholderText("e.g., /odom")
        form_layout.addRow("Odom Topic Name:", self.odom_input)

        # 4. Command Vel Topic
        self.cmd_vel_input = QLineEdit()
        self.cmd_vel_input.setPlaceholderText("e.g., /cmd_vel")
        form_layout.addRow("Cmd Vel Topic Name:", self.cmd_vel_input)

        # 5. Lidar Topic
        self.lidar_input = QLineEdit()
        self.lidar_input.setPlaceholderText("e.g., /scan")
        form_layout.addRow("Lidar Topic Name:", self.lidar_input)

        # Add form layout to main layout
        main_layout.addLayout(form_layout)

        # Submit Button
        self.submit_btn = QPushButton("Save Configuration")
        self.submit_btn.setStyleSheet(secondary_button_style)
        self.submit_btn.clicked.connect(self.submit_form)
        main_layout.addWidget(self.submit_btn)

        # Set the layout
        self.setLayout(main_layout)

    def browse_file(self):
        """Opens a file dialog to select the URDF file."""
        options = QFileDialog.Options()
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select URDF File",
            "",
            "URDF Files (*.urdf);;All Files (*)",
            options=options,
        )
        if file_path:
            self.urdf_input.setText(file_path)

    def submit_form(self):
        """Collects data and displays it."""
        data = {
            "Robot Name": self.name_input.text(),
            "URDF Path": self.urdf_input.text(),
            "Odom Topic": self.odom_input.text(),
            "Cmd Vel Topic": self.cmd_vel_input.text(),
            "Lidar Topic": self.lidar_input.text(),
        }

        # Validation: Check if Robot Name is empty
        if not data["Robot Name"]:
            QMessageBox.warning(self, "Input Error", "Please enter a Robot Name.")
            return

        # Print to console (for debugging/logging)
        print("\n--- Form Submitted ---")
        for key, value in data.items():
            print(f"{key}: {value}")


if __name__ == "__main__":
    app = QApplication(sys.argv)

    # Apply a standard style (Fusion is generally available across plaforms)
    app.setStyle("Fusion")

    form = RobotConfigForm()
    form.show()

    sys.exit(app.exec_())
