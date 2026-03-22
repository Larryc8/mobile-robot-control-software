import sys

import rospy
from config_model import UserConfigFileManager
from input_textdialog import CustomDialog, InputDialog
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

        self.robots_dropdown = QComboBox()
        # Load Config
        self.config = self.user_config.read_data()

        robots_options = list(self.config["robots"].keys())

        for option in robots_options:
            self.robots_dropdown.addItem(option)

        self.robots_dropdown.setCurrentText(self.config["robot"])

        self.robots_dropdown.currentIndexChanged.connect(self.handleDropDownRobotChange)

        # Title
        title_label = QLabel("Enter Robot Details")
        main_layout.addWidget(title_label)

        main_layout.addWidget(self.robots_dropdown)
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

        # 3. Odom Topic
        self.imu_input = QLineEdit()
        self.imu_input.setPlaceholderText("e.g., /imu")
        form_layout.addRow("Imu Topic Name:", self.imu_input)

        self.baseframe_input = QLineEdit()
        self.baseframe_input.setPlaceholderText("e.g., /base_link")
        form_layout.addRow("Base Frame Name:", self.baseframe_input)

        # Add form layout to main layout
        main_layout.addLayout(form_layout)

        self.alert_label = QLabel("")
        main_layout.addWidget(self.alert_label)
        self.alert_label.setStyleSheet("color: red; font-weight: bold")

        main_layout.addLayout(buttons_layout)
        main_layout.addStretch(2)

        # Submit Button
        self.submit_btn = QPushButton("Save Configuration")
        self.submit_btn.setStyleSheet(secondary_button_style)
        add_option_btn.clicked.connect(self.submit_form)
        delete_option_btn.clicked.connect(self.delete_robot)
        # main_layout.addWidget(self.submit_btn)

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

    def handleDropDownRobotChange(self):
        robot = self.robots_dropdown.currentText()

        dg = CustomDialog(
            self,
            f"Acabas de seleccionar {robot}!",
            message="Para ejecutar los cambios, cierre y abra la aplicacion nuevamente",
            interative=False,
            retries=0,
        )
        dg.exec_()

        self.user_config.update_value("robot", robot)
        print("HI, HOLA! SE CAMBIO DE ROBot", robot)

    def submit_form(self):
        """Collects data and displays it."""
        name = self.name_input.text()
        data = {
            "robot": self.name_input.text(),
            "URDF_path": self.urdf_input.text(),
            "odom_topic": self.odom_input.text(),
            "cmd_vel_topic": self.cmd_vel_input.text(),
            "lidar_topic": self.lidar_input.text(),
            "imu_topic": self.imu_input.text(),
            "baseframe": self.baseframe_input.text(),
        }

        if not data["robot"]:
            self.alert_label.setText("Llene todos los campos, ingrese un nombre valido")
            return
        if not data["odom_topic"]:
            self.alert_label.setText(
                "Llene todos los campos, ingrese un topico para la odometria"
            )
            return

        if not data["cmd_vel_topic"]:
            self.alert_label.setText(
                "Llene todos los campos, ingrese un topico valido de cmd_vel"
            )
            return

        if not data["lidar_topic"]:
            self.alert_label.setText(
                "Llene todos los campos, ingrese un topico valido para el lidar"
            )
            return

        self.alert_label.setText("")
        self.user_config.update_value(f"robots.{name}", data)
        self.robots_dropdown.addItem(name)

        dg = CustomDialog(
            self,
            "Quieres agregar un nuevo robot?",
            message="Quedará guardado de forma <span style={font-weight: bold}>permanente</span>",
            positive_response="Guardar",
            negative_response="Descartar",
            retries=1,
        )
        dg.exec_()

        # Print to console (for debugging/logging)
        print("\n--- Form Submitted ---")
        print(data)

    def delete_robot(self):
        robot = self.robots_dropdown.currentText()
        dg = CustomDialog(
            self,
            f"Quieres eliminar: {robot}?",
            message="Se eliminara de forma <span style={font-weight: bold}>permanente</span>",
            positive_response="Cancelar",
            negative_response="Eliminar",
            retries=1,
        )
        dg.exec_()

        if dg.response == "Positive":
            return

        pass


if __name__ == "__main__":
    app = QApplication(sys.argv)

    # Apply a standard style (Fusion is generally available across plaforms)
    app.setStyle("Fusion")

    form = RobotConfigForm()
    form.show()

    sys.exit(app.exec_())
