"""Robot profile form: name, URDF path and ROS topic names, stored per-robot."""
import sys

import rospy
from config_model import UserConfigFileManager
from input_textdialog import CustomDialog, InputDialog
from PyQt5.QtCore import QSettings, Qt
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
    colored_button_style,
    secondary_button_style,
)


class RobotConfigForm(QWidget):
    """Add/edit/delete named robot profiles (URDF path + topic names),
    persisted via ``QSettings`` and selectable from a dropdown."""

    def __init__(self):
        super().__init__()
        self.user_config = UserConfigFileManager()
        self._config = QSettings("MyCompany", "MyApp")
        self.init_ui()
        self.init_params()

    def init_ui(self):
        """Build the robot dropdown, profile form and add/delete buttons."""
        # Window settings
        self.setWindowTitle("Configuración del Robot")
        # self.setGeometry(100, 100, 500, 300)  # x, y, width, height

        self.robots = self._config.value(
            "robots",
            {
                "turtlebot3": {
                    "name": "turtlebot3",
                    "URDF_path": "empty",
                    "odom_topic": "odom",
                    "cmd_vel_topic": "cmd_vel",
                    "lidar_topic": "scan",
                    "imu_topic": "imu",
                    "baseframe": "base_link",
                }
            },
        )

        # Main Layout
        main_layout = QVBoxLayout()
        buttons_layout = QHBoxLayout()
        delete_option_btn = QPushButton("Borrar robot")
        add_option_btn = QPushButton("+ Agregar Nuevo")
        delete_option_btn.setIcon(
            QApplication.style().standardIcon(QStyle.SP_DialogDiscardButton)
        )

        delete_option_btn.setStyleSheet(border_button_style_danger)
        add_option_btn.setStyleSheet(colored_button_style)

        buttons_layout.addWidget(delete_option_btn)
        buttons_layout.addWidget(add_option_btn)

        self.robots_dropdown = QComboBox()
        # Load Config
        self.config = self.user_config.read_data()

        robot_name = self._config.value("name", "turtlebot3")
        self.robots_dropdown.setCurrentText(robot_name)
        # self.robots_dropdown.setCurrentText(self._config.value('robot', 'turtlebot3'))

        # Title
        title_label = QLabel("Ingrese los detalles del robot")
        main_layout.addWidget(title_label)

        main_layout.addWidget(self.robots_dropdown)
        # Form Layout for inputs
        # main_layout.addStretch(0)

        form_layout = QFormLayout()

        # 1. Robot Name
        self.name_input = QLineEdit()
        self.name_input.setPlaceholderText("ej., TurtleBot3")
        form_layout.addRow("Nombre del robot:", self.name_input)

        # 2. URDF File Path (Custom layout for line edit + browse button)
        self.urdf_input = QLineEdit()
        self.urdf_input.setPlaceholderText("/ruta/al/modelo.urdf")

        browse_btn = QPushButton("Explorar...")
        browse_btn.clicked.connect(self.browse_file)

        urdf_layout = QHBoxLayout()
        urdf_layout.addWidget(self.urdf_input)
        urdf_layout.addWidget(browse_btn)

        form_layout.addRow("Ruta del archivo URDF:", urdf_layout)

        # 3. Odom Topic
        self.odom_input = QLineEdit()
        self.odom_input.setPlaceholderText("ej., /odom")
        form_layout.addRow("Nombre del tópico Odom:", self.odom_input)

        # 4. Command Vel Topic
        self.cmd_vel_input = QLineEdit()
        self.cmd_vel_input.setPlaceholderText("ej., /cmd_vel")
        form_layout.addRow("Nombre del tópico Cmd Vel:", self.cmd_vel_input)

        # 5. Lidar Topic
        self.lidar_input = QLineEdit()
        self.lidar_input.setPlaceholderText("ej., /scan")
        form_layout.addRow("Nombre del tópico Lidar:", self.lidar_input)

        # 3. Odom Topic
        self.imu_input = QLineEdit()
        self.imu_input.setPlaceholderText("ej., /imu")
        form_layout.addRow("Nombre del tópico IMU:", self.imu_input)

        self.baseframe_input = QLineEdit()
        self.baseframe_input.setPlaceholderText("ej., /base_link")
        form_layout.addRow("Nombre del marco base:", self.baseframe_input)

        # Add form layout to main layout
        main_layout.addLayout(form_layout)

        self.alert_label = QLabel("")
        main_layout.addWidget(self.alert_label)
        self.alert_label.setStyleSheet("color: red; font-weight: bold")

        main_layout.addLayout(buttons_layout)
        main_layout.addStretch(2)

        # Submit Button
        self.submit_btn = QPushButton("Guardar configuración")
        self.submit_btn.setStyleSheet(secondary_button_style)
        add_option_btn.clicked.connect(self.submit_form)
        delete_option_btn.clicked.connect(self.delete_robot)
        # main_layout.addWidget(self.submit_btn)

        # Set the layout
        self.setLayout(main_layout)

    def init_params(self):
        """Populate the dropdown from stored robot profiles and select the active one."""
        robots_options = list(self.robots.keys())

        for option in robots_options:
            self.robots_dropdown.addItem(option)

        robot_name = self._config.value("robot", "turtlebot3")
        self.update_param_view(robot_name=robot_name)

        self.robots_dropdown.currentIndexChanged.connect(self.handleDropDownRobotChange)

    def update_param_view(self, robot_name):
        """Fill the form fields from the stored profile for ``robot_name``."""
        robot = self.robots[robot_name]

        print("robot config", robot)

        self.name_input.setText(robot["name"])
        self.urdf_input.setText(robot["URDF_path"])
        self.odom_input.setText(robot["odom_topic"])
        self.cmd_vel_input.setText(robot["cmd_vel_topic"])
        self.lidar_input.setText(robot["lidar_topic"])
        self.imu_input.setText(robot["imu_topic"])
        self.baseframe_input.setText(robot["baseframe"])

    def browse_file(self):
        """Opens a file dialog to select the URDF file."""
        options = QFileDialog.Options()
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Seleccionar archivo URDF",
            "",
            "Archivos URDF (*.urdf);;Todos los archivos (*)",
            options=options,
        )
        if file_path:
            self.urdf_input.setText(file_path)

    def handleDropDownRobotChange(self):
        """Switch the active robot profile and persist the selection (requires app restart)."""
        robot = self.robots_dropdown.currentText()
        self.update_param_view(robot)

        dg = CustomDialog(
            self,
            f"Acabas de seleccionar {robot}!",
            message="Para ejecutar los cambios, cierre y abra la aplicacion nuevamente",
            interative=False,
            retries=0,
        )
        dg.exec_()

        # self.user_config.update_value("robot", robot)
        self._config.setValue("robot", robot)
        print("HI, HOLA! SE CAMBIO DE ROBot", robot)

    def submit_form(self):
        """Validate required fields and save the new robot profile."""
        name = self.name_input.text()
        data = {
            "name": self.name_input.text(),
            "URDF_path": self.urdf_input.text(),
            "lidar_topic": self.lidar_input.text(),
            "imu_topic": self.imu_input.text(),
            "odom_topic": self.odom_input.text(),
            "cmd_vel_topic": self.cmd_vel_input.text(),
            "baseframe": self.baseframe_input.text(),
        }

        if not data["name"]:
            self.alert_label.setText("Llene todos los campos, ingrese un nombre válido")
            return
        if not data["odom_topic"]:
            self.alert_label.setText(
                "Llene todos los campos, ingrese un tópico para la odometría"
            )
            return

        if not data["cmd_vel_topic"]:
            self.alert_label.setText(
                "Llene todos los campos, ingrese un tópico válido de cmd_vel"
            )
            return

        if not data["lidar_topic"]:
            self.alert_label.setText(
                "Llene todos los campos, ingrese un tópico válido para el lidar"
            )
            return

        self.alert_label.setText("")
        # self.user_config.update_value(f"robots.{name}", data)
        self.robots[name] = data
        self._config.setValue("robots", self.robots)
        print("all robots", self.robots)
        self.robots_dropdown.addItem(name)
        self.robots_dropdown.setCurrentText(name)

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
        """Confirm and delete the currently selected robot profile.

        Note: the confirm dialog's positive/negative labels are swapped
        ("Cancelar" is the positive response) and the actual deletion is
        not implemented below the confirmation check.
        """
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
