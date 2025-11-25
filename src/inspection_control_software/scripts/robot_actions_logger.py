import csv
import datetime
import os
import sys
import typing
from ctypes import alignment
from datetime import datetime

import rospy
from config_model import UserConfigFileManager
from PyQt5.QtCore import QObject, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QDesktopServices, QFont, QIcon, QPalette, QTextCursor
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)
from sensor_msgs.msg import BatteryState
from styles.buttons import border_button_style, secondary_button_style


class CsvHandler:
    """
    A class to handle creating and writing to a CSV file.

    It creates the file with a specified header only if the file
    does not already exist.
    """

    def __init__(self, filepath, header):
        """
        Initializes the CsvHandler with a file path and header.

        Args:
            filepath (str): The path to the CSV file.
            header (list): A list of strings for the CSV header.
        """
        self.filepath = filepath
        self.header = header
        self._create_file_if_not_exists()

    def _create_file_if_not_exists(self):
        """
        Checks if the file exists. If not, creates it and writes the header.
        This is a private method, intended for internal use by the class.
        """
        # os.path.exists() checks if a file or directory exists at the path
        if not os.path.exists(self.filepath):
            print(f"File '{self.filepath}' not found. Creating it now... ✍️")
            with open(self.filepath, mode="w", newline="") as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow(self.header)  # Write the header row
        else:
            print(f"File '{self.filepath}' already exists.")

    def append_row(self, row_data):
        """
        Appends a single row of data to the CSV file.

        Args:
            row_data (list): A list of values for the new row.
        """
        if len(row_data) != len(self.header):
            print("Error: Row data does not match header length.")
            return

        # 'a' mode stands for append
        with open(self.filepath, mode="a", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(row_data)
        print(f"Appended row: {row_data}")


# from PyQt5.QtGui import QDesktopServices
# from PyQt5.QtCore import QUrl
# import os
#
# # ... (your PyQt application setup)
#
# file_path = "path/to/your/document.pdf"  # Replace with your file path
# if os.path.exists(file_path):
#     url = QUrl.fromLocalFile(file_path)
#     QDesktopServices.openUrl(url)
# else:
#     print("File not found:", file_path)
#
#
#
# options = QFileDialog.Options()
# You can use QFileDialog.DontUseNativeDialog if you prefer
# options |= QFileDialog.DontUseNativeDialog

# The method returns a tuple: (fileName, filter)


def add_color(msg):
    if isinstance(msg, bool):
        return f"<span style='color:#196F3D;font-weight: bold'>bool {msg}</span>"
    if isinstance(msg, str):
        return f"<span style='color:#196F3D;font-weight: bold'>str {msg}</span>"
    if isinstance(msg, (int, float)):
        return f"<span style='color:#633974;font-weight: bold'>num {msg}</span>"


class FixedMessage(QGroupBox):
    log_file_updated = pyqtSignal(str)

    def __init__(self) -> None:
        super().__init__()
        self.setStyleSheet("""
            QGroupBox {
                background-color: #F5F5F5;
                border: 2px dashed blue;
                border-radius: 4px;
                font-style: italic;
                padding: 0px;
            }
        """)

        # self.setContentsMargins(0, 0, 0, 0)
        self.user_config = UserConfigFileManager("./config/app_config.json")
        self.setFlat(True)
        config = self.user_config.read_data()

        layout = QHBoxLayout()
        self.text = QLabel(
            f"Para ver el historil revise <span style='color: royalblue; text-decoration: underline'>{config['log_history_filepath']}</span>"
        )

        self.text.setTextInteractionFlags(Qt.TextSelectableByMouse)

        layout.setContentsMargins(6, 6, 6, 6)
        self.show_btn = QPushButton("Mostrar")
        self.export_btn = QPushButton("Exportar")
        self.export_btn.setIcon(QIcon("./public/export2.svg"))

        self.export_btn.clicked.connect(self.export_log)

        self.export_btn.setStyleSheet(secondary_button_style)

        layout.addWidget(self.text, 7)
        # layout.addWidget(self.show_btn, 1)
        layout.addWidget(self.export_btn, 1)
        self.setLayout(layout)

    def export_log(self, x):
        options = QFileDialog.Options()
        fileName, _ = QFileDialog.getSaveFileName(
            self,
            "Seleccione la ruta para guardar el archivo de log",
            "",  # Default directory
            "All Files (*);;Text Files (*.txt);;Python Files (*.py)",  # Filter
            options=options,
        )
        print(fileName)
        self.text.setText(
            f"Para ver el historial de logs revise <span style='color: royalblue; text-decoration: underline'>{fileName}</span>"
        )
        self.user_config.update_value("log_history_filepath", fileName)
        self.log_file_updated.emit(fileName)


class RobotActionsLoggerView(QGroupBox):
    """
    A reusable widget for displaying application log messages.
    It automatically adds timestamps and provides options to clear or save the log.
    """

    log_file_updated = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._queue_size = 9
        self.log_queue = []
        self.log_history = []
        self.battery_state = 100
        msgs: list[str] = ["" for i in range(self._queue_size)]
        self.labels: QLabel = [QLabel(msg) for msg in msgs]

        self.up_btn = QPushButton("u")
        self.down_btn = QPushButton("d")

        layout = QVBoxLayout()
        buttons_layout = QVBoxLayout()
        main_layout = QGridLayout()
        layout.setSpacing(2)

        self.setStyleSheet("""
            QGroupBox {
                background-color:  white;
                border-radius: 2px;
                border: 1px solid gray;
                padding:4px;
            }
        """)

        for l in self.labels:
            layout.addWidget(l)
            l.setStyleSheet("font-family: 'Courier New'; font-size: 14px;")
        self.labels[0].setText(
            "<span style='color: green; font-weight: bold'>CHALA ES SUPER LINDA! SOY TU FAN! BIENVENIDO!</span>"
        )

        lb = FixedMessage()
        layout.addWidget(lb)
        lb.log_file_updated.connect(self.update_log_file)

        buttons_layout.addWidget(self.up_btn)
        buttons_layout.addWidget(self.down_btn)

        # layout.addWidget(self.log_display)
        main_layout.addLayout(layout, 0, 0)
        # main_layout.addLayout(buttons_layout, 0, 1)

        self.setLayout(main_layout)

    def update_log(self, log_msg: str) -> None:
        if len(self.log_queue) == self._queue_size:
            self.log_queue.pop(0)

        self.log_queue = [*self.log_queue, log_msg]
        self.log_history = [*self.log_history, log_msg]

        # self.log_display.clear()
        for i, text in enumerate(self.log_queue):
            print(text)
            self.labels[i].setText(text)

    def update_log_file(self, filepath):
        self.log_file_updated.emit(filepath)


class Logger(QObject):
    log_changed: pyqtSignal = pyqtSignal(str)

    def __init__(self, parent: QObject = None) -> None:
        super().__init__(parent)
        self.user_config = UserConfigFileManager("./config/app_config.json")
        config = self.user_config.read_data()
        self._logger = CsvHandler(
            header=["time", "level", "battery", "msg"],
            filepath=config["log_history_filepath"],
        )
        self.battery_state = 100

        self.battery_state_sub = rospy.Subscriber(
            "/battery_state", BatteryState, self.update_battery
        )

    def update_battery(self, msg):
        self.battery_state = msg.percentage * 100

    def log(self, msg: str = "Chala es suepr linda! Bienvenido!", level=None) -> None:
        now = datetime.now()
        timestamp = now.strftime("%A, %B %d, %Y - %I:%M %p")
        self.log_changed.emit(
            f"<span style='color: gray;'>{timestamp}</span> [INFO] [battery: {self.battery_state:.1f}%] {msg}"
        )
        self._logger.append_row(
            [timestamp, "INFO", f"battery: {self.battery_state}%", msg]
        )

    def update_log_file(self, filepath):
        config = self.user_config.read_data()
        self._logger = CsvHandler(
            header=["time", "level", "battery", "msg"],
            filepath=config["log_history_filepath"],
        )


logger = Logger()
