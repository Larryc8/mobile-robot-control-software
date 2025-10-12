import typing
import sys
from datetime import datetime
from PyQt5.QtWidgets import (
    QApplication,
    QGroupBox,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QPlainTextEdit,
    QPushButton,
    QHBoxLayout,
    QLabel,
    QFileDialog
)
from PyQt5.QtCore import QObject, Qt, pyqtSignal

class RobotActionsLoggerView(QGroupBox):
    """
    A reusable widget for displaying application log messages.
    It automatically adds timestamps and provides options to clear or save the log.
    """
    def __init__(self, parent=None):
        super().__init__('Logger', parent)
        msgs: list[str] = ["<span style='color: red; font-weight: bold'>Riascos Manyoma the best lastnames of the world</span>" for i in range(4)]
        self.labels = [QLabel(msg) for msg in msgs]
        layout = QVBoxLayout()

        for l in self.labels:
            layout.addWidget(l)

        self.setLayout(layout)

    def update_log(self, log_msgs: list) -> None:
        for i, text in enumerate(log_msgs):
            self.labels[i].setText(text)

class Logger(QObject):
    log_changed: pyqtSignal = pyqtSignal(list)

    def __init__(self, parent: typing.Optional['QObject'] = None) -> None:
        super().__init__(parent)

    def log(self) -> None:
        self.log_changed.emit(['Harold el mejor del mundo' for i in range(4)])

logger = Logger()
