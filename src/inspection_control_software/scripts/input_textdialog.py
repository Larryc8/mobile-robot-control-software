import sys

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import (
    QApplication,
    QDialog,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
)
from styles.buttons import (
    border_button_style,
    border_button_style_danger,
    colored_button_style,
    minimal_button_style,
    patrol_checkbox_style,
    primary_button_style,
    secondary_button_style,
    tertiary_button_style,
    toggle_button_style,
)
from styles.labels import (
    error_label_style,
    inactive_label_style,
    minimal_label_style,
    normal_label_style,
    subtitle_label_style,
)


class InputDialog(QDialog):
    submitted = pyqtSignal(str)

    def __init__(self, parent, title, msg="Algo de texto", child=None):
        super().__init__(parent)
        self.setWindowTitle("Guardar archivo de mapeo")
        self.setGeometry(100, 100, 300, 150)
        self.filename = "defaultname"
        self.title = title
        self.atempts = 0
        self.child = child
        self.msg = msg
        # self.setStyleSheet("""
        #     QMessageBox {
        #         background-color: #f8f9fa;
        #         font-size: 16px;
        #     }
        # """)

        self.initUI()

    def initUI(self):
        layout = QVBoxLayout()
        buttons_layout = QHBoxLayout()

        # Label
        self.tittle_label = QLabel(self.title)
        self.tittle_label.setStyleSheet(subtitle_label_style)
        self.alert_label = QLabel("Ingresa un nombre valido")
        self.alert_label.setStyleSheet(error_label_style)
        self.alert_label.hide()
        layout.addWidget(self.tittle_label)
        layout.addWidget(self.alert_label)
        a = QLabel(self.msg)
        layout.addWidget(a)

        # Text input field
        self.text_input = QLineEdit()
        layout.addWidget(self.text_input)

        # Submit button
        self.discard_btn = QPushButton("Descartar")
        self.submit_btn = QPushButton("Continuar")
        self.discard_btn.clicked.connect(self.on_discard)
        self.submit_btn.clicked.connect(self.on_submit)

        self.discard_btn.setStyleSheet(secondary_button_style)
        self.submit_btn.setStyleSheet(primary_button_style)

        buttons_layout.addWidget(self.discard_btn)
        buttons_layout.addWidget(self.submit_btn)

        if self.child:
            self.submitted.connect(self.child.handleSubmit)
            layout.addWidget(self.child)

        layout.addLayout(buttons_layout)

        self.setLayout(layout)

    def on_submit(self):
        input_text = self.text_input.text()
        if self.child.directory_path:
            return

        if input_text :
            self.filename = self.check_filename(input_text)
            self.filename = f"{self.child.directory_path}/{self.filename}"
            self.accept()
            return

        self.alert_label.show()

        if self.atempts > 1:
            self.accept()  # Close the dialog
            return
        self.atempts = self.atempts + 1

    def check_filename(self, filename):
        """
        Cleans a filename by removing leading/trailing whitespace
        and replacing internal spaces with underscores.
        """
        if not filename:
            return None  # or raise an error if strict

        # 1. strip() removes spaces at the very start and end (e.g., "  file.txt  " -> "file.txt")
        # 2. replace(" ", "_") changes remaining spaces to underscores (e.g., "my file.txt" -> "my_file.txt")
        clean_name = filename.strip().replace(" ", "_")

        return clean_name

    def on_discard(self):
        self.filename = None
        self.accept()


class CustomDialog(QDialog):
    def __init__(
        self,
        parent,
        title: str,
        message: str = "Hola mucho gusto! Soy un Error",
        positive_response: str = "Yes",
        negative_response: str = "No",
        retries: int = 0,
        interative: bool = True,
        child=None,
    ):
        super().__init__(parent)
        self.setWindowTitle("Alerta!")
        self.setFixedSize(500, 150)
        self.response = "Negative"
        self.title = title
        self.message = message
        self.positive_response = positive_response
        self.negative_response = negative_response
        self.retries = retries
        self.interative = interative
        self.child = child
        self.atempts = 0

        self.initUI()

    def initUI(self):
        layout = QVBoxLayout()
        buttons_layout = QHBoxLayout()

        # Label
        self.title_label = QLabel(self.title)
        self.alert_label = QLabel("Esta seguro de esta acción?")
        self.message_label = QLabel(self.message)
        self.message_label.setWordWrap(True)
        self.alert_label.hide()

        layout.addWidget(self.title_label)
        layout.addWidget(self.message_label)
        layout.addWidget(self.alert_label)

        # Submit button
        self.positive_btn = QPushButton(self.positive_response)
        self.negative_btn = QPushButton(self.negative_response)
        self.default_close_btn = QPushButton("Cerrar")

        self.positive_btn.clicked.connect(self.setPositiveResponse)
        self.negative_btn.clicked.connect(self.setNegativeResponse)
        self.default_close_btn.clicked.connect(self.close)

        self.positive_btn.setStyleSheet(primary_button_style)
        self.default_close_btn.setStyleSheet(primary_button_style)
        self.negative_btn.setStyleSheet(secondary_button_style)
        self.title_label.setStyleSheet(subtitle_label_style)
        self.message_label.setStyleSheet(normal_label_style)
        self.alert_label.setStyleSheet(error_label_style)

        if self.interative:
            buttons_layout.addWidget(self.negative_btn, 1)
            buttons_layout.addWidget(self.positive_btn, 2)
        else:
            buttons_layout.addWidget(self.default_close_btn)

        if self.child:
            layout.addWidget(self.child)
        layout.addLayout(buttons_layout)

        self.setLayout(layout)

    def setPositiveResponse(self):
        self.response = "Positive"
        self.accept()

    def close(self):
        self.accept()

    def setNegativeResponse(self):
        self.response = "Negative"
        self.atempts = self.atempts + 1

        if self.atempts > self.retries:
            self.accept()  # Close the dialog
            return

        if self.atempts == self.retries:
            self.alert_label.show()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    dialog = InputDialog()
    dialog.exec_()
