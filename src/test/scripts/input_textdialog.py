import sys
from PyQt5.QtWidgets import (
    QApplication,
    QDialog,
    QVBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QMessageBox,
    QHBoxLayout,
)


from styles.buttons import (
    border_button_style,
    border_button_style_danger,
    primary_button_style,
    secondary_button_style,
    colored_button_style,
    tertiary_button_style,
    toggle_button_style,
    minimal_button_style,
    patrol_checkbox_style,
)

from styles.labels import (
    inactive_label_style,
    minimal_label_style,
    subtitle_label_style,
    error_label_style,
    normal_label_style
)


class InputDialog(QDialog):
    def __init__(self, parent):
        super().__init__(parent)
        self.setWindowTitle("Text Input Dialog")
        self.setGeometry(100, 100, 300, 150)
        self.filename = "defaultname"
        self.atempts = 0
        # self.setStyleSheet("""
        #     QMessageBox {
        #         background-color: #f8f9fa;
        #         font-size: 16px;
        #     }
        # """)

        self.initUI()

    def initUI(self):
        layout = QVBoxLayout()

        # Label
        self.label = QLabel("Ponle un nombre a tu feo mama!")
        self.label.setStyleSheet(subtitle_label_style)
        self.alert_label = QLabel("ERROR: Ingresa un nombre a tu feo mapa!")
        self.alert_label.setStyleSheet(error_label_style)
        self.alert_label.hide()
        layout.addWidget(self.label)
        layout.addWidget(self.alert_label)

        # Text input field
        self.text_input = QLineEdit()
        layout.addWidget(self.text_input)

        # Submit button
        self.submit_btn = QPushButton("Continuar")
        self.submit_btn.clicked.connect(self.on_submit)
        self.submit_btn.setStyleSheet(primary_button_style)
        layout.addWidget(self.submit_btn)

        self.setLayout(layout)

    def on_submit(self):
        input_text = self.text_input.text()
        if input_text:
            self.filename = input_text
            self.accept()
            return

        self.alert_label.show()

        if self.atempts > 1:
            self.accept()  # Close the dialog
            return
        self.atempts = self.atempts + 1


class CustomDialog(QDialog):
    def __init__(
        self,
        parent,
        title: str,
        message: str = '',
        positive_response: str = "Yes",
        negative_response: str = "No",
        retries: int = 0,
        interative: bool = True
    ):
        super().__init__(parent)
        self.setWindowTitle("Alerta!")
        self.setGeometry(100, 100, 400, 100)
        self.response = "Negative"
        self.title = title
        self.message = message
        self.positive_response = positive_response
        self.negative_response = negative_response
        self.retries = retries
        self.interative = interative
        self.atempts = 0 

        self.initUI()

    def initUI(self):
        layout = QVBoxLayout()
        buttons_layout = QHBoxLayout()

        # Label
        self.title_label = QLabel(self.title)
        self.alert_label = QLabel("ERROR: muy mallll!")
        self.message_label = QLabel(self.message)
        self.alert_label.hide()

        layout.addWidget(self.title_label)
        if self.message:
            layout.addWidget(self.message_label)
        layout.addWidget(self.alert_label)

        # Submit button
        self.positive_btn = QPushButton(self.positive_response)
        self.negative_btn = QPushButton(self.negative_response)
        self.default_close_btn = QPushButton('Cerrar')

        self.positive_btn.clicked.connect(self.setPositiveResponse)
        self.negative_btn.clicked.connect(self.setNegativeResponse)
        self.default_close_btn.clicked.connect(self.close)

        self.positive_btn.setStyleSheet(primary_button_style)
        self.default_close_btn.setStyleSheet(primary_button_style)
        self.negative_btn.setStyleSheet(tertiary_button_style)
        self.title_label.setStyleSheet(subtitle_label_style)
        self.message_label.setStyleSheet(normal_label_style )
        self.alert_label.setStyleSheet(error_label_style)

        if self.interative:
            buttons_layout.addWidget(self.negative_btn, 1)
            buttons_layout.addWidget(self.positive_btn, 2)
        else:
            buttons_layout.addWidget(self.default_close_btn)

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
