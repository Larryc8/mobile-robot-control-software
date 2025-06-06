import sys
from PyQt5.QtWidgets import (
    QApplication,
    QDialog,
    QVBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QMessageBox,
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

from styles.labels import inactive_label_style, minimal_label_style


class InputDialog(QDialog):
    def __init__(self, parent):
        super().__init__(parent)
        self.setWindowTitle("Text Input Dialog")
        self.setGeometry(100, 100, 300, 150)
        self.filename = ""
        # self.setStyleSheet("""
        #     QMessageBox {
        #         background-color: #f8f9fa;
        #         font-size: 16px;
        #     }
        # """)
        self.setStyleSheet("""
            QDialog {
                background-color: #f5f5f5;
                border: 1px solid #ccc;
                border-radius: 8px;
            }
            QLabel {
                font-size: 14px;
                padding: 20px;
            }
          """)

        self.initUI()

    def initUI(self):
        layout = QVBoxLayout()

        # Label
        self.label = QLabel("Ponle un nombre a tu feo mama")
        layout.addWidget(self.label)

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
        self.filename = input_text
        self.accept()  # Close the dialog


class CustomDialog(QDialog):
    def __init__(self, parent, text):
        super().__init__(parent)
        self.setWindowTitle("Text Input Dialog")
        self.setGeometry(100, 100, 300, 150)
        self.filename = ""
        self.setStyleSheet("""
            QMessageBox {
                background-color: #f8f9fa;
                font-size: 16px;
            }
        """)

        self.initUI()

    def initUI(self):
        layout = QVBoxLayout()

        # Label
        self.label = QLabel("Ponle un nombre a tu feo mama")
        layout.addWidget(self.label)

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
        self.filename = input_text
        self.accept()  # Close the dialog


if __name__ == "__main__":
    app = QApplication(sys.argv)
    dialog = InputDialog()
    dialog.exec_()
