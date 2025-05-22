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

from styles.labels import inactive_label_style, minimal_label_style, subtitle_label_style, error_label_style


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
        self.alert_label = QLabel('ERROR: Ingresa un nombre a tu feo mapa!')
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



if __name__ == "__main__":
    app = QApplication(sys.argv)
    dialog = InputDialog()
    dialog.exec_()
