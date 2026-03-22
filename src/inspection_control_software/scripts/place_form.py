import sys

from database_manager import DataBase
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


class GroupWrapper(QGroupBox):
    def __init__(self, text, children=[]) -> None:
        super().__init__(text)
        self.layout = QVBoxLayout()
        for child in children:
            self.layout.addWidget(child, alignment=Qt.AlignTop)
        # self.setMaximumHeight(100)

        self.setLayout(self.layout)


class PlaceForm(QWidget):
    def __init__(self):
        super().__init__()
        self.directory_path: str = ""
        form_layout = QFormLayout()
        self.setFixedWidth(550)

        browse_btn = QPushButton("Buscar...")
        browse_btn.clicked.connect(self.browse_file)

        urdf_layout = QHBoxLayout()
        self.urdf_input = QLineEdit()
        self.urdf_input.setPlaceholderText("/path/to/model.urdf")

        urdf_layout.addWidget(self.urdf_input)
        urdf_layout.addWidget(browse_btn)
        form_layout.addRow("URDF File Path:", urdf_layout)

        a = QComboBox()
        a.addItems(["opcion 1", "opcion 2"])
        form_layout.addRow("Seleccione un lugar", a)

        self.odom_input = QLineEdit()
        self.odom_input.setPlaceholderText("e.g., /odom")
        form_layout.addRow("Odom Topic Name:", self.odom_input)

        self.odom_input1 = QLineEdit()
        self.odom_input1.setPlaceholderText("e.g., /odom")
        form_layout.addRow("Odom Topic Name:", self.odom_input1)

        layout = QVBoxLayout()
        layout.addLayout(form_layout)

        self.setLayout(layout)

    def handleSubmit(self, filename):
        pass

    def browse_file(self):
        """Opens a file dialog to select the URDF file."""
        options = QFileDialog.Options()
        # 1. Tell it to only show directories
        options |= QFileDialog.ShowDirsOnly
        # 2. Force Qt to use its own dialog instead of the Windows/Mac one
        # (This ensures files are actually hidden, not just grayed out)
        # options |= QFileDialog.DontUseNativeDialog
        # We use getExistingDirectory instead of getOpenFileName
        self.directory_path = QFileDialog.getExistingDirectory(
            self,
            "Seleccione un directorio válido",  # Title of the window
            "",  # Starting directory (empty = current)
            options=options,
        )

        # Note: getExistingDirectory returns a string (the path), not a tuple.
        if self.directory_path:
            self.urdf_input.setText(self.directory_path)

    def create_place(self):
        name = self.name_input.text()
        email = self.email_input.text()
        password = self.password_input.text()
        user_type = self.type_combo.currentText()

        if not name or not email or not password:
            QMessageBox.warning(self, "Error", "All fields are required!")
            return

        if user_type == "Select type":
            QMessageBox.warning(self, "Error", "Please select a user type!")
            return

        QMessageBox.information(
            self,
            "Success",
            f"User created successfully!\n\nName: {name}\nEmail: {email}\nType: {user_type}",
        )
        self.clear_form()

    def clear_form(self):
        self.name_input.clear()
        self.email_input.clear()
        self.password_input.clear()
        self.type_combo.setCurrentIndex(0)  # Reset to "Select type"


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = PlaceForm()
    window.show()
    sys.exit(app.exec_())
