import sys
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QMessageBox,
    QComboBox,
    QGroupBox,
)


from  database_manager import DataBase

class GroupWrapper(QGroupBox):
    def __init__(self, text, children=[]) -> None:
        super().__init__(text)
        self.layout = QVBoxLayout()
        for child in children:
            self.layout.addWidget(child)

        self.setLayout(self.layout)


class UserForm(QGroupBox):
    def __init__(self):
        super().__init__("Formulario")
        layout = QVBoxLayout()
        self.setMaximumWidth(700)

        # Create UI elements
        self.title_label = QLabel("Informacion del lugar de inspeccion")
        self.title_label.setStyleSheet("font-size: 16px; font-weight: bold;")
        self.title_label.setMaximumHeight(80)

        self.name_label = QLabel("Nombre")
        self.name_input = QLineEdit()

        self.email_label = QLabel("Institucion")
        self.email_input = QLineEdit()

        self.password_label = QLabel("Direccion")
        self.password_input = QLineEdit()
        # self.password_input.setEchoMode(QLineEdit.Password)

        self.phone_label = QLabel("Contacto (Numeor telefonico)")
        self.phone_input = QLineEdit()

        # Add dropdown menu for user type
        self.type_label = QLabel("Seleccione un lugar deos lugares ddionibles o  cree un no nuvo desdell el menu de crear un lugar, los mapas que se creen van a estar asociados  a esta lugar")
        self.type_label.setWordWrap(True)
        self.type_combo = QComboBox()
        self.type_combo.addItems(
            ["Select type", "University", "Job", "Student", "Teacher", "Employee"]
        )

        self.create_button = QPushButton("Crear Lugar")
        self.create_button.clicked.connect(self.create_place)

        # Add widgets to layout
        layout.addWidget(self.title_label)

        select_place_widgets = (self.type_label, self.type_combo, )

        widgets = (
            self.name_label,
            self.name_input,
            self.email_label,
            self.email_input,
            self.password_label,
            self.password_input,
            self.phone_label,
            self.phone_input,
        )

        select_place_container = GroupWrapper(
            text="Seleccione un lugar para la inspeccion", 
            children=select_place_widgets
        )
        crete_place_container = GroupWrapper(text="Crear nuevo lugar", children=widgets)


        layout.addWidget(select_place_container)
        layout.addWidget(crete_place_container)
        layout.addWidget(self.create_button)
        layout.setSpacing(30)

        self.setLayout(layout)

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
    window = UserForm()
    window.show()
    sys.exit(app.exec_())
