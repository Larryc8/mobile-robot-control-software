import sys

from PyQt5.QtCore import (
    QSettings,
    Qt,  # Add this import at the top
)
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication,
    QDialog,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QPushButton,
    QVBoxLayout,
)
from styles.buttons import primary_button_style, secondary_button_style

# ... inside init_ui ...
# self.list_widget = QListWidget()
# This ensures text is cut off with "..." in the middle or end


class CustomFileDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Abrir archivos - mapas usados reciente")
        self._MAX_FILES = 3
        self.resize(800, 300)

        # Initialize Settings (to remember files across sessions)
        self.settings = QSettings("MyCompany", "MyApp")
        self.recent_files = self.settings.value("recentFiles", [])

        self.selected_file = None
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()

        # 1. Recent Files Section
        layout.addWidget(QLabel("<b>Archivos recientes:</b>"))
        self.list_widget = QListWidget()
        self.list_widget.setTextElideMode(Qt.ElideMiddle)
        # Optional: Ensure the list doesn't create a horizontal scrollbar
        self.list_widget.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        font = QFont()
        font.setPointSize(11)  # Set font size to 12 points
        font.setBold(True)  # Optional: make it bold
        self.list_widget.setFont(font)
        self.list_widget.addItems(self.recent_files)
        self.list_widget.itemDoubleClicked.connect(self.open_recent)
        layout.addWidget(self.list_widget)

        # 2. Buttons
        btn_layout = QHBoxLayout()

        self.btn_browse = QPushButton("Mostrar todos...")
        self.btn_browse.setStyleSheet(primary_button_style)
        self.btn_browse.clicked.connect(self.browse_files)

        self.btn_cancel = QPushButton("Cancelar")
        self.btn_cancel.setStyleSheet(secondary_button_style)
        self.btn_cancel.clicked.connect(self.reject)

        btn_layout.addWidget(self.btn_browse)
        btn_layout.addStretch()
        btn_layout.addWidget(self.btn_cancel)

        layout.addLayout(btn_layout)
        self.setLayout(layout)

    def browse_files(self):
        # Open the standard system dialog
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Abrir archivo de configuración de mapa",
            "",
            "Archivo de configuración (*.yaml)",
        )
        if file_path:
            self.save_and_close(file_path)

    def open_recent(self, item):
        self.save_and_close(item.text())

    def save_and_close(self, path):
        self.selected_file = path

        # Update Recent List logic
        if path in self.recent_files:
            self.recent_files.remove(path)
        self.recent_files.insert(0, path)

        # Keep only the last 10 items
        self.recent_files = self.recent_files[:10]

        # Save to disk
        self.settings.setValue("recentFiles", self.recent_files)
        self.accept()


# --- Usage Example ---
if __name__ == "__main__":
    app = QApplication(sys.argv)

    dialog = CustomFileDialog()
    if dialog.exec_():
        print(f"User selected: {dialog.selected_file}")

    sys.exit()
