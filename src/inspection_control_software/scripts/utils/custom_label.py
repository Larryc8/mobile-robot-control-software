import sys
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFontMetrics, QPainter

# Qt.TextElideMode.ElideRight
# Qt.TextElideMode.ElideLeft
# Qt.TextElideMode.ElideMiddle

from custom_tooltip import CustomToolTip

class ElidedLabel(QLabel):
    def __init__(self, text="", parent=None):
        super().__init__(parent)
        self._full_text = text
        # self.setToolTip(text) # Show full text on hover
        tooltip = text
        self.tooltip = CustomToolTip(self, delay=100)
        self.tooltip.install(self, tooltip)


    def setText(self, text):
        self._full_text = text
        # self.setToolTip(text)
        super().setText(text)

    def paintEvent(self, event):
        painter = QPainter(self)
        metrics = QFontMetrics(self.font())

        # This is where the magic happens
        elided = metrics.elidedText(self._full_text, Qt.TextElideMode.ElideMiddle, self.width())

        # Draw the elided text instead of the full text
        painter.drawText(self.rect(), self.alignment(), elided)

# --- Example Usage ---
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = QWidget()
    layout = QVBoxLayout(window)

    label = ElidedLabel("This is a very long piece of text that should have an ellipsis if the window is too small.")
    label.setStyleSheet("border: 1px solid gray; font-size: 16px;")

    layout.addWidget(label)
    window.resize(300, 100)
    window.show()
    sys.exit(app.exec())
