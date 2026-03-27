from PyQt5.QtCore import QEvent, QObject, QPoint, QRectF, Qt, QTimer
from PyQt5.QtGui import (
    QBrush,
    QColor,
    QFont,
    QFontMetrics,
    QPainter,
    QPainterPath,
    QPen,
)
from PyQt5.QtWidgets import QGraphicsDropShadowEffect, QWidget


class TooltipWidget(QWidget):
    def __init__(self, text, parent=None):
        super().__init__(parent)
        self.text = text
        self.setWindowFlags(Qt.ToolTip | Qt.FramelessWindowHint)
        self.setAttribute(Qt.WA_TranslucentBackground)

        # Font settings
        self.font = QFont("Segoe UI", 9)
        # self.font.setBold(True)

        # Metrics
        self.padding = 7
        self.arrow_height = 8
        self.arrow_width = 14
        self.border_radius = 6

        # Calculate size
        self.update_size()

    def update_size(self):
        fm = QFontMetrics(self.font)
        text_rect = fm.boundingRect(self.text)
        self.text_width = text_rect.width()
        self.text_height = text_rect.height()

        total_width = self.text_width + 2 * self.padding
        total_height = self.text_height + 2 * self.padding + self.arrow_height
        self.setFixedSize(total_width, total_height)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Colors
        bg_color = QColor("black")
        text_color = QColor("white")
        border_color = QColor("gray")

        # Draw path
        path = QPainterPath()

        # Main rectangle (shifted down by arrow_height)
        rect = QRectF(
            0, self.arrow_height, self.width(), self.height() - self.arrow_height
        )
        path.addRoundedRect(rect, self.border_radius, self.border_radius)

        # Arrow (triangle) on top
        arrow_x_center = self.width() / 2

        # Define triangle points
        path.moveTo(
            arrow_x_center - self.arrow_width / 2, self.arrow_height
        )  # Left base
        path.lineTo(arrow_x_center, 0)  # Tip
        path.lineTo(
            arrow_x_center + self.arrow_width / 2, self.arrow_height
        )  # Right base

        # Fill path
        painter.fillPath(path, QBrush(bg_color))

        # Draw border
        pen = QPen(border_color)
        pen.setWidth(1)
        painter.setPen(pen)
        painter.drawPath(path)

        # Draw text
        painter.setPen(text_color)
        painter.setFont(self.font)
        text_rect = QRectF(
            0, self.arrow_height, self.width(), self.height() - self.arrow_height
        )
        painter.drawText(text_rect, Qt.AlignCenter, self.text)


class CustomToolTip(QObject):
    def __init__(self, parent=None, delay=1000):
        super().__init__(parent)
        self.delay = delay
        self.timer = QTimer(self)
        self.timer.setSingleShot(True)
        self.timer.timeout.connect(self.show_tooltip)
        self.widget = None
        self.text = ""
        self.tooltip_widget = None

    def install(self, widget: QWidget, text: str):
        self.widget = widget
        self.text = text
        widget.installEventFilter(self)

    def eventFilter(self, obj, event):
        if obj == self.widget:
            if event.type() == QEvent.Enter:
                self.timer.start(self.delay)
            elif event.type() == QEvent.Leave:
                self.timer.stop()
                self.hide_tooltip()
            elif event.type() == QEvent.MouseButtonPress:
                self.timer.stop()
                self.hide_tooltip()
        return super().eventFilter(obj, event)

    def show_tooltip(self):
        if self.widget and self.widget.underMouse():
            if self.tooltip_widget:
                self.tooltip_widget.close()

            self.tooltip_widget = TooltipWidget(self.text)

            # Calculate position
            widget_pos = self.widget.mapToGlobal(QPoint(0, 0))
            widget_rect = self.widget.rect()

            # Center horizontally relative to target widget
            x = widget_pos.x() + (widget_rect.width() - self.tooltip_widget.width()) / 2

            # Position below the target widget
            y = widget_pos.y() + widget_rect.height()

            self.tooltip_widget.move(int(x), int(y))
            self.tooltip_widget.show()

    def hide_tooltip(self):
        if self.tooltip_widget:
            self.tooltip_widget.hide()
