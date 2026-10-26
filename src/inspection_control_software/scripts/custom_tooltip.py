"""Custom speech-bubble tooltips, including an interactive variant with action buttons."""
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


class BaseTooltipWidget(QWidget):
    """Frameless speech-bubble shape (rounded rect + top arrow); subclasses
    add their own content on top of :meth:`paintEvent`'s bubble."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowFlags(Qt.ToolTip | Qt.FramelessWindowHint)
        self.setAttribute(Qt.WA_TranslucentBackground)

        # Style settings
        self.padding = 8
        self.arrow_height = 8
        self.arrow_width = 14
        self.border_radius = 8
        self.bg_color = QColor(25, 25, 25, 240)  # Premium dark grey
        self.border_color = QColor(60, 60, 60, 255)
        self.text_color = QColor("white")

    def paintEvent(self, event):
        """Draw the rounded-rect + arrow speech-bubble background."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Draw path
        path = QPainterPath()

        # Main rectangle (shifted down by arrow_height)
        rect = QRectF(
            0, self.arrow_height, self.width(), self.height() - self.arrow_height
        )
        path.addRoundedRect(rect, self.border_radius, self.border_radius)

        # Arrow (triangle) on top
        arrow_x_center = self.width() / 2
        path.moveTo(arrow_x_center - self.arrow_width / 2, self.arrow_height)
        path.lineTo(arrow_x_center, 0)
        path.lineTo(arrow_x_center + self.arrow_width / 2, self.arrow_height)

        # Fill and border
        painter.fillPath(path, QBrush(self.bg_color))
        pen = QPen(self.border_color)
        pen.setWidth(1)
        painter.setPen(pen)
        painter.drawPath(path)


class TooltipWidget(BaseTooltipWidget):
    """Plain text speech-bubble, auto-sized to fit its content."""

    def __init__(self, text, parent=None):
        super().__init__(parent)
        self.text = text
        self.font = QFont("Segoe UI", 9)
        self.update_size()

    def update_size(self):
        """Resize the bubble to fit the current text."""
        fm = QFontMetrics(self.font)
        text_rect = fm.boundingRect(self.text)
        self.text_width = text_rect.width()
        self.text_height = text_rect.height()

        total_width = self.text_width + 2 * self.padding
        total_height = self.text_height + 2 * self.padding + self.arrow_height
        self.setFixedSize(int(total_width), int(total_height))

    def paintEvent(self, event):
        """Draw the bubble background then the centered text."""
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setPen(self.text_color)
        painter.setFont(self.font)
        text_rect = QRectF(
            0, self.arrow_height, self.width(), self.height() - self.arrow_height
        )
        painter.drawText(text_rect, Qt.AlignCenter, self.text)


from PyQt5.QtWidgets import QHBoxLayout, QPushButton, QVBoxLayout, QLabel


class ActionTooltipWidget(BaseTooltipWidget):
    """Speech-bubble with a text label plus up to two action buttons,
    used by :class:`ActionToolTip` for interactive onboarding hints."""

    def __init__(self, text, btn1_text=None, btn2_text=None, parent=None):
        super().__init__(parent)
        self.text = text
        self.btn1_text = btn1_text
        self.btn2_text = btn2_text

        # Layout
        self.main_layout = QVBoxLayout(self)
        self.main_layout.setContentsMargins(
            self.padding, self.arrow_height + self.padding, self.padding, self.padding
        )
        self.main_layout.setSpacing(8)

        # Text Label
        self.label = QLabel(self.text)
        self.label.setStyleSheet(f"color: white; font-family: 'Segoe UI'; font-size: 9pt;")
        self.label.setAlignment(Qt.AlignCenter)
        self.main_layout.addWidget(self.label)

        # Buttons Layout
        if btn1_text or btn2_text:
            self.btn_layout = QVBoxLayout()
            self.btn_layout.setSpacing(6)

            self.btn1 = None
            self.btn2 = None

            button_style = """
                QPushButton {
                    background-color: #3d3d3d;
                    color: white;
                    border: 1px solid #555555;
                    border-radius: 4px;
                    padding: 4px 12px;
                    font-size: 9pt;
                }
                QPushButton:hover {
                    background-color: #505050;
                    border: 1px solid #666666;
                }
                QPushButton:pressed {
                    background-color: #2b2b2b;
                }
            """

            if btn1_text:
                self.btn1 = QPushButton(btn1_text)
                self.btn1.setStyleSheet(button_style)
                self.btn1.setCursor(Qt.PointingHandCursor)
                self.btn_layout.addWidget(self.btn1)

            if btn2_text:
                self.btn2 = QPushButton(btn2_text)
                self.btn2.setStyleSheet(button_style)
                self.btn2.setCursor(Qt.PointingHandCursor)
                self.btn_layout.addWidget(self.btn2)

            self.main_layout.addLayout(self.btn_layout)

        self.adjustSize()
        # Ensure it's not too small or too large
        min_width = 150 if (btn1_text or btn2_text) else 80
        self.setFixedWidth(max(self.width(), min_width))
        self.setFixedHeight(self.height())

    def enterEvent(self, event):
        """Hook for the controller to detect hover-in (no extra behavior here)."""
        # We'll use this in the controller
        super().enterEvent(event)

    def leaveEvent(self, event):
        """Hook for the controller to detect hover-out (no extra behavior here)."""
        super().leaveEvent(event)


class CustomToolTip(QObject):
    """Hover-triggered tooltip controller: shows a :class:`TooltipWidget`
    below the installed widget after ``delay`` ms of hovering, with a grace
    period before hiding so the mouse can move onto the tooltip itself."""

    def __init__(self, parent=None, delay=300):
        super().__init__(parent)
        self.delay = delay
        self.timer = QTimer(self)
        self.timer.setSingleShot(True)
        self.timer.timeout.connect(self.show_tooltip)
        self._active_hover = True

        self.hide_timer = QTimer(self)
        self.hide_timer.setSingleShot(True)
        self.hide_timer.timeout.connect(self.hide_tooltip)

        self.widget = None
        self.text = ""
        self.tooltip_widget = None
        self.destroyed.connect(self.handle_destruction)

    def install(self, widget: QWidget, text: str):
        """Attach this tooltip to ``widget``, to be shown on hover with ``text``."""
        self.widget = widget
        self.text = text
        widget.installEventFilter(self)

    def eventFilter(self, obj, event):
        """Drive the show/hide timers from enter/leave/click events on the
        target widget and the tooltip itself."""
        if not self._active_hover:
            return super().eventFilter(obj, event)

        if obj == self.widget:
            if event.type() == QEvent.Enter:
                self.hide_timer.stop()
                self.timer.start(self.delay)
            elif event.type() == QEvent.Leave:
                self.timer.stop()
                # Start hide timer to allow mouse to move to the tooltip
                self.hide_timer.start(200)
            elif event.type() == QEvent.MouseButtonPress:
                self.timer.stop()
                self.hide_tooltip()
        elif self.tooltip_widget and obj == self.tooltip_widget:
            if event.type() == QEvent.Enter:
                self.hide_timer.stop()
            elif event.type() == QEvent.Leave:
                self.hide_timer.start(200)

        return super().eventFilter(obj, event)

    def handle_destruction(self):
         """Clean up the installed event filter when this object is destroyed."""
         self.widget.removeEventFilter(self)

    def show_tooltip(self):
        """Create, position and show the tooltip widget."""
        if True or self.widget and self.widget.underMouse():
            self._create_tooltip()
            self._position_tooltip()
            self.tooltip_widget.show()

    def _create_tooltip(self):
        """(Re)build the plain :class:`TooltipWidget` for the current text."""
        if self.tooltip_widget:
            self.tooltip_widget.close()
        self.tooltip_widget = TooltipWidget(self.text)
        self.tooltip_widget.installEventFilter(self)

    def _position_tooltip(self):
        """Center the tooltip horizontally below the target widget."""
        # Calculate position
        widget_pos = self.widget.mapToGlobal(QPoint(0, 0))
        widget_rect = self.widget.rect()

        # Center horizontally relative to target widget
        x = widget_pos.x() + (widget_rect.width() - self.tooltip_widget.width()) / 2
        # Position below the target widget
        y = widget_pos.y() + widget_rect.height()

        self.tooltip_widget.move(int(x), int(y))

    def hide_tooltip(self):
        """Hide the tooltip, unless the mouse has moved onto it."""
        if self.tooltip_widget and not self.tooltip_widget.underMouse():
            self.tooltip_widget.hide()


class ActionToolTip(CustomToolTip):
    """Explicitly-triggered tooltip (``show_tooltip`` called directly, not
    on hover) that renders an :class:`ActionTooltipWidget` with up to two
    action buttons, temporarily highlighting the target widget's border."""

    def __init__(self, parent=None, delay=600):
        super().__init__(parent, delay)
        self.btn1_text = None
        self.btn1_callback = None
        self.btn2_text = None
        self.btn2_callback = None
        self._active_hover = False
        self.widget = None

    def install(
        self,
        widget: QWidget,
        text: str,
        btn1_text: str = None,
        btn1_callback=None,
        btn2_text: str = None,
        btn2_callback=None,
    ):
        """Install on ``widget`` with up to two labeled action buttons/callbacks."""
        self.btn1_text = btn1_text
        self.btn1_callback = btn1_callback
        self.btn2_text = btn2_text
        self.btn2_callback = btn2_callback
        self.widget = widget
        self.widget.setObjectName("specific")
        self._current = self.widget.styleSheet()
        super().install(widget, text)

    def _create_tooltip(self):
        """Highlight the target widget's border and build the action tooltip, wiring button callbacks."""
        if self.tooltip_widget:
            self.tooltip_widget.close()

        extra = "\n#specific { border:2px solid  blue; border-radius: 4px;}"
        self.widget.setStyleSheet(self._current + extra)
        print("Widget style sheet:", self.widget.styleSheet())
        print("Widget:", self.widget)

        self.tooltip_widget = ActionTooltipWidget(
            self.text, self.btn1_text, self.btn2_text
        )
        self.tooltip_widget.installEventFilter(self)

        if self.btn1_text and self.btn1_callback:
            self.tooltip_widget.btn1.clicked.connect(self.btn1_callback)
            self.tooltip_widget.btn1.clicked.connect(self.hide_tooltip_forced)

        if self.btn2_text and self.btn2_callback:
            self.tooltip_widget.btn2.clicked.connect(self.btn2_callback)
            self.tooltip_widget.btn2.clicked.connect(self.hide_tooltip_forced)

    def hide_tooltip_forced(self):
        """Hide the tooltip and restore the target widget's original style."""
        if self.tooltip_widget:
            self.tooltip_widget.hide()
            self.widget.setStyleSheet(self._current)
