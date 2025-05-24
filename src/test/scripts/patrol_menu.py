from PyQt5.QtWidgets import (
    QMainWindow,
    QApplication,
    QPushButton,
    QWidget,
    QTabWidget,
    QVBoxLayout,
    QLayout,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QSlider,
    QCheckBox,
    QComboBox,
    QGroupBox,
    QScrollArea,
    QStackedLayout,
    QMenu,
    QTimeEdit,
    QStyle,
)
from PyQt5.QtCore import Qt, pyqtSlot, pyqtSignal, QTime

from styles.buttons import primary_button_style, secondary_button_style 
from styles.labels import inactive_label_style, border_label_style, error_label_style

class PatrolsMenu(QMainWindow):
    update_date = pyqtSignal(dict)

    def __init__(self, parent, selected_days, patrolid, time='0000', state=None) -> None:
        super().__init__()
        # self.update_date = pyqtSignal(dict)
        # print(parent)
        # self.setFixedSize(300, 150)
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.setStyleSheet("""
            QMainWindow {
                border: 2px solid lightgray;
                border-radius: 5px;
                background-color: white;
            }
        """)

        self.setGeometry(0, 0, 380, 275)
        self.patrolid = patrolid
        self.state = state
        self.parent = parent
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.Popup)
        self.days_labels = DaySelect(selected_days=selected_days)
        self.time = TimeSelect()
        self.time.setTime(time) 

        buttons_layout = QHBoxLayout()
        close_btn = QPushButton(" Cerrar")
        save_btn = QPushButton(" Guardar")
        buttons_layout.addWidget(close_btn, 1)
        buttons_layout.addWidget(save_btn, 2)
        save_btn.setStyleSheet(primary_button_style)
        close_btn.setStyleSheet(secondary_button_style)

        icon_save = QApplication.style().standardIcon(QStyle.SP_DialogSaveButton)
        save_btn.setIcon(icon_save)

        self.empty_days_alert = QLabel('✘ ERROR: Seleccione almenos un dia')
        self.empty_days_alert.setStyleSheet(error_label_style)
        self.empty_days_alert.hide()
        layout = QVBoxLayout()
        layout.addWidget(self.time, alignment=Qt.AlignTop)
        layout.addWidget(self.days_labels, alignment=Qt.AlignTop)
        layout.addWidget(self.empty_days_alert, alignment=Qt.AlignTop)

        layout.addLayout(buttons_layout)
        save_btn.clicked.connect(self.save)
        close_btn.clicked.connect(self.close)
        self.central_widget.setLayout(layout)

    def show_popup(self):
        # popup_x = self.parent.x() + (self.parent.width() - self.width()) // 2
        # popup_y = self.parent.y() + (self.parent.height() - self.height()) // 2
        popup_x = self.parent.x() + (self.parent.width() - self.width() - 45) 
        popup_y = self.parent.y() + (self.parent.height() - self.height())//2 
        # print("popup", popup_x, popup_y)
        self.move(popup_x, popup_y)
        self.show()

    def close_popup(self):
        self.hide()

    def save(self):
        time = self.time.getTime()
        days = self.days_labels.get_selected_days()
        # print('patrol menu', days)
        x = {
                f"{name}": {"day": f"{name}", "time": time, "finished": False, 'patrolid': self.patrolid}
            for name in days
            if name
        }
        if len(x) == 0:
            self.empty_days_alert.show()
            return 

        a = {'days': x, 'time': time, "state": self.state}
        print("A MA, FUNCIONA patrool menu", x)
        self.update_date.emit({self.patrolid: a})
        self.hide()


class TimeSelect(QGroupBox):
    def __init__(self) -> None:
        super().__init__("Seleccione la hora de las patrullas")
        self.hours = QTimeEdit(self)
        self.minutes = QTimeEdit(self)
        # self.hours.setStyleSheet(f"font-size: {fontsize}px;")
        # self.minutes.setStyleSheet(f"font-size: {fontsize}px;")
        self.hours.setDisplayFormat("HH")
        self.minutes.setDisplayFormat("mm")

        self.layout = QHBoxLayout()
        self.layout.addWidget(self.hours)
        self.layout.addWidget(self.minutes)
        self.setLayout(self.layout)
        style =  ("""
            QTimeEdit {
                border: 2px solid #3498db;
                padding: 2px 10px;
                background: white;
                font-size: 25px;
            }
            
            QTimeEdit:hover {
                border-color: #2980b9;
            }
            
            QTimeEdit::up-button, QTimeEdit::down-button {
                background-color: #3498db;
                border: none;
                width: 40px;
            }
            
            QTimeEdit::up-button:hover, QTimeEdit::down-button:hover {
                background-color: blue;
            } 
        """)
        self.hours.setStyleSheet(style)
        self.minutes.setStyleSheet(style)
        # self.minutes.setTime()
        print('current time', QTime.currentTime())

    def setTime(self, time: str):
        x = [digit for digit in time]
        hours_value = ''.join(x[:2])
        minutes_value = ''.join(x[2:])
        self.hours.setTime(QTime.fromString(hours_value, 'HH'))
        self.minutes.setTime(QTime.fromString(minutes_value, 'mm'))

    def getTime(self):
        hours = self.hours.time()
        minutes = self.minutes.time()
        time_str = f'{hours.toString("HH")}{minutes.toString("mm")}'
        return time_str


class DaySelect(QGroupBox):
    days_shortname = ["Lun", "Mar", "Mie", "Jue", "Vie", "Sab", "Dom"]

    def __init__(self, selected_days) -> None:
        super().__init__("Seleccione los dias para hacer el patrullaje")
        self.setMinimumHeight(80)
        self.setMaximumHeight(80)
        self.layout = QHBoxLayout()
        self.days_labels = [DayLabel(day) for day in self.days_shortname]

        for text, label in zip(self.days_shortname, self.days_labels):
            if text in selected_days:
                label.setStyleSheet(border_label_style)
                label.isSelected = False
                label.state = True
            else:
                label.setStyleSheet(inactive_label_style)
                label.isSelected = True
                label.state = False

            self.layout.addWidget(label)
        self.setLayout(self.layout)

    def get_selected_days(self):
        return [label.value() for label in self.days_labels]


class DayLabel(QLabel):
    def __init__(self, text):
        super().__init__(text)
        self.isSelected = True
        self.state = True
        self.value1 = None
        self.text = text

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            if self.isSelected:
                self.setStyleSheet(border_label_style)
                # self.setStyleSheet("background-color: red; font-size: 20px;")
            else:
                # self.value1 = None
                self.setStyleSheet(inactive_label_style)

            self.isSelected = not self.isSelected
            self.state = not self.state
            super().mousePressEvent(event)

    def value(self):
        if self.state:
            return self.text
        else:
            return None
