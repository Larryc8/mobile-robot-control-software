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
)
from PyQt5.QtCore import Qt, pyqtSlot, pyqtSignal


class PatrolsMenu(QWidget):
    update_date = pyqtSignal(dict)

    def __init__(self, parent, selected_days, patrolid) -> None:
        super().__init__(parent)
        # self.update_date = pyqtSignal(dict)
        # print(parent)
        # self.setFixedSize(300, 150)
        self.patrolid = patrolid
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.Popup)
        self.days_labels = DaySelect(selected_days=selected_days)
        self.time = TimeSelect(menu=None)
        close_btn = QPushButton("Close")
        save_btn = QPushButton("Save")
        layout = QVBoxLayout(self.time)
        layout.addWidget(self.time)
        layout.addWidget(self.days_labels)
        layout.addWidget(close_btn)
        layout.addWidget(save_btn)
        save_btn.clicked.connect(self.save)
        self.setLayout(layout)

    def save(self):
        time = self.time.getTime()
        days = self.days_labels.get_selected_days()
        x = {
            f"{name}": {"day": f"{name}", "time": time, "finished": False}
            for name in days
            if name
        }
        a = {'days': x, 'time': time}
        # x['time'] = time
        print("A MA, FUNCIONA", x)
        # self.update_date.emit('ww')
        self.update_date.emit({self.patrolid: a})
        self.hide()


class TimeSelect(QGroupBox):
    def __init__(self, menu) -> None:
        super().__init__("Selct the time for patrol")
        self.hours = QTimeEdit()
        self.minutes = QTimeEdit()
        self.hours.setStyleSheet("font-size: 30px;")
        self.minutes.setStyleSheet("font-size: 30px;")
        self.hours.setDisplayFormat("HH")
        self.minutes.setDisplayFormat("mm")

        self.layout = QHBoxLayout()
        self.layout.addWidget(self.hours)
        self.layout.addWidget(self.minutes)
        self.setLayout(self.layout)

    def setTime(self):
        pass

    def getTime(self):
        alarm_time = self.hours.time()
        alarm_str = alarm_time.toString("HH")
        return alarm_str


class DaySelect(QGroupBox):
    days_shortname = ["Lun", "Mar", "Mie", "Jue", "Vie", "Sab", "Dom"]

    def __init__(self, selected_days) -> None:
        super().__init__("Select the days for patrol")
        self.layout = QHBoxLayout()
        self.days_labels = [DayLabel(day) for day in self.days_shortname]

        for text, label in zip(self.days_shortname, self.days_labels):
            if text in selected_days:
                label.setStyleSheet("background-color: red; font-size: 20px;")
                label.isSelected = False
                label.state = True
            else:
                label.setStyleSheet("background-color: gray; font-size: 20px;")
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
                self.setStyleSheet("background-color: red; font-size: 20px;")
            else:
                # self.value1 = None
                self.setStyleSheet("background-color: gray; font-size: 20px;")

            self.isSelected = not self.isSelected
            self.state = not self.state
            super().mousePressEvent(event)

    def value(self):
        if self.state:
            return self.text
        else:
            return None
