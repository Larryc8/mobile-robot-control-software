from enum import Enum
from PyQt5.QtCore import QSize
from PyQt5.QtGui import QFont, QIcon, QPixmap, QTransform, QFontMetrics
from pyqttoast import Toast, ToastPreset, ToastPosition


class NotificationType(Enum):
    LIDAR_ERROR = ('No hay dstos del LiDar', 'Reise la configuracion del robot', ToastPreset.ERROR)
#     MAP_LOAD_ERROR
#     MAP_CREATION_ERROR
#     MAP_LOAD_SUCCESS
#     MAP_CREATION_SUCCESS
#     PATROLS
#     CHECKPOINTS


class Notification(Toast):
    def __init__(self, parent, title: str='', msg: str= '', preset=None, duration=4000, type: NotificationType = None) -> None:
        font = 'Helvetica'
        super().__init__(parent)
        self.setDuration(duration)  # Hide after 5 seconds

        if type:
            (title, msg, preset ) = type.value

        self.setTitle(title)
        self.setText(msg)

        self.applyPreset(preset)  # Apply style preset
        self.setPosition(ToastPosition.TOP_MIDDLE)  # Default: ToastPosition.BOTTOM_RIGHT
        # self.setTitleFont(QFont(font, 10, QFont.Weight.Bold))  # Default: QFont('Arial', 9, QFont.Weight.Bold)
        # self.setTextFont(QFont(font, 10))   # Default: QFont('Arial', 9)
        self.setIconSize(QSize(25, 25))
        self.setFixedSize(QSize(500, 75))
        self.setPositionRelativeToWidget(parent)
        self.setOffset(30, 75)  # Default: 20, 45
        # setBackgroundColor(QColor('#292929'))       # Default: #E7F4F9
        # setTitleColor(QColor('#FFFFFF'))            # Default: #000000
        # setTextColor(QColor('#D0D0D0'))             # Default: #5C5C5C
        # setDurationBarColor(QColor('#3E9141'))      # Default: #5C5C5C
        # setIconColor(QColor('#3E9141'))             # Default: #5C5C5C
        # setIconSeparatorColor(QColor('#585858'))    # Default: #D9D9D9
        # setCloseButtonIconColor(QColor('#C9C9C9'))  # Default: #000000
                # toast.show()

