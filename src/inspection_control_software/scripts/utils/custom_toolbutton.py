from PyQt5.QtWidgets import QToolButton
from PyQt5.QtGui import QFont, QFontMetrics, QIcon, QPixmap, QTransform, QFont
from PyQt5.QtCore import QSize, Qt

from custom_tooltip import CustomToolTip


class CustomToolButtom(QToolButton):
    def __init__(self, text:str ="", icon :str ="./public/map-editing-svgrepo-com.svg", text2="hola**", icon2 :str ="./public/map-editing-svgrepo-com.svg", twist:bool=False, size=30, tooltip:str = ""):
       super().__init__()
       self.__state = True
       self.__is_selected = twist
       self.__style =  self.styleSheet()

       self.__icon = icon
       self.__icon2 = icon2
       self.__text = text
       self.__text2 = text2

       icon_start = QIcon(icon)
       icon_size = QSize(size, size) # Set width and height in pixels
       self.setIconSize(icon_size)
       self.setIcon(icon_start)


       if tooltip:
        self.tooltip = CustomToolTip(self, delay=100)
        self.tooltip.install(self, tooltip)

       if text:
        self.setText(text)
        font = self.font()
        font.setPointSize(8)    # Set size in points
        # font.setPixelSize(18)  # Or set size in pixels

        self.setFont(font)

        self.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextUnderIcon)

       if twist:
           self.__is_selected = True
           self.setStyleSheet("background-color: gray")

    def toggle(self):
        if self.__state:
            icon = QIcon(self.__icon2)
            text = self.__text2
        else:
            icon = QIcon(self.__icon)
            text = self.__text

        self.setIcon(icon)
        # self.setText(self.__text2)
        self.__state = not self.__state
        return self.__state

    def toggle_selected(self) -> bool:
        self.__is_selected = not self.__is_selected

        if self.__is_selected:
            self.setStyleSheet("background-color: gray")
        else:
            self.setStyleSheet(self.__style)

        return self.__is_selected

    def isSelected(self):
        return self.__is_selected
