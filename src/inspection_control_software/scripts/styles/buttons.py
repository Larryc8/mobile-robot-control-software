# 000000 - Black

# 696969 - Dim Gray

# 808080 - Gray

# A9A9A9 - Dark Gray

# C0C0C0 - Silver

# D3D3D3 - Light Gray

# DCDCDC - Gainsboro

# F5F5F5 - White Smoke

# FFFFFF - White
#
color = '#2C3E50'


menu_style = """
    QMenu {
        background-color: white;
        border: 1px solid #bdc3c7;
        border-radius: 3px;
        padding: 5px;
        color: gray;
    }
    QMenu::item {
        padding: 5px 18px;
        border-radius: 3px;
        color: black;
        background-color: transparent;
    }
    QMenu::item:selected {
        background-color: black;
        color: white;
    }
    QMenu::item:disabled {
        background-color: transparent;
        color: gray;
    }
    QMenu::separator {
        height: 1px;
        background-color: lightgray;
        margin: 2px 0px;
    }
"""

button_with_menu_style = """
    QPushButton::menu-indicator {
        image: url(public/menu-4-fill-svgrepo-com.svg);
        subcontrol-position: right center;
        subcontrol-origin: padding;
        padding-right: 15px;
        width: 16px;
        height: 16px;
    }
"""

border_button_style = """
    QPushButton {
        background: transparent;
        color: #2C3E50;
        border: 1px solid #2C3E50;
        border-radius: 2px;
        padding: 6px 6px;
        font-weight: bold;
    }
    QPushButton:hover {
        background: #DCDCDC ;
    }
    QPushButton:pressed {
        background-color: #3e8e41;
    }

    QPushButton:disabled {
        border-color: lightgray;
        color: lightgray
    }
"""

border_button_style_danger = """
    QPushButton {
        background: transparent;
        color: #2C3E50;
        border: 1px solid #2C3E50;
        border-radius: 2px;
        padding: 6px 6px;
        font-weight: bold;
    }
    QPushButton:hover {
        background: #DCDCDC ;
    }
    QPushButton:pressed {
        background-color: red;
    }
"""


colored_button_style = """
    QPushButton {
        background-color: #2C3E50;
        border-radius: 2px;
        font-weight: bold;
        color: white;
        border: none;
        padding: 6px 6px;
    }
    QPushButton:hover {
        background-color: #45a049;
    }
    QPushButton:pressed {
        background-color: #3e8e41;
    }
    QPushButton:disabled {
        background-color: transparent;
        border: 1px solid lightgray;
        color: lightgray
    }
"""


primary_button_style = """
    QPushButton {
        background-color: #4682B4;
        border-radius: 2px;
        color: white;
        border: none;
        padding: 6px 6px;
        font-size: 16px;
    }
    QPushButton:hover {
        background-color: #B0BEC5;
    }
    QPushButton:pressed {
        background-color: #4682B4;
    }

    QPushButton:disabled {
        background-color: gray;
        border: 1px solid gray;
    }
"""

secondary_button_style = """
    QPushButton {
        background-color: lightgray;
        border-radius: 2px;
        color: black;
        border: none;
        padding: 6px 6px;
        font-size: 16px;
    }
    QPushButton:hover {
        background: #F5F5F5;
        border: 1px solid lightgray;
    }
    QPushButton:pressed {
        background-color: black;
        color: white
    }
"""

tertiary_button_style = """
    QPushButton {
        background: #DCDCDC ;
        color: gray;
        border: none;
        padding: 5px;
        text-align: left;
        font-size: 16px;
    }
    QPushButton:hover {
        background: #F5F5F5;
    }
"""

toggle_button_style = """
    QPushButton {
        border: none;
        background-color: transparent;
        color: #2C3E50;
        font-weight: bold;
        padding: 6px 6px;
    }
    QPushButton:hover {
        background-color: lightgray;
    }
    QPushButton:disabled {
        border-radius: 2px;
        background-color: #2C3E50;
        color: white;
    }

"""
# minimal_button_style =
minimal_button_style = """
    QPushButton {
        background-color: lightgray;
        border-radius: 2px;
        color: black;
        border: none;
        padding: 3px 3px;
        font-size: 16px;
        font-weight: bold;
    }
    QPushButton:hover {
        background-color: gray;
    }
    QPushButton:pressed {
        background-color: lightgray;
    }
    QPushButton:disabled {
        border: 1px solid gray;
        color: gray
    }
"""

tag_button_style = """
    QPushButton {
        background-color: #e0e0e0;
        border: 1px solid #b0b0b0;
        border-radius: 10px;
        padding:  6px;
        color: #333333;
        font-size: 14px;
    }

    QPushButton:hover {
        background-color: #d0d0d0;
    }

    QPushButton:pressed {
        background-color: #c0c0c0;
    }
"""


tag_selected_button_style = """
            QPushButton {
                background-color: #4682B4;
                border: 1px solid #90caf9;
                border-radius: 10px;
                padding: 6px;
                color: white;
                font-size: 14px;
                font-weight: bold;
            }
        """

patrol_checkbox_style = """
    QCheckBox {
        font-size: 14px;
        color: #424242;
    }
    QCheckBox::indicator {
        width: 15px;
        height: 15px;
        border: 1px solid #757575;
        border-radius: 2px;
    }
    QCheckBox::indicator:checked {
        background-color: #2C3E50;
        border: 1px solid gray;
    }
    QCheckBox::indicator:hover {
        border: 1px solid #424242;
    }
    QCheckBox::indicator:checked:hover {
        background-color: #d5ffff;
        border: 1px solid gray;
    }
    QCheckBox::indicator:unchecked:hover {
        background: lightgray;
    }
"""

modern_checkbox_style = """
    QCheckBox {
        spacing: 8px;
        font-size: 14px;
        color: #424242;
    }
    QCheckBox::indicator {
        width: 18px;
        height: 18px;
        border: 2px solid #757575;
        border-radius: 2px;
    }
    QCheckBox::indicator:checked {
        background-color: #6200ee;
        border: 2px solid #6200ee;
        image: url(:/icons/check.svg);
    }
    QCheckBox::indicator:hover {
        border: 2px solid #424242;
    }
    QCheckBox::indicator:checked:hover {
        background-color: #3700b3;
        border: 2px solid #3700b3;
    }
"""

common_slider_style = """
    QSlider::handle:horizontal {
        width: 20px;
        height: 12px;
        margin: -4px 0;
        background:  white;
        border: 1px solid #777;
        border-radius: 3px;
    }

    QSlider::tick-mark:horizontal {
        height: 4px;
        width: 1px;
        background: #999;
    }
"""


simple_slider_rightleft_style = (
    """
    QSlider::groove:horizontal {
        height: 8px;
        background:  #4682B4;
        border-radius: 1px;
    }

    QSlider::sub-page:horizontal {
        background: lightgray;
        border-radius: 1px;
    }
"""
    + common_slider_style
)

simple_slider_leftright_style = (
    """
    QSlider::groove:horizontal {
        height: 8px;
        background: lightgray;
        border-radius: 1px;
    }

    QSlider::sub-page:horizontal {
        background: #4682B4;
        border-radius: 1px;
    }
"""
    + common_slider_style
)

modern_line_edit_style = """
    QLineEdit {
        border: none;
        border-bottom: 2px solid #bdbdbd;
        background: transparent;
        padding: 5px;
        font-size: 16px;
    }

    QLineEdit:focus {
        border-bottom: 2px solid #2196F3;
    }
"""
simple_line_edit_style = """
    QLineEdit {
        background-color: #f0f0f0;
        border: 2px solid #ccc;
        border-radius: 5px;
        padding: 5px;
        font-size: 14px;
    }

    QLineEdit:focus {
        border: 2px solid #6a9fb5;
        background-color: #fff;
    }
"""
error_simple_line_edit_style = """
    QLineEdit {
        background-color: #fff3f3;
        border: 1px solid #ffcccc;
        border-radius: 3px;
        padding: 5px;
    }

    QLineEdit:focus {
        border: 1px solid #ff9999;
    }
"""
