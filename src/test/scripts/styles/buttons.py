border_button_style = ("""
    QPushButton {
        background: transparent;
        color: #3498db;
        border: 1px solid #3498db;
        border-radius: 2px;
        padding: 6px 6px;
    }
    QPushButton:hover {
        background: #3498db;
        color: white;
    }    
    QPushButton:pressed {
        background-color: #3e8e41;
    }
""")

border_button_style_danger = ("""
    QPushButton {
        background: transparent;
        color: #3498db;
        border: 1px solid #3498db;
        border-radius: 2px;
        padding: 6px 6px;
    }
    QPushButton:hover {
        background: pink;
        color: white;
    }    
    QPushButton:pressed {
        background-color: red;
    }
""")




colored_button_style = ("""
    QPushButton {
        background-color: #2C3E50;
        border-radius: 2px;
        color: white;
        border: none;
        padding: 6px 6px;
        font-size: 16px;
    }
    QPushButton:hover {
        background-color: #45a049;
    }
    QPushButton:pressed {
        background-color: #3e8e41;
    }
""")


primary_button_style = ("""
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
""")

secondary_button_style = ("""
    QPushButton {
        background-color: lightgray;
        border-radius: 2px;
        color: black;
        border: none;
        padding: 6px 6px;
        font-size: 16px;
    }
    QPushButton:hover {
        background-color: #45a049;
    }
    QPushButton:pressed {
        background-color: #3e8e41;
    }
""")

tertiary_button_style = ("""
    QPushButton {
        background: none;
        color: black;
        border: none;
        padding: 5px;
        text-align: left;
        font-size: 16px;
        font-weight: bold;
    }
    QPushButton:hover {
        color: magenta;
        text-decoration: underline;
    }
""")

toggle_button_style = """
    QPushButton {
        border: none;
        background-color: transparent;
        color: #4682B4;
        font-weight: bold;
        padding: 6px 6px;
    }
    QPushButton:hover {
        background-color: lightgray;
    }
    QPushButton:disabled {
        border: 1px solid #8f8f91;
        border-radius: 2px;
        background-color: #4682B4;
        color: #E0E0E0;
    }

"""
# minimal_button_style = 
minimal_button_style = ("""
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
""")


patrol_checkbox_style = ("""
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
        background-color: red;
        border: 2px solid #6200ee;
    }
    QCheckBox::indicator:hover {
        border: 2px solid #424242;
    }
    QCheckBox::indicator:checked:hover {
        background-color: pink;
        border: 2px solid #3700b3;
    }
    QCheckBox::indicator:unchecked:hover {
        background: #bbb;
    }
""")

modern_checkbox_style = ("""
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
""")
