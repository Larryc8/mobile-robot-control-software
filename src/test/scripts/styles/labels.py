hover_label_style = ("""
    QLabel:hover {
        background-color: green;
    }
""")
inactive_hover_label_style = ("""
    QLabel:hover {
        background-color: green;
    }
""")
border_label_style = ("""
    QLabel {
        background-color: blue; 
        color: #2c3e50;
        border: 1px solid rgba(44, 62, 80, 100);
        border-radius: 3px;
        padding: 2px 5px;
        color: white;
    }
""" + hover_label_style )

inactive_label_style = ("""
    QLabel {
        background-color: gray; 
        color: #2c3e50;
        border: 1px solid rgba(44, 62, 80, 100);
        border-radius: 3px;
        color: white;
        padding: 2px 5px;
    }
""" + hover_label_style  )

minimal_label_style = ("""
    QLabel {
        background-color: gray; 
        color: #2c3e50;
        font-size: 16px;
        border: 1px solid rgba(44, 62, 80, 100);
        border-radius: 3px;
        color: white;
        padding: 2px 5px;
    }
""" )

# self.success_label.setText("✓ SUCCESS: File saved successfully")
succes_label_style = ("""
    QLabel {
        color: #388e3c;
        background-color: #e8f5e9;
        padding: 2px;
        border: 1px solid #a5d6a7;
        border-radius: 5px;
        font-weight: bold;
    }
""")

# self.info_label = QLabel()
# self.info_label.setText("ℹ INFO: Operation completed successfully")
info_label_style = ("""
    QLabel {
        color: #0288d1;
        background-color: #e1f5fe;
        padding: 5px;
        border: 1px solid #81d4fa;
        border-radius: 3px;
    }
""")

# self.warning_label = QLabel()
# self.warning_label.setText("⚠ WARNING: This action cannot be undone")
warning_label_style = ("""
    QLabel {
        color: #ff8f00;
        background-color: #fff8e1;
        padding: 5px;
        border: 1px solid #ffe082;
        border-radius: 5px;
    }
""")

# self.error_label.setText("✖ ERROR: Invalid input detected")
error_label_style = ("""
    QLabel {
        color: #d32f2f;
        background-color: #ffebee;
        padding: 5px;
        border: 1px solid #ef9a9a;
        border-radius: 5px;
        font-weight: bold;
    }
""")

# title_label = QLabel("Main Title")
title_label_style = ("""
    QLabel {
        font-size: 24px;
        font-weight: bold;
        color: #333333;
        padding: 10px 0;
        margin-bottom: 10px;
    }
""")

# subtitle_label = QLabel("Section Subtitle")
subtitle_label_style = ("""
    QLabel {
        font-size: 18px;
        font-weight: 600;
        color: #444444;
        padding: 8px 0;
        margin-bottom: 8px;
        border-bottom: 1px solid #e0e0e0;
    }
""")

# section_header = QLabel("Section Header")
section_header_label_style = ("""
    QLabel {
        font-size: 16px;
        font-weight: 500;
        color: #2c3e50;
        background-color: #f8f9fa;
        padding: 6px 12px;
        border-radius: 4px;
        margin: 15px 0 5px 0;
    }
""")


# muted_label = QLabel("Secondary information")
muted_label_style = ("""
    QLabel {
        font-size: 15px;
        color: #777777;
        padding: 2px 0;
    }
""")

# code_label = QLabel("print('Hello World')")
code_label_style = ("""
    QLabel {
        font-family: monospace;
        font-size: 13px;
        color: #333;
        background-color: #f5f5f5;
        padding: 6px 10px;
        border-radius: 4px;
        border: 1px solid #ddd;
    }
""")
