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
        padding: 10px;
        border: 1px solid #ef9a9a;
        border-radius: 5px;
        font-weight: bold;
    }
""")
        
