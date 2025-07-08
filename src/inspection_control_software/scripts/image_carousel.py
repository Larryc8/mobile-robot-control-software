import sys
import os
from PyQt5.QtWidgets import (QApplication, QWidget, QLabel, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QFileDialog)
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import Qt, QTimer


class CustomLabel(QLabel):
    def __init__(self, text):
        super().__init__(text)

    # def releaseMouse(self, event):
    def mouseDoubleClickEvent(self, event):
        if event.button() == Qt.LeftButton:
            print("clicked one carousel")
        pass

class ImageCarousel(QWidget):
    def __init__(self, buffer):
        super().__init__()
        self.setWindowTitle("Image Carousel")
        self.setGeometry(100, 100, 800, 600)
        
        # Image variables
        self.buffer_data = []
        self.buffer_data = buffer
        self.current_index = 0
        
        # UI Elements
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setStyleSheet("border: 2px solid gray;")
        
        # Buttons
        self.prev_button = QPushButton("Previous")
        self.next_button = QPushButton("Next")
        self.pause_button = QPushButton("Borrar")
        
        # Button styling
        button_style = """
            QPushButton {
                padding: 8px;
                font-size: 14px;
                min-width: 80px;
            }
        """
        self.prev_button.setStyleSheet(button_style)
        self.next_button.setStyleSheet(button_style)
        self.pause_button.setStyleSheet(button_style)
        
        # Button connections
        self.prev_button.clicked.connect(self.show_previous_image)
        self.next_button.clicked.connect(self.show_next_image)
        self.pause_button.clicked.connect(self.discard_buffered_data)
        
        # Layout
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.pause_button)
        button_layout.addWidget(self.prev_button)
        button_layout.addWidget(self.next_button)
        
        main_layout = QVBoxLayout()
        main_layout.addWidget(self.image_label)
        main_layout.addLayout(button_layout)

        mini_images_layout = QHBoxLayout()
        
        self.mini_images_viz = [CustomLabel('Image') for i in range(5)]

        for label in self.mini_images_viz:
            label.setStyleSheet("border: 2px solid gray;")
            mini_images_layout.addWidget(label) 
            label.setFixedSize(200, 150)
 
        main_layout.addLayout(mini_images_layout)
        self.setLayout(main_layout)
        self.show_empty_image()
    
    def show_empty_image(self):
        # Create a blank pixmap
        empty_pixmap = QPixmap(600, 400)
        empty_pixmap.fill(Qt.white)
        self.image_label.setPixmap(empty_pixmap)
        self.image_label.setText("No images loaded\nClick 'Load Images' to add images")
        self.image_label.setAlignment(Qt.AlignCenter)
    
    def discard_buffered_data(self):
        if self.current_index < len(self.buffer_data):
            empty_pixmap = QPixmap(600, 400)
            empty_pixmap.fill(Qt.white)
            image_label = self.mini_images_viz[self.current_index]
            image_label.setPixmap(empty_pixmap)
            image_label.setText("No images loaded\nClick 'Load Images' to add images")
            image_label.setAlignment(Qt.AlignCenter)
            self.buffer_data.pop(self.current_index)
        self.display_all_images()


    def load_images(self, images: list = []):
        # Open file dialog to select images
        # files, _ = QFileDialog.getOpenFileNames(
        #     self, "Select Images", "", 
        #     "Image Files (*.png *.jpg *.jpeg *.bmp *.gif)"
        # )
        
        # if files:
        #     self.image_files = files
        #     self.current_index = 0
        #     self.display_current_image()
        # print(f'{__name__} images loaded!', type(images[0]))
        # self.image_files = images
        # for img, file_path, pose in self.buffer_data: 
        #     if img is not None:
        #         height, width, channel = img.shape
        #         bytes_per_line = 3 * width
        #         q_img = QImage(
        #             img.data,
        #             width,
        #             height,
        #             bytes_per_line,
        #             QImage.Format_RGB888,
        #         ).rgbSwapped()

        #         pixmap = QPixmap.fromImage(q_img)
        #         self.image_files.append(pixmap)

        self.display_current_image()
        self.display_all_images()

    def display_all_images(self):
        for label, data in zip(self.mini_images_viz, self.buffer_data):

            img, file_path, pose = data
            if img is not None:
                height, width, channel = img.shape
                bytes_per_line = 3 * width
                q_img = QImage(
                    img.data,
                    width,
                    height,
                    bytes_per_line,
                    QImage.Format_RGB888,
                ).rgbSwapped()

                pixmap = QPixmap.fromImage(q_img)
            
                # Scale the image to fit the label while maintaining aspect ratio
                scaled_pixmap = pixmap.scaled(
                    self.image_label.width() - 10, 
                    self.image_label.height() - 10, 
                    Qt.KeepAspectRatio,
                    Qt.SmoothTransformation
                )
                label.setPixmap(scaled_pixmap)
                label.setText("")
                label.setScaledContents(True)
        
    def display_current_image(self):
        if len(self.buffer_data) == 0:
            self.show_empty_image()
            return
            
        img, file_path, pose = self.buffer_data[self.current_index]
        if img is not None:
            height, width, channel = img.shape
            bytes_per_line = 3 * width
            q_img = QImage(
                img.data,
                width,
                height,
                bytes_per_line,
                QImage.Format_RGB888,
            ).rgbSwapped()

            pixmap = QPixmap.fromImage(q_img)
        
            # Scale the image to fit the label while maintaining aspect ratio
            scaled_pixmap = pixmap.scaled(
                self.image_label.width() - 10, 
                self.image_label.height() - 10, 
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
            
            self.image_label.setPixmap(scaled_pixmap)
            self.image_label.setText("")
    
    def show_next_image(self):
        if  self.buffer_data is None:
            return
            
        self.current_index = (self.current_index + 1) % len(self.buffer_data)
        self.display_current_image()
    
    def show_previous_image(self):
        if self.buffer_data is  None:
            return
            
        self.current_index = (self.current_index - 1) % len(self.buffer_data)
        self.display_current_image()
    
    
    def resizeEvent(self, event):
        # Redisplay the current image when window is resized
        self.display_current_image()
        super().resizeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    carousel = ImageCarousel()
    carousel.show()
    sys.exit(app.exec_())
