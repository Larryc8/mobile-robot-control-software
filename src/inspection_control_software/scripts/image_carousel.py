import sys
import os
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QLabel,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QGroupBox,
    QFileDialog,
    QStyle
)
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import Qt, pyqtSignal, QTimer

from utils.patrol import PatrolEndState, userOperation, operationMode

from styles.buttons import border_button_style, secondary_button_style, border_button_style_danger

# Button styling
button_base_style = """
    QPushButton {
        height: 100%;
        padding: 8px;
        font-size: 14px;
        width: 20px;
    }
"""

thumnail_base_style = """
"""

class CustomLabel(QWidget):
    clicked = pyqtSignal(int)

    def __init__(self, text, index):
        super().__init__()
        self.label = QLabel()
        self.status_label = QLabel("EL pepe")
        self.index = index
        self.id = None
        layout = QVBoxLayout()

        layout.addWidget(self.label, 4)
        layout.addWidget(self.status_label, 1)

        self.setStyleSheet("""
            QWidget {
                background-color: gray;
                border-radius: 5px;
                font-weight: bold;
            }
        """)

        self.setLayout(layout)

    def mouseReleaseEvent(self, event):
        print(f"{__name__} hola ")
        self.clicked.emit(self.index)

    def mouseDoubleClickEvent(self, event):
        if event.button() == Qt.LeftButton:
            print("clicked one carousel")


class ImageCarousel(QWidget):
    def __init__(self, buffer):
        super().__init__()
        self.setWindowTitle("Image Carousel")
        self.setGeometry(100, 100, 800, 650)

        # Image variables
        self.buffer_data = []
        self.buffer_data = buffer
        self.current_index = 0
        self.MAX_THUMBNAILS = 4
        self.loaded_images = []
        self.data_container = []
        self.imaages_to_show = []
        self.use_filepath = False

        # UI Elements
        self.image_label = QLabel()
        self.page_label = QLabel("0/0")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setStyleSheet("border: 2px solid gray;  background-color: gray")

        # Buttons
        self.prev_button = QPushButton("<")
        self.next_button = QPushButton(">")
        self.delete_button = QPushButton("Descartar desague")

 
        self.prev_button.setStyleSheet(button_base_style + border_button_style )
        self.next_button.setStyleSheet(button_base_style + border_button_style)
        self.delete_button.setStyleSheet(secondary_button_style)
        self.delete_button.setIcon(QApplication.style().standardIcon(QStyle.SP_DialogCancelButton))

        # Button connections
        self.prev_button.clicked.connect(self.show_previous_image)
        self.next_button.clicked.connect(self.show_next_image)
        self.delete_button.clicked.connect(self.discard_buffered_data)

        # Layout
        button_layout = QHBoxLayout()
        # button_layout.addWidget(self.page_label)
        button_layout.addWidget(self.delete_button)


        layout = QHBoxLayout()
        thumbnails_layout = QHBoxLayout()

        self.images_thumbnail = [CustomLabel(text="Image", index=i) for i in range(self.MAX_THUMBNAILS)]
        [
            label.clicked.connect(self.update_thumbnail)
            for label in self.images_thumbnail
        ]

        for label in self.images_thumbnail:
            label.setStyleSheet("border: 2px solid gray; background-color: gray")
            thumbnails_layout.addWidget(label)
            label.setFixedSize(200, 170)


        layout.addWidget(self.prev_button)
        layout.addLayout(thumbnails_layout)
        layout.addWidget(self.next_button)

        main_layout = QVBoxLayout()
        main_layout.addWidget(self.image_label)
        main_layout.addLayout(layout)
        main_layout.addLayout(button_layout)

        self.setLayout(main_layout)
        self.show_empty_image()

    def update_thumbnail(self, index):
        for thumbnail in self.images_thumbnail:
            thumbnail.setStyleSheet("border: 2px solid gray;")

        label = self.images_thumbnail[index]
        label.setStyleSheet("border: 3px solid blue;")
        self.select_image(index)

    def get_user_operation(self, use_operation):
        if use_operation == userOperation.LOADMAP:
            while len(self.buffer_data):
                self.buffer_data.pop()
            self.display_all_images(data_array=[], use_filepath=False)

    def update_dreinage_info(self, current_point_id, next_point_id, point_state):
        if point_state in [2, 3]:
            x = [
                label for label in self.images_thumbnail if label.id == current_point_id
            ]
            if len(x):
                x[0].status_label.setText("Revisado!")

    def reset_dreinage_info(self, x=None, y=None):
        for thumbnail in self.images_thumbnail:
            thumbnail.status_label.setText("Pendiente")

    def load_stored_points(self, stored_points):
        while len(self.buffer_data):
            self.buffer_data.pop()

        if stored_points:
            for point in stored_points.get("points"):
                id, x_meters, y_meters, map_file, yaw, gui_yaw, image = point
                self.buffer_data.append((id, image))

            self.display_all_images(data_array=self.buffer_data[self.current_index: self.current_index + self.MAX_THUMBNAILS], use_filepath=True)

    def show_empty_image(self):
        # Create a blank pixmap
        empty_pixmap = QPixmap(600, 400)
        empty_pixmap.fill(Qt.white)
        self.image_label.setPixmap(empty_pixmap)
        self.image_label.setText("No images loaded")
        self.image_label.setAlignment(Qt.AlignCenter)

    def discard_buffered_data(self):
        if self.current_index < len(self.buffer_data):
            self.buffer_data.pop(self.current_index)
        self.display_all_images(data_array=self.buffer_data[self.current_index: self.current_index + self.MAX_THUMBNAILS], use_filepath=False)

    def load_images(self, images: list = []):
        self.display_current_image()
        self.display_all_images(data_array=self.buffer_data[self.current_index: self.current_index + self.MAX_THUMBNAILS], use_filepath=False)

    def display_all_images(self, data_array, use_filepath):
        self.data_container = data_array
        self.use_filepath = use_filepath

        for i, label in enumerate(self.images_thumbnail):
            if i < len(data_array):
                if use_filepath:
                    id, data = data_array[i]
                    pixmap = QPixmap(data)
                else:
                    id = None
                    data = data_array[i]
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
                    else:
                        pixmap = None

                    # Scale the image to fit the label while maintaining aspect ratio
                scaled_pixmap = pixmap.scaled(
                    self.image_label.width() - 10,
                    self.image_label.height() - 10,
                    Qt.KeepAspectRatio,
                    Qt.SmoothTransformation,
                )
                label.label.setPixmap(scaled_pixmap)
                label.label.setScaledContents(True)
                label.id = id
            else:
                empty_pixmap = QPixmap(600, 400)
                empty_pixmap.fill(Qt.white)
                label.label.setPixmap(empty_pixmap)
                label.label.setText(
                    "No images loaded\nClick 'Load Images' to add images"
                )
                label.label.setAlignment(Qt.AlignCenter)

    def display_current_image(self):
        if len(self.data_container) == 0:
            return

        if self.use_filepath:
            pass
            _, path = self.data_container[self.current_index]
            pixmap = QPixmap(path)
        else:
            img, file_path, pose = self.data_container[self.current_index]
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

        scaled_pixmap = pixmap.scaled(
            self.image_label.width() - 10,
            self.image_label.height() - 10,
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation,
        )

        self.image_label.setPixmap(scaled_pixmap)

    def show_next_image(self):
        if self.buffer_data is None:
            return

        if not len(self.buffer_data):
            return

        self.current_index = (self.current_index + 1) % (len(self.buffer_data)//self.MAX_THUMBNAILS)
        self.display_all_images(data_array=self.buffer_data[self.current_index: self.current_index + self.MAX_THUMBNAILS], use_filepath=True)

    def show_previous_image(self):
        if self.buffer_data is None:
            return

        if not len(self.buffer_data):
            return

        self.current_index = (self.current_index - 1) % (len(self.buffer_data)//self.MAX_THUMBNAILS)
        self.display_all_images(data_array=self.buffer_data[self.current_index: self.current_index + self.MAX_THUMBNAILS], use_filepath=True)

    def select_image(self, index):
        if self.buffer_data is None:
            return

        self.current_index = index
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
