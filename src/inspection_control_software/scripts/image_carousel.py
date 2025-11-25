import os
import sys

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QImage, QKeySequence, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QFileDialog,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QShortcut,
    QStyle,
    QVBoxLayout,
    QWidget,
)
from styles.buttons import (
    border_button_style,
    border_button_style_danger,
    secondary_button_style,
)
from utils.patrol import (
    PatrolEndState,
    checkpointEndState,
    operationMode,
    userOperation,
)

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
                background-color: lightgray;
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
        self.use_filepath = False
        self.current_index = 0
        self.MAX_THUMBNAILS = 3
        self.loaded_images = []
        self.data_container = []
        self.imaages_to_show = []
        self.use_filepath = False
        self.data_model = {}

        # UI Elements
        self.image_label = QLabel()
        self.page_label = QLabel("0/0")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setStyleSheet(
            "border: 2px solid gray;  background-color: gray"
        )

        # Buttons
        self.prev_button = QPushButton("<")
        self.next_button = QPushButton(">")
        self.delete_button = QPushButton("Descartar desague")

        self.prev_button.setStyleSheet(button_base_style + border_button_style)
        self.next_button.setStyleSheet(button_base_style + border_button_style)
        self.delete_button.setStyleSheet(secondary_button_style)
        self.delete_button.setIcon(
            QApplication.style().standardIcon(QStyle.SP_DialogCancelButton)
        )

        # Button connections
        self.prev_button.clicked.connect(self.show_previous_image)
        self.next_button.clicked.connect(self.show_next_image)
        self.delete_button.clicked.connect(self.discard_buffered_data)

        # Layout
        button_layout = QHBoxLayout()
        # button_layout.addWidget(self.page_label)
        button_layout.addWidget(self.delete_button)
        self.delete_button.hide()

        layout = QHBoxLayout()
        thumbnails_layout = QHBoxLayout()

        self.images_thumbnail = [
            CustomLabel(text="Image", index=i) for i in range(self.MAX_THUMBNAILS)
        ]
        [
            label.clicked.connect(self.update_thumbnail)
            for label in self.images_thumbnail
        ]

        for label in self.images_thumbnail:
            label.setStyleSheet("border: 2px solid #A9A9A9; background-color: #808080")
            thumbnails_layout.addWidget(label)
            label.setFixedSize(200, 170)

        layout.addWidget(self.prev_button)
        layout.addLayout(thumbnails_layout)
        layout.addWidget(self.next_button)

        main_layout = QVBoxLayout()
        # main_layout.addWidget(
        #     QLabel(
        #         "fotos de referencia para la inspeccion tomadas de la zonas de interes"
        #     )
        # )
        main_layout.addWidget(self.image_label)
        main_layout.addLayout(layout)
        main_layout.addLayout(button_layout)

        self.setLayout(main_layout)
        self.show_empty_image()

    def create_data_model(self) -> None:
        pass

    def update_thumbnail(self, index):
        for thumbnail in self.images_thumbnail:
            thumbnail.setStyleSheet("border: 2px solid gray;")

        label = self.images_thumbnail[index]
        label.setStyleSheet("border: 3px solid blue; background-color: #A9A9A9")
        self.select_image(index)

    def get_user_operation(self, use_operation):
        if use_operation == userOperation.CREATEMAP:
            self.delete_button.show()
            return

        if use_operation == userOperation.LOADMAP:
            self.use_filepath = True
            while len(self.buffer_data):
                self.buffer_data.pop()
            self.display_all_images(data_array=[], use_filepath=self.use_filepath)
        self.delete_button.hide()

    def update_dreinage_info(self, current_point_id, next_point_id, point_state):
        if point_state in [2, 3]:
            x = [
                label for label in self.images_thumbnail if label.id == current_point_id
            ]
            self.data_model[current_point_id] = (
                checkpointEndState.CHECKED.value
            )  # "<span style='color: green; font-weight: bold'>Bueno</span>"
            if len(x):
                x[0].status_label.setText(checkpointEndState.CHECKED.value)

    def reset_dreinage_status(self, x=None, y=None):
        for thumbnail in self.images_thumbnail:
            thumbnail.status_label.setText(checkpointEndState.PENDING.value)

        for key, value in self.data_model.items():
            self.data_model[key] = checkpointEndState.PENDING.value

    def load_stored_points(self, stored_points):
        while len(self.buffer_data):
            self.buffer_data.pop()

        self.use_filepath = True

        if stored_points:
            for point in stored_points.get("points"):
                id, x_meters, y_meters, map_file, yaw, gui_yaw, image, *_ = point
                self.buffer_data.append((id, image))
                self.data_model[id] = checkpointEndState.PENDING.value

            self.display_all_images(
                data_array=self.buffer_data[
                    self.current_index : self.current_index + self.MAX_THUMBNAILS
                ],
                use_filepath=self.use_filepath,
            )

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
        self.display_all_images(
            data_array=self.buffer_data[
                self.current_index : self.current_index + self.MAX_THUMBNAILS
            ],
            use_filepath=self.use_filepath,
        )

    def load_images(self, images: list = []):
        self.use_filepath = False
        self.display_current_image()
        self.display_all_images(
            data_array=self.buffer_data[
                self.current_index : self.current_index + self.MAX_THUMBNAILS
            ],
            use_filepath=self.use_filepath,
        )
        pass

    def display_all_images(self, data_array, use_filepath):
        self.data_container = data_array
        self.use_filepath = use_filepath

        for i, label in enumerate(self.images_thumbnail):
            label.status_label.setText(checkpointEndState.NONE.value)
            if i < len(data_array):
                if use_filepath:
                    id, data = data_array[i]
                    pixmap = QPixmap(data)
                else:
                    id = None
                    data = data_array[i]
                    img, file_path, pose, *_ = data
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
                if id:
                    label.status_label.setText(self.data_model[id])
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

        if self.current_index >= len(self.data_container):
            return

        if self.use_filepath:
            pass
            _, path = self.data_container[self.current_index]
            pixmap = QPixmap(path)
        else:
            img, file_path, pose, *_ = self.data_container[self.current_index]
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
        print(f"buffer dat len {len(self.buffer_data)}")
        if self.buffer_data is None:
            return

        if not len(self.buffer_data):
            return

        if len(self.buffer_data) % self.MAX_THUMBNAILS:
            pages = (len(self.buffer_data) // self.MAX_THUMBNAILS) + 1
        else:
            pages = len(self.buffer_data) // self.MAX_THUMBNAILS

        if len(self.buffer_data) < self.MAX_THUMBNAILS:
            self.current_index = 0  # (self.current_index - 1) % 1
        else:
            self.current_index = (self.current_index + 1) % pages

        # self.display_all_images(data_array=self.buffer_data[self.current_index: self.current_index + self.MAX_THUMBNAILS], use_filepath=self.use_filepath)
        self.display_all_images(
            data_array=self.buffer_data[
                self.current_index * self.MAX_THUMBNAILS : self.current_index
                * self.MAX_THUMBNAILS
                + self.MAX_THUMBNAILS
            ],
            use_filepath=self.use_filepath,
        )

    def show_previous_image(self):
        if self.buffer_data is None:
            return

        if not len(self.buffer_data):
            return

        if len(self.buffer_data) % self.MAX_THUMBNAILS:
            pages = (len(self.buffer_data) // self.MAX_THUMBNAILS) + 1
        else:
            pages = len(self.buffer_data) // self.MAX_THUMBNAILS

        if not (len(self.buffer_data) // self.MAX_THUMBNAILS):
            self.current_index = 0  # (self.current_index - 1) % 1
        else:
            self.current_index = (self.current_index - 1) % pages

        self.display_all_images(
            data_array=self.buffer_data[
                self.current_index * self.MAX_THUMBNAILS : self.current_index
                * self.MAX_THUMBNAILS
                + self.MAX_THUMBNAILS
            ],
            use_filepath=self.use_filepath,
        )

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
