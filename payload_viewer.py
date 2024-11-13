import sys
import cv2
import numpy as np
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout, QMenuBar, QMenu,
                             QPushButton, QLabel, QSlider, QStackedWidget, QFrame, QDialog, QFormLayout, QLineEdit, QDialogButtonBox)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QImage, QPixmap, QAction
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtCore import QUrl

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # Set the main window properties
        self.setWindowTitle("Enhanced Viewer UI")
        self.setGeometry(100, 100, 1400, 800)  # Adjust width and height as needed
        self.setStyleSheet("background-color: #2E2E2E; color: #FFFFFF;")  # Dark theme with white text
        
        # Create the main widget and layout
        main_widget = QWidget()
        main_layout = QHBoxLayout(main_widget)

        # Left column widget and layout
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        left_widget.setLayout(left_layout)
        left_widget.setStyleSheet("background-color: #3A3A3A; border-radius: 10px; padding: 10px;")

        # Stacked widget for different views
        self.view_stack = QStackedWidget()
        self.view_stack.setContentsMargins(20, 20, 20, 20)  # Scaled down 16:9 resolution for 1920x1080
        self.view_stack.setStyleSheet("padding: 10px; border-radius: 20px; background-color: #3A3A3A;")
        left_layout.addWidget(self.view_stack)

        # Webcam view
        self.webcam_view = QLabel()
        self.webcam_view.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.view_stack.addWidget(self.webcam_view)

        # Map view
        self.map_view = QWebEngineView()
        self.map_view.setUrl(QUrl('https://www.openstreetmap.org'))  # Load a basic map from OpenStreetMap
        self.view_stack.addWidget(self.map_view)

        # Images view
        self.images_view = QLabel("Images View")
        self.images_view.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.view_stack.addWidget(self.images_view)

        # Current view label
        self.current_view_label = QLabel("Current View: Webcam")
        self.current_view_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.current_view_label.setStyleSheet("font-size: 16px; padding-top: 10px;")
        left_layout.addWidget(self.current_view_label)

        # Right column widget and layout
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_widget.setLayout(right_layout)
        right_widget.setStyleSheet("background-color: #3A3A3A; border-radius: 10px; padding: 10px;")

        # Top section of the right column
        top_section_widget = QFrame()
        top_section_layout = QVBoxLayout(top_section_widget)
        top_section_widget.setStyleSheet(" border-radius: 10px; padding: 10px;")

        self.top_button = QPushButton("Auto Adjust")
        self.top_button.setFixedHeight(50)
        self.top_button.setStyleSheet("QPushButton { background-color: #5A5A5A; border: none; border-radius: 5px; font-size: 16px; color: #FFFFFF; } QPushButton::pressed { background-color: #4A4A4A; }")
        self.top_button.setEnabled(False)
        self.top_button_state = False
        self.top_button.clicked.connect(self.toggle_auto_adjust)
        top_section_layout.addWidget(self.top_button)

        top_label = QLabel("Feature Status: Inactive")
        top_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        top_label.setStyleSheet("font-size: 14px;")
        top_section_layout.addWidget(top_label)

        right_layout.addWidget(top_section_widget, 1)

        # Bottom section of the right column
        bottom_section_widget = QFrame()
        bottom_section_layout = QVBoxLayout(bottom_section_widget)
        bottom_section_widget.setStyleSheet(" border-radius: 10px; padding: 10px;")

        bottom_button1 = QPushButton("Retract")
        bottom_button1.setFixedHeight(50)
        bottom_button1.setStyleSheet("QPushButton { background-color: #5A5A5A; border: none; border-radius: 5px; font-size: 16px; } QPushButton::Hover { background-color: #6A6A6A } QPushButton::pressed { background-color: #4A4A4A; }")
        bottom_button1.clicked.connect(self.retract_button_clicked)
        self.bottom_button1 = bottom_button1
        bottom_section_layout.addWidget(bottom_button1)

        bottom_button2 = QPushButton("Extend")
        bottom_button2.setFixedHeight(50)
        bottom_button2.setStyleSheet("QPushButton { background-color: #5A5A5A; border: none; border-radius: 5px; font-size: 16px; } QPushButton::Hover { background-color: #6A6A6A } QPushButton::pressed { background-color: #4A4A4A; }")
        bottom_button2.clicked.connect(self.extend_button_clicked)
        self.bottom_button2 = bottom_button2
        bottom_section_layout.addWidget(bottom_button2)

        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.slider.setTickInterval(1)
        self.slider.setMaximum(10)
        self.slider.setMinimum(0)
        self.slider.setFixedHeight(50)
        self.slider.setStyleSheet("border-radius: 5px;")
        self.slider.valueChanged.connect(self.slider_value_changed)
        bottom_section_layout.addWidget(self.slider)
        right_layout.addWidget(bottom_section_widget, 1)

        # Set stretch for columns (3:1 ratio)
        main_layout.addWidget(left_widget, 3)  # Takes up 3/4 of the space
        main_layout.addWidget(right_widget, 1)  # Takes up 1/4 of the space
        
        # Set the main widget as the central widget
        self.setCentralWidget(main_widget)

        # Create menu bar
        menu_bar = self.menuBar()
        menu_bar.setStyleSheet("background-color: #3A3A3A; color: #FFFFFF;")

        # Add File menu
        file_menu = menu_bar.addMenu("File")
        
        # Add Settings menu
        settings_menu = menu_bar.addMenu("Settings")

        # Add Edit menu
        edit_menu = menu_bar.addMenu("Edit")

        # Add View Selector to Settings menu
        view_selector_menu = QMenu("View Selector", self)
        settings_menu.addMenu(view_selector_menu)
        settings_menu.setStyleSheet("QMenu::item { background-color: #3A3A3A; color: #FFFFFF; } QMenu::item:selected { background-color: #5A5A5A; }")

        # Populate View Selector with view options
        views = ["Webcam", "Map", "Images"]
        for index, view in enumerate(views):
            action = QAction(view, self)
            action.setData(index)
            action.triggered.connect(lambda checked, idx=index: self.change_view(idx))
            view_selector_menu.addAction(action)
            view_selector_menu.setStyleSheet("QMenu::item { background-color: #2E2E2E; color: #FFFFFF; } QMenu::item:selected { background-color: #5A5A5A; }")

        # Add slider settings option to Edit menu
        slider_settings_action = QAction("Slider Settings", self)
        slider_settings_action.triggered.connect(self.open_slider_settings_dialog)
        edit_menu.addAction(slider_settings_action)
        edit_menu.setStyleSheet("QMenu::item { background-color: #3A3A3A; color: #FFFFFF; } QMenu::item:selected { background-color: #5A5A5A; }")

        # Webcam setup
        self.cap = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.start_webcam()

    def change_view(self, index):
        self.view_stack.setCurrentIndex(index)
        view_names = ["Webcam", "Map", "Images"]
        self.current_view_label.setText(f"Current View: {view_names[index]}")
        if index == 0:  # Webcam view
            self.start_webcam()
        else:
            self.stop_webcam()

    def start_webcam(self):
        if self.cap is None:
            self.cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)  # Hardcoded to use camera with index 1
        if not self.cap.isOpened():
            return
        if not self.timer.isActive():
            self.timer.start(30)

    def stop_webcam(self):
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
        self.cap = None
        self.timer.stop()
        self.webcam_view.clear()

    def update_frame(self):
        if self.cap is not None and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                image = QImage(frame, frame.shape[1], frame.shape[0], QImage.Format.Format_RGB888)
                pixmap = QPixmap.fromImage(image).scaled(1280, 720, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)
                self.webcam_view.setPixmap(pixmap)

    def open_slider_settings_dialog(self):
        dialog = QDialog(self)
        dialog.setWindowTitle("Slider Settings")
        dialog.setStyleSheet("background-color: #3A3A3A; color: #FFFFFF; border-radius: 10px; padding: 10px;")

        layout = QFormLayout(dialog)

        min_input = QLineEdit(str(self.slider.minimum()))
        min_input.setStyleSheet("background-color: #5A5A5A; border: none; border-radius: 5px; color: #FFFFFF;")
        max_input = QLineEdit(str(self.slider.maximum()))
        max_input.setStyleSheet("background-color: #5A5A5A; border: none; border-radius: 5px; color: #FFFFFF;")
        step_input = QLineEdit(str(self.slider.tickInterval()))
        step_input.setStyleSheet("background-color: #5A5A5A; border: none; border-radius: 5px; color: #FFFFFF;")

        layout.addRow("Min Value:", min_input)
        layout.addRow("Max Value:", max_input)
        layout.addRow("Step:", step_input)

        button_box = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        button_box.accepted.connect(lambda: self.update_slider_settings(min_input, max_input, step_input, dialog))
        button_box.rejected.connect(dialog.reject)
        button_box.setStyleSheet("QPushButton { background-color: #5A5A5A; border: none; border-radius: 5px; color: #FFFFFF; } QPushButton::pressed { background-color: #4A4A4A; }")
        layout.addRow(button_box)

        dialog.exec()

    def update_slider_settings(self, min_input, max_input, step_input, dialog):
        try:
            min_value = int(min_input.text())
            max_value = int(max_input.text())
            step_value = int(step_input.text())

            self.slider.setMinimum(min_value)
            self.slider.setMaximum(max_value)
            self.slider.setTickInterval(step_value)

            dialog.accept()
        except ValueError:
            pass

    # Button and slider functions
    def toggle_auto_adjust(self):
        if self.top_button_state:
            self.top_button.setStyleSheet("QPushButton { background-color: #8B0000; border: none; border-radius: 5px; font-size: 16px; color: #FFFFFF; } QPushButton::pressed { background-color: #4A4A4A; }")
            self.top_button.setText("Auto Adjust: Off")
        else:
            self.top_button.setStyleSheet("QPushButton { background-color: #006400; border: none; border-radius: 5px; font-size: 16px; color: #FFFFFF; } QPushButton::pressed { background-color: #4A4A4A; }")
            self.top_button.setText("Auto Adjust: On")
        self.top_button_state = not self.top_button_state

    def extend_button_clicked(self):
        self.top_button.setEnabled(True)
        self.top_button.setStyleSheet("QPushButton { background-color: #5A5A5A; border: none; border-radius: 5px; font-size: 16px; color: #FFFFFF; } QPushButton::pressed { background-color: #4A4A4A; }")
        self.bottom_button2.setStyleSheet("QPushButton { background-color: #006400; border: none; border-radius: 5px; font-size: 16px; color: #FFFFFF; } QPushButton::pressed { background-color: #4A4A4A; }")
        self.bottom_button1.setStyleSheet("QPushButton { background-color: #5A5A5A; border: none; border-radius: 5px; font-size: 16px; color: #FFFFFF; } QPushButton::pressed { background-color: #4A4A4A; }")
        self.top_button.setStyleSheet("QPushButton { background-color: #5A5A5A; border: none; border-radius: 5px; font-size: 16px; color: #FFFFFF; } QPushButton::pressed { background-color: #4A4A4A; }")
        self.bottom_button2.setStyleSheet("QPushButton { background-color: #006400; border: none; border-radius: 5px; font-size: 16px; color: #FFFFFF; } QPushButton::pressed { background-color: #4A4A4A; }")
        self.top_button.setEnabled(True)

    def retract_button_clicked(self):
        self.top_button.setEnabled(False)
        self.top_button.setStyleSheet("QPushButton { background-color: #5A5A5A; border: none; border-radius: 5px; font-size: 16px; color: #FFFFFF; }")
        self.top_button.setText("Auto Adjust")
        self.top_button_state = False
        self.bottom_button1.setStyleSheet("QPushButton { background-color: #8B0000; border: none; border-radius: 5px; font-size: 16px; color: #FFFFFF; } QPushButton::pressed { background-color: #4A4A4A; }")
        self.bottom_button2.setStyleSheet("QPushButton { background-color: #5A5A5A; border: none; border-radius: 5px; font-size: 16px; color: #FFFFFF; } QPushButton::pressed { background-color: #4A4A4A; }")
        self.bottom_button1.setStyleSheet("QPushButton { background-color: #8B0000; border: none; border-radius: 5px; font-size: 16px; color: #FFFFFF; } QPushButton::pressed { background-color: #4A4A4A; }")

    def slider_value_changed(self, value):
        pass

# Application setup
app = QApplication(sys.argv)
window = MainWindow()
window.show()

sys.exit(app.exec())
