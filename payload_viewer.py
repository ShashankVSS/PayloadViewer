# payload_viewer.py

import sys
import cv2
import os
import base64  # Added for encoding images
from datetime import datetime
import numpy as np
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout, QMenuBar, QMenu,
    QPushButton, QLabel, QSlider, QStackedWidget, QFrame, QDialog, QFormLayout, 
    QLineEdit, QDialogButtonBox, QMessageBox, QComboBox, QGridLayout, QScrollArea
)
from PyQt6.QtCore import Qt, QTimer, QThread, pyqtSignal, QObject, pyqtSlot
from PyQt6.QtGui import QImage, QPixmap, QAction
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtCore import QUrl

from PyQt6.QtWebChannel import QWebChannel  # *** Changed ***
from PyQt6.QtCore import QObject, pyqtSlot  # *** Changed ***

from PIL import Image
from PIL.ExifTags import TAGS, GPSTAGS

import folium  # Added for map generation
from io import BytesIO  # To handle HTML in memory

from serialcoms import SerialComs

from pymavlink import mavutil  # *** Added for MAVLink communication ***


# *** Removed FullScreenImageViewer Class ***
# The FullScreenImageViewer class is no longer needed and has been removed.
# *** End of Removed Section ***


# *** Added JSBridge Class ***
class JSBridge(QObject):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.main_window = parent

    @pyqtSlot(str)
    def select_image_in_gallery(self, image_path):  # *** Changed ***
        """Slot to select the image in gallery view."""
        self.main_window.select_image_in_gallery(image_path)
# *** End of Added Section ***


class ImageLoaderThread(QThread):
    images_loaded = pyqtSignal(list)  # Signal to send loaded images back to the UI

    def __init__(self, images_dir):
        super().__init__()
        self.images_dir = images_dir

    def run(self):
        image_files = [
            f for f in os.listdir(self.images_dir) if f.lower().endswith(('.png', '.jpg', '.jpeg'))
        ]
        # Emit the image file paths back to the main thread
        self.images_loaded.emit(image_files)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # Initialize the current frame storage
        self.current_frame = None  # Added to store the latest webcam frame
        
        # Initialize image label mapping and selection tracking
        self.image_labels = {}  # *** Changed ***
        self.selected_image_label = None  # *** Changed ***
        
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
        self.view_stack.addWidget(self.map_view)

        # Images view
        self.images_view_container = QWidget()
        self.images_view_layout = QVBoxLayout(self.images_view_container)
        self.images_view_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        
        self.gallery_grid = QGridLayout()
        self.gallery_grid.setSpacing(10)

        # Scroll Area for the gallery
        self.gallery_scroll_area = QScrollArea()
        self.gallery_scroll_area.setWidgetResizable(True)
        self.gallery_content = QWidget()
        self.gallery_content.setLayout(self.gallery_grid)
        self.gallery_scroll_area.setWidget(self.gallery_content)
        self.gallery_scroll_area.setStyleSheet("""
            QScrollBar:vertical {
                background: #2E2E2E;
                width: 12px;
                margin: 0px;
                border-radius: 6px;
            }
            QScrollBar::handle:vertical {
                background: #FFFFFF;  /* White color for the scrollbar handle */
                min-height: 20px;
                border-radius: 6px;  /* Rounded corners */
            }
            QScrollBar::handle:vertical:hover {
                background: #CCCCCC;  /* Light gray on hover */
            }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
                border: none;
                background: none;
            }
            QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {
                background: transparent;
            }
        """)

        # Add the gallery to the layout
        self.images_view_layout.addWidget(self.gallery_scroll_area)
        self.view_stack.addWidget(self.images_view_container)

        # Load images from /images directory
        self.load_gallery_images()

        # Timer for periodic folder checking
        self.image_check_timer = QTimer()
        self.image_check_timer.timeout.connect(self.load_gallery_images)

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
        top_section_widget.setStyleSheet("border-radius: 10px; padding: 10px;")

        # Initialize SerialComs without default port and baudrate
        self.serial_coms = SerialComs()
        self.serial_coms.connected.connect(self.on_connected)
        self.serial_coms.disconnected.connect(self.on_disconnected)
        self.serial_coms.error.connect(self.on_error)
        self.serial_coms.data_received.connect(self.on_data_received)
        self.serial_coms.latency_measured.connect(self.update_latency_display)
        self.serial_coms.processed_ranges_avg.connect(self.update_average_range)
        self.serial_coms.encoder_position_received.connect(self.update_encoder_value)
        self.serial_coms.extension_state_received.connect(self.update_extension_state)
        # Removed incorrect signal assignment
        self.serial_coms.detection_received.connect(self.on_detection_received)  # *** Connect detection signal ***

        # Define connection state
        self.connected = False

        # Auto Adjust button
        self.top_button = QPushButton("Auto Adjust")
        self.top_button.setFixedHeight(50)
        self.top_button.setStyleSheet("""
            QPushButton { 
                background-color: #5A5A5A; 
                border: none; 
                border-radius: 5px; 
                font-size: 16px; 
                color: #FFFFFF; 
            } 
            QPushButton::pressed { 
                background-color: #4A4A4A; 
            }
        """)
        self.top_button.setEnabled(False)
        self.top_button_state = False
        self.top_button.clicked.connect(self.toggle_auto_adjust)
        top_section_layout.addWidget(self.top_button)

        # Feature status label
        self.top_label = QLabel("Feature Status: Inactive")
        self.top_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.top_label.setStyleSheet("font-size: 14px;")
        top_section_layout.addWidget(self.top_label)

        # New Data Display Labels
        self.average_range_label = QLabel("Average Processed Range: N/A")
        self.average_range_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.average_range_label.setStyleSheet("font-size: 14px;")
        top_section_layout.addWidget(self.average_range_label)

        self.encoder_value_label = QLabel("Encoder Value: N/A")
        self.encoder_value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.encoder_value_label.setStyleSheet("font-size: 14px;")
        top_section_layout.addWidget(self.encoder_value_label)

        self.extension_state_label = QLabel("Current Extension State: N/A")
        self.extension_state_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.extension_state_label.setStyleSheet("font-size: 14px;")
        top_section_layout.addWidget(self.extension_state_label)

        # *** Added "Metal Detected" label ***
        self.metal_detected_label = QLabel("")  # Initialize with empty text
        self.metal_detected_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.metal_detected_label.setStyleSheet("font-size: 14px; color: red;")  # Red color for visibility
        self.metal_detected_label.hide()  # Hide initially
        top_section_layout.addWidget(self.metal_detected_label)  # Add to the layout
        # *** End of Added Section ***

        right_layout.addWidget(top_section_widget, 1)

        # Bottom section of the right column
        bottom_section_widget = QFrame()
        bottom_section_layout = QVBoxLayout(bottom_section_widget)
        bottom_section_widget.setStyleSheet("border-radius: 10px; padding: 10px;")

        # Retract button
        self.bottom_button1 = QPushButton("Retract")
        self.bottom_button1.setFixedHeight(50)
        self.bottom_button1.setStyleSheet("""
            QPushButton { 
                background-color: #5A5A5A; 
                border: none; 
                border-radius: 5px; 
                font-size: 16px; 
                color: #FFFFFF; 
            } 
            QPushButton::hover { 
                background-color: #6A6A6A; 
            } 
            QPushButton::pressed { 
                background-color: #4A4A4A; 
            }
        """)
        self.bottom_button1.clicked.connect(self.retract_button_clicked)
        bottom_section_layout.addWidget(self.bottom_button1)

        # Extend button
        self.bottom_button2 = QPushButton("Extend")
        self.bottom_button2.setFixedHeight(50)
        self.bottom_button2.setStyleSheet("""
            QPushButton { 
                background-color: #5A5A5A; 
                border: none; 
                border-radius: 5px; 
                font-size: 16px; 
                color: #FFFFFF; 
            } 
            QPushButton::hover { 
                background-color: #6A6A6A; 
            } 
            QPushButton::pressed { 
                background-color: #4A4A4A; 
            }
        """)
        self.bottom_button2.clicked.connect(self.extend_button_clicked)
        bottom_section_layout.addWidget(self.bottom_button2)

        # Slider
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.slider.setTickInterval(1)
        self.slider.setMaximum(10)
        self.slider.setMinimum(0)
        self.slider.setFixedHeight(50)
        self.slider.setStyleSheet("border-radius: 5px;")
        self.slider.valueChanged.connect(self.slider_value_changed)
        bottom_section_layout.addWidget(self.slider)

        # Stop button
        self.stop_button = QPushButton("Stop")
        self.stop_button.setFixedHeight(50)
        self.stop_button.setStyleSheet("""
            QPushButton { 
                background-color: #5A5A5A; 
                border: none; 
                border-radius: 5px; 
                font-size: 16px; 
                color: #FFFFFF; 
            } 
            QPushButton::hover { 
                background-color: #6A6A6A; 
            } 
            QPushButton::pressed { 
                background-color: #4A4A4A; 
            }
        """)
        self.stop_button.clicked.connect(self.stop_all_buttons)
        bottom_section_layout.addWidget(self.stop_button)

        # Connection indicator in the bottom section
        self.connection_indicator_top = QLabel()
        self.update_connection_indicator()
        self.connection_indicator_top.setAlignment(Qt.AlignmentFlag.AlignCenter)
        bottom_section_layout.addWidget(self.connection_indicator_top)

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

        # Add connection settings option to Settings menu
        connection_settings_action = QAction("Connection Settings", self)
        connection_settings_action.triggered.connect(self.open_connection_settings_dialog)
        settings_menu.addAction(connection_settings_action)

        # Add MAVLink settings option to Settings menu
        mavlink_settings_action = QAction("MAVLink Settings", self)
        mavlink_settings_action.triggered.connect(self.open_mavlink_settings_dialog)
        settings_menu.addAction(mavlink_settings_action)

        # Webcam setup
        self.cap = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.start_webcam()

        # Initialize QWebChannel after setting up the map_view
        self.channel = QWebChannel()
        self.bridge = JSBridge(self)  # *** Changed ***
        self.channel.registerObject("bridge", self.bridge)  # *** Changed ***
        self.map_view.page().setWebChannel(self.channel)  # *** Changed ***

        # Initially disable buttons
        self.update_buttons_state()

        # Initialize MAVLink connection variables
        self.mav = None  # MAVLink connection
        self.mav_connected = False  # MAVLink connection state

    def change_view(self, index):
        """Switch between different views in the stacked widget."""
        self.view_stack.setCurrentIndex(index)
        view_names = ["Webcam", "Map", "Images"]
        self.current_view_label.setText(f"Current View: {view_names[index]}")

        if index == 0:  # Webcam view
            self.start_webcam()
            self.image_check_timer.stop()
        elif index == 1:  # Map view
            self.update_map_markers()  # Update map markers when switching to Map view
            self.stop_webcam()
            self.image_check_timer.stop()
        elif index == 2:  # Images View
            self.load_gallery_images()  # Reload gallery on entering Images View
            self.image_check_timer.start(30000)  # Check for new images every 30 seconds
            self.stop_webcam()
        else:  # Other views
            self.stop_webcam()
            self.image_check_timer.stop()

    def start_webcam(self):
        """Start the webcam feed."""
        if self.cap is None:
            self.cap = cv2.VideoCapture(0)  # Adjust camera index as needed
        if not self.cap.isOpened():
            QMessageBox.critical(self, "Webcam Error", "Failed to open webcam.")
            return
        if not self.timer.isActive():
            self.timer.start(30)

    def stop_webcam(self):
        """Stop the webcam feed."""
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
        self.cap = None
        self.timer.stop()
        self.webcam_view.clear()

    def update_frame(self):
        """Update the webcam frame in the UI."""
        if self.cap is not None and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                self.current_frame = frame.copy()  # Store the original frame for saving
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                image = QImage(frame, frame.shape[1], frame.shape[0], QImage.Format.Format_RGB888)
                pixmap = QPixmap.fromImage(image).scaled(1280, 720, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)
                self.webcam_view.setPixmap(pixmap)

    def get_exif_data(self, image_path):
        """Extract EXIF data from an image."""
        try:
            image = Image.open(image_path)
            exif_data = image._getexif() or {}
            parsed_exif = {}
            
            # Map EXIF data to readable tags
            for tag, value in exif_data.items():
                tag_name = TAGS.get(tag, tag)
                parsed_exif[tag_name] = value

            # Parse GPS data if available
            gps_data = parsed_exif.get("GPSInfo")
            if gps_data:
                gps_info = {}
                for key, val in gps_data.items():
                    decoded = GPSTAGS.get(key, key)
                    gps_info[decoded] = val
                parsed_exif["GPSInfo"] = gps_info

            return parsed_exif
        except Exception as e:
            print(f"Error reading EXIF data: {e}")
            return {}

    def parse_exif_data(self, exif_data):
        """Extract relevant EXIF fields (date, time, and GPS coordinates)."""
        # Get date/time
        date_time = exif_data.get("DateTime", "Not Found")

        # Extract GPS coordinates if available
        gps_info = exif_data.get("GPSInfo", {})
        if gps_info:
            def convert_to_degrees(value):
                """Convert GPS coordinate to degrees."""
                d, m, s = value
                return d + (m / 60.0) + (s / 3600.0)

            latitude = gps_info.get("GPSLatitude")
            latitude_ref = gps_info.get("GPSLatitudeRef", "")
            longitude = gps_info.get("GPSLongitude")
            longitude_ref = gps_info.get("GPSLongitudeRef", "")

            if latitude and longitude and latitude_ref and longitude_ref:
                lat = convert_to_degrees(latitude)
                lon = convert_to_degrees(longitude)
                if latitude_ref != "N":
                    lat = -lat
                if longitude_ref != "E":
                    lon = -lon
                gps_coordinates = f"{lat:.6f}, {lon:.6f}"
            else:
                gps_coordinates = "Not Found"
        else:
            gps_coordinates = "Not Found"

        return date_time, gps_coordinates


    def load_gallery_images(self):
        """Load images from the /images directory using a separate thread."""
        images_dir = "images"  # Directory containing images
        if not os.path.exists(images_dir):
            os.makedirs(images_dir)  # Create the directory if it doesn't exist

        # Stop any running thread before starting a new one
        if hasattr(self, "image_loader_thread") and self.image_loader_thread.isRunning():
            self.image_loader_thread.terminate()

        # Start a new thread to load images
        self.image_loader_thread = ImageLoaderThread(images_dir)
        self.image_loader_thread.images_loaded.connect(self.update_gallery)  # Connect to update UI
        self.image_loader_thread.start()

    def update_gallery(self, image_files):
        """Update the gallery UI with the loaded images."""
        # Clear the current grid to avoid duplicates
        while self.gallery_grid.count():
            widget = self.gallery_grid.takeAt(0).widget()
            if widget:
                widget.deleteLater()

        # Reset image_labels mapping
        self.image_labels.clear()  # *** Changed ***
        self.selected_image_label = None  # *** Changed ***

        if not image_files:
            no_images_label = QLabel("No images found")
            no_images_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            no_images_label.setStyleSheet("font-size: 16px; color: #FFFFFF;")
            self.gallery_grid.addWidget(no_images_label, 0, 0, 1, 5)
            return

        # Add images in a fixed 5x5 grid layout
        rows = max(len(image_files) // 5 + (1 if len(image_files) % 5 else 0), 5)
        for row in range(rows):
            for col in range(5):
                index = row * 5 + col
                if index < len(image_files):
                    image_path = os.path.join("images", image_files[index])
                    thumbnail = QLabel()
                    pixmap = QPixmap(image_path).scaled(150, 150, Qt.AspectRatioMode.KeepAspectRatio,
                                                        Qt.TransformationMode.SmoothTransformation)
                    thumbnail.setPixmap(pixmap)
                    thumbnail.setAlignment(Qt.AlignmentFlag.AlignCenter)
                    thumbnail.setStyleSheet("border: 1px solid #5A5A5A; border-radius: 5px;")
                    thumbnail.mousePressEvent = lambda e, p=image_path: self.show_image_details(p)
                    self.gallery_grid.addWidget(thumbnail, row, col)
                    
                    # Map image path to thumbnail QLabel
                    self.image_labels[image_path] = thumbnail  # *** Changed ***
                else:
                    # Add an empty placeholder to maintain the grid structure
                    placeholder = QLabel()
                    placeholder.setFixedSize(150, 150)
                    self.gallery_grid.addWidget(placeholder, row, col)

        # After updating the gallery, update the map markers
        self.update_map_markers()

    def show_image_details(self, image_path):
        """Show details of a clicked image including date, time, and GPS from EXIF data."""
        # Extract EXIF data
        exif_data = self.get_exif_data(image_path)
        date_time, gps_coordinates = self.parse_exif_data(exif_data)

        # Display dialog
        dialog = QDialog(self)
        dialog.setWindowTitle("Image Details")
        dialog.setStyleSheet("background-color: #3A3A3A; color: #FFFFFF; border-radius: 10px; padding: 10px;")

        layout = QHBoxLayout(dialog)

        # Left: Display image (3/4 of the width)
        image_layout = QVBoxLayout()
        self.image_label = QLabel()
        pixmap = QPixmap(image_path).scaled(800, 600, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)
        self.image_label.setPixmap(pixmap)
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        image_layout.addWidget(self.image_label)
        layout.addLayout(image_layout, 3)

        # Right: Details (1/4 of the width)
        details_layout = QVBoxLayout()

        # Date and time of capture
        capture_time_label = QLabel(f"Capture Time: {date_time}")
        capture_time_label.setStyleSheet("font-size: 14px; color: #FFFFFF;")
        details_layout.addWidget(capture_time_label)

        # GPS coordinates
        gps_label = QLabel(f"GPS Coordinates: {gps_coordinates}")
        gps_label.setStyleSheet("font-size: 14px; color: #FFFFFF;")
        details_layout.addWidget(gps_label)

        # Mini map view (cleaner OpenStreetMap view)
        if gps_coordinates != "Not Found":
            lat, lon = map(float, gps_coordinates.split(", "))
            map_url = f"https://www.openstreetmap.org/export/embed.html?bbox={lon - 0.005}," \
                    f"{lat - 0.005},{lon + 0.005},{lat + 0.005}&layer=map&marker={lat},{lon}"
        else:
            map_url = "https://www.openstreetmap.org"
        map_view = QWebEngineView()
        map_view.setUrl(QUrl(map_url))
        map_view.setFixedHeight(300)
        map_view.setFixedWidth(300)
        details_layout.addWidget(map_view)

        # *** Changed: Replace "View Full Screen" with "Select in Gallery" ***
        # Add "Select in Gallery" button
        select_gallery_button = QPushButton("Select in Gallery")
        select_gallery_button.setFixedHeight(40)
        select_gallery_button.setStyleSheet("""
            QPushButton { 
                background-color: #5A5A5A; 
                border: none; 
                border-radius: 5px; 
                font-size: 14px; 
                color: #FFFFFF; 
            }
            QPushButton::hover { 
                background-color: #6A6A6A; 
            }
            QPushButton::pressed { 
                background-color: #4A4A4A; 
            }
        """)
        select_gallery_button.clicked.connect(lambda: self.select_image_in_gallery(image_path))  # *** Changed ***
        details_layout.addWidget(select_gallery_button)
        # *** End of Changed Section ***

        # Add Close button
        close_button = QPushButton("Close")
        close_button.setStyleSheet("""
            QPushButton {
                background-color: #5A5A5A;
                border: none;
                border-radius: 5px;
                font-size: 16px;
                color: #FFFFFF;
            }
            QPushButton::hover {
                background-color: #6A6A6A;
            }
            QPushButton::pressed {
                background-color: #4A4A4A;
            }
        """)
        close_button.clicked.connect(dialog.accept)
        details_layout.addWidget(close_button)

        layout.addLayout(details_layout, 1)
        dialog.setLayout(layout)
        dialog.exec()

    # *** Removed view_full_screen Method ***
    # The view_full_screen method is no longer needed and has been removed.
    # *** End of Removed Section ***

    # *** Added select_image_in_gallery Method ***
    def select_image_in_gallery(self, image_path):
        """Select and highlight the image in the gallery view."""
        if image_path in self.image_labels:
            # Reset previous selection
            if self.selected_image_label:
                self.selected_image_label.setStyleSheet("border: 1px solid #5A5A5A; border-radius: 5px;")

            # Highlight the selected image
            selected_label = self.image_labels[image_path]
            selected_label.setStyleSheet("border: 3px solid #FFD700; border-radius: 5px;")  # Gold border
            self.selected_image_label = selected_label

            # Switch to Images view
            self.view_stack.setCurrentIndex(2)

            # Scroll to the selected image
            self.gallery_scroll_area.ensureWidgetVisible(selected_label)
        else:
            QMessageBox.warning(self, "Image Not Found", f"The image {image_path} is not in the gallery.")
    # *** End of Added Section ***

    def open_slider_settings_dialog(self):
        """Open a dialog to adjust slider settings."""
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
        button_box.setStyleSheet("""
            QPushButton { 
                background-color: #5A5A5A; 
                border: none; 
                border-radius: 5px; 
                color: #FFFFFF; 
            } 
            QPushButton::pressed { 
                background-color: #4A4A4A; 
            }
        """)
        layout.addRow(button_box)

        dialog.exec()

    def open_connection_settings_dialog(self):
        """Open a dialog to manage serial connection settings."""
        self.connection_settings_dialog = QDialog(self)
        dialog = self.connection_settings_dialog  # Alias for convenience
        dialog.setWindowTitle("Connection Settings")
        dialog.setStyleSheet("background-color: #3A3A3A; color: #FFFFFF; border-radius: 10px; padding: 10px;")

        layout = QVBoxLayout(dialog)

        # Serial port selection
        port_layout = QHBoxLayout()
        port_label = QLabel("Serial Port:")
        port_label.setStyleSheet("font-size: 14px;")
        self.port_combo_dialog = QComboBox()
        self.port_combo_dialog.setStyleSheet("background-color: #5A5A5A; border: none; border-radius: 5px; color: #FFFFFF;")
        available_ports = self.serial_coms.list_available_ports()
        if available_ports:
            self.port_combo_dialog.addItems(available_ports)
        else:
            self.port_combo_dialog.addItem("No Ports Found")
        port_layout.addWidget(port_label)
        port_layout.addWidget(self.port_combo_dialog)
        layout.addLayout(port_layout)

        # Baudrate selection
        baud_layout = QHBoxLayout()
        baud_label = QLabel("Baudrate:")
        baud_label.setStyleSheet("font-size: 14px;")
        self.baud_combo_dialog = QComboBox()
        self.baud_combo_dialog.setStyleSheet("background-color: #5A5A5A; border: none; border-radius: 5px; color: #FFFFFF;")
        baudrates = [57600, 115200]
        self.baud_combo_dialog.addItems([str(b) for b in baudrates])
        self.baud_combo_dialog.setCurrentText(str(self.serial_coms.baudrate) if self.serial_coms.baudrate else "57600")
        baud_layout.addWidget(baud_label)
        baud_layout.addWidget(self.baud_combo_dialog)
        layout.addLayout(baud_layout)

        # Connection indicator
        self.connection_indicator_dialog = QLabel()
        self.update_connection_indicator_dialog()
        self.connection_indicator_dialog.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.connection_indicator_dialog)

        # Latency display
        latency_layout = QHBoxLayout()
        latency_label = QLabel("Ping-Pong Latency:")
        latency_label.setStyleSheet("font-size: 14px;")
        self.latency_display_label = QLabel("N/A")  # Initial value
        self.latency_display_label.setStyleSheet("font-size: 14px; color: #FFFFFF;")
        latency_layout.addWidget(latency_label)
        latency_layout.addWidget(self.latency_display_label)
        layout.addLayout(latency_layout)

        # Connect/Disconnect button
        self.connection_button_dialog = QPushButton("Connect" if not self.connected else "Disconnect")
        self.connection_button_dialog.setFixedHeight(50)
        self.connection_button_dialog.setStyleSheet("""
            QPushButton { 
                background-color: #5A5A5A; 
                border: none; 
                border-radius: 5px; 
                font-size: 16px; 
                color: #FFFFFF; 
            } 
            QPushButton::pressed { 
                background-color: #4A4A4A; 
            }
        """)
        self.connection_button_dialog.clicked.connect(self.toggle_connection_dialog)
        layout.addWidget(self.connection_button_dialog)

        dialog.setLayout(layout)
        dialog.exec()

    def open_mavlink_settings_dialog(self):
        """Open a dialog to manage MAVLink connection settings."""
        self.mavlink_settings_dialog = QDialog(self)
        dialog = self.mavlink_settings_dialog  # Alias for convenience
        dialog.setWindowTitle("MAVLink Settings")
        dialog.setStyleSheet("background-color: #3A3A3A; color: #FFFFFF; border-radius: 10px; padding: 10px;")

        layout = QVBoxLayout(dialog)

        # MAVLink IP input
        mav_ip_layout = QHBoxLayout()
        mav_ip_label = QLabel("MAVLink IP:")
        mav_ip_label.setStyleSheet("font-size: 14px;")
        self.mav_ip_input_dialog = QLineEdit("udp:127.0.0.1:14550")  # Default MAVLink IP
        self.mav_ip_input_dialog.setStyleSheet("background-color: #5A5A5A; border: none; border-radius: 5px; color: #FFFFFF;")
        mav_ip_layout.addWidget(mav_ip_label)
        mav_ip_layout.addWidget(self.mav_ip_input_dialog)
        layout.addLayout(mav_ip_layout)

        # Connection indicator
        self.mavlink_connection_indicator_dialog = QLabel()
        self.update_mavlink_connection_indicator_dialog()
        self.mavlink_connection_indicator_dialog.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.mavlink_connection_indicator_dialog)

        # Connect/Disconnect button
        self.mavlink_connection_button_dialog = QPushButton("Connect" if not self.mav_connected else "Disconnect")
        self.mavlink_connection_button_dialog.setFixedHeight(50)
        self.mavlink_connection_button_dialog.setStyleSheet("""
            QPushButton { 
                background-color: #5A5A5A; 
                border: none; 
                border-radius: 5px; 
                font-size: 16px; 
                color: #FFFFFF; 
            } 
            QPushButton::pressed { 
                background-color: #4A4A4A; 
            }
        """)
        self.mavlink_connection_button_dialog.clicked.connect(self.toggle_mavlink_connection_dialog)
        layout.addWidget(self.mavlink_connection_button_dialog)

        dialog.setLayout(layout)
        dialog.exec()

    def toggle_connection_dialog(self):
        """Connect or disconnect based on current state from the serial connection dialog."""
        if not self.connected:
            # Get selected port and baudrate
            selected_port = self.port_combo_dialog.currentText()
            if selected_port == "No Ports Found":
                QMessageBox.critical(self, "Connection Error", "No serial ports available.")
                return
            selected_baudrate = int(self.baud_combo_dialog.currentText())
            self.serial_coms.port = selected_port
            self.serial_coms.baudrate = selected_baudrate
            self.serial_coms.connect_serial()
            self.connection_button_dialog.setEnabled(False)  # Disable to prevent multiple clicks
        else:
            self.serial_coms.disconnect_serial()

    def toggle_mavlink_connection_dialog(self):
        """Connect or disconnect based on current state from the MAVLink settings dialog."""
        if not self.mav_connected:
            # Get MAVLink IP
            mav_ip = self.mav_ip_input_dialog.text().strip()
            if not mav_ip:
                QMessageBox.warning(self, "Input Error", "Please enter a valid MAVLink IP.")
                return
            self.connect_mavlink(mav_ip)
            self.mavlink_connection_button_dialog.setEnabled(False)  # Disable to prevent multiple clicks
        else:
            self.disconnect_mavlink()

    def connect_mavlink(self, mav_ip):
        """Establish a MAVLink connection."""
        try:
            self.mav = mavutil.mavlink_connection(mav_ip)
            self.mav.wait_heartbeat(timeout=5)
            self.mav_connected = True
            self.update_mavlink_connection_indicator_dialog()
            self.mavlink_connection_button_dialog.setText("Disconnect")
            QMessageBox.information(self, "MAVLink Connected", f"Connected to MAVLink on {mav_ip}")
        except Exception as e:
            QMessageBox.warning(self, "MAVLink Connection Error", f"Failed to connect to MAVLink: {e}")
            self.mav_connected = False
            self.update_mavlink_connection_indicator_dialog()
            self.mavlink_connection_button_dialog.setEnabled(True)

    def disconnect_mavlink(self):
        """Disconnect the MAVLink connection."""
        if self.mav:
            self.mav.close()
            self.mav = None
        self.mav_connected = False
        self.update_mavlink_connection_indicator_dialog()
        self.mavlink_connection_button_dialog.setText("Connect")
        QMessageBox.information(self, "MAVLink Disconnected", "Disconnected from MAVLink.")
        self.mavlink_connection_button_dialog.setEnabled(True)

    def update_mavlink_connection_indicator_dialog(self):
        """Update the visual indicator of the MAVLink connection status in the dialog."""
        if self.mav_connected:
            self.mavlink_connection_indicator_dialog.setText("MAVLink Connection: <span style='color: green;'>⬤</span>")
        else:
            self.mavlink_connection_indicator_dialog.setText("MAVLink Connection: <span style='color: red;'>⬤</span>")

    def update_connection_indicator_dialog(self):
        """Update the visual indicator of the serial connection status in the dialog."""
        if self.connected:
            self.connection_indicator_dialog.setText("Serial Connection: <span style='color: green;'>⬤</span>")
        else:
            self.connection_indicator_dialog.setText("Serial Connection: <span style='color: red;'>⬤</span>")

    def update_connection_indicator(self):
        """Update the visual indicator of the serial connection status."""
        if self.connected:
            self.connection_indicator_top.setText("Connection:   <span style='color: green;'>⬤</span>")
        else:
            self.connection_indicator_top.setText("Connection:   <span style='color: red;'>⬤</span>")

    def update_latency_display(self, latency):
        """Update the latency display in the connection settings."""
        if hasattr(self, 'connection_settings_dialog') and self.connection_settings_dialog.isVisible():
            if hasattr(self, 'latency_display_label'):
                self.latency_display_label.setText(f"{latency:.2f} ms")

    def on_connected(self):
        """Handle actions when a serial connection is established."""
        self.connected = True
        self.update_connection_indicator()
        self.update_connection_indicator_dialog()
        if hasattr(self, 'connection_button_dialog') and self.connection_button_dialog:
            try:
                self.connection_button_dialog.setText("Disconnect")
                self.connection_button_dialog.setEnabled(True)
            except RuntimeError:
                pass  # The button has been deleted
        self.update_buttons_state()
        QMessageBox.information(self, "Connected", f"Connected to {self.serial_coms.port} at {self.serial_coms.baudrate} baud.")

    def on_disconnected(self):
        """Handle actions when the serial connection is lost or closed."""
        self.connected = False
        self.update_connection_indicator()
        self.update_connection_indicator_dialog()
        if hasattr(self, 'connection_button_dialog') and self.connection_button_dialog:
            try:
                self.connection_button_dialog.setText("Connect")
                self.connection_button_dialog.setEnabled(True)
            except RuntimeError:
                pass  # The button has been deleted
        self.update_buttons_state()
        QMessageBox.warning(self, "Disconnected", "Serial connection has been disconnected.")
        # Reset Auto Adjust to default state
        self.reset_auto_adjust()

    def on_error(self, message):
        """Handle errors emitted from the SerialComs module."""
        self.update_connection_indicator()
        QMessageBox.critical(self, "Serial Error", message)
        self.connected = False
        self.update_connection_indicator_dialog()
        if hasattr(self, 'connection_button_dialog') and self.connection_button_dialog:
            try:
                self.connection_button_dialog.setText("Connect")
                self.connection_button_dialog.setEnabled(True)
            except RuntimeError:
                pass  # The button has been deleted
        self.update_buttons_state()
        # Reset Auto Adjust to default state
        self.reset_auto_adjust()

    def on_data_received(self, data):
        """Handle data received from the serial connection."""
        # Handle received data if needed
        print(f"Data received: {data}")

    def on_detection_received(self, detection_info):
        """Handle detection events received from serial communication."""
        self.status_label.setText(f"Status: Detection - {detection_info}")
        self.save_image_with_gps(detection_info)

    def update_average_range(self, average):
        """Update the average processed range display."""
        self.average_range_label.setText(f"Average Processed Range: {average}")

    def update_encoder_value(self, encoder_value):
        """Update the encoder value display."""
        self.encoder_value_label.setText(f"Encoder Value: {encoder_value}")

    def update_extension_state(self, extension_state):
        """Update the current extension state display."""
        self.extension_state_label.setText(f"Current Extension State: {extension_state}")

    def update_slider_settings(self, min_input, max_input, step_input, dialog):
        """Update slider settings based on user input."""
        try:
            min_value = int(min_input.text())
            max_value = int(max_input.text())
            step_value = int(step_input.text())

            self.slider.setMinimum(min_value)
            self.slider.setMaximum(max_value)
            self.slider.setTickInterval(step_value)

            dialog.accept()
        except ValueError:
            QMessageBox.warning(self, "Invalid Input", "Please enter valid integer values.")

    # *** Added Map Marker Functionality ***
    def update_map_markers(self):
        """Collect GPS coordinates from images and update the map view with markers."""
        images_dir = "images"
        image_files = [
            f for f in os.listdir(images_dir) if f.lower().endswith(('.png', '.jpg', '.jpeg'))
        ]
        coordinates = []
        for image_file in image_files:
            image_path = os.path.join(images_dir, image_file)
            exif_data = self.get_exif_data(image_path)
            _, gps_coordinates = self.parse_exif_data(exif_data)
            if gps_coordinates != "Not Found":
                try:
                    lat, lon = map(float, gps_coordinates.split(", "))
                    coordinates.append((lat, lon, image_file))  # Include image name for marker popup
                except ValueError:
                    print(f"Invalid GPS coordinates in {image_file}: {gps_coordinates}")

        if not coordinates:
            # If no coordinates found, display a default map
            folium_map = folium.Map(location=[0, 0], zoom_start=2)
        else:
            # Calculate the average latitude and longitude for centering the map
            avg_lat = sum([coord[0] for coord in coordinates]) / len(coordinates)
            avg_lon = sum([coord[1] for coord in coordinates]) / len(coordinates)
            folium_map = folium.Map(location=[avg_lat, avg_lon], zoom_start=10)

            # Add markers to the map
            for lat, lon, image_name in coordinates:
                image_path = os.path.join("images", image_name)
                try:
                    # Open and resize the image for preview
                    with Image.open(image_path) as img:
                        img.thumbnail((100, 100))  # Resize to 100x100 pixels
                        buffered = BytesIO()
                        img.save(buffered, format="PNG")
                        img_str = base64.b64encode(buffered.getvalue()).decode()
                        img_html = f"<img src='data:image/png;base64,{img_str}' width='100' height='100'>"
                except Exception as e:
                    print(f"Error processing image {image_name} for popup: {e}")
                    img_html = ""

                # *** Changed: Replace "View Full Screen" with "Select in Gallery" ***
                # HTML for the popup with the "Select in Gallery" button
                popup_html = f"""
                <b>{image_name}</b><br>
                Lat: {lat}<br>
                Lon: {lon}<br>
                {img_html}<br>
                <button onclick="selectInGallery('{image_path}')">Select in Gallery</button>
                """
                # *** End of Changed Section ***

                folium.Marker(
                    location=[lat, lon],
                    popup=folium.Popup(popup_html, max_width=300)
                ).add_to(folium_map)

        # Save the map to an HTML string
        data = BytesIO()
        folium_map.save(data, close_file=False)
        html = data.getvalue().decode()

        # Inject JavaScript for QWebChannel communication
        # *** Changed: Update the injected_js to call select_image_in_gallery ***
        # This script initializes the QWebChannel and defines the selectInGallery function
        injected_js = """
        <script src="qrc:///qtwebchannel/qwebchannel.js"></script>
        <script type="text/javascript">
            new QWebChannel(qt.webChannelTransport, function(channel) {
                window.bridge = channel.objects.bridge;
            });

            function selectInGallery(imagePath) {
                if (bridge) {
                    bridge.select_image_in_gallery(imagePath);
                }
            }
        </script>
        """
        # *** End of Changed Section ***

        # Insert the injected JavaScript before the closing </body> tag
        html = html.replace("</body>", injected_js + "</body>")

        # Load the modified HTML into the QWebEngineView
        self.map_view.setHtml(html, QUrl(""))

    # *** End of Added Section ***

    # *** Removed Method to View Full Screen ***
    # The view_full_screen method is no longer needed and has been removed.
    # *** End of Removed Section ***

    # *** Added Key Press Event Handling for Image Save Debugging ***
    # def keyPressEvent(self, event):
    #     """Override the key press event to detect backtick key."""
    #     # Optional: Uncomment the next line for debugging key presses
    #     # print(f"Key Pressed: '{event.text()}' (Key Code: {event.key()})")
        
    #     if event.text() == '`':  # Detect backtick key (`) press
    #         self.handle_metal_detected()
    #     else:
    #         super().keyPressEvent(event)

    def handle_metal_detected(self):
        """Handle the metal detection event triggered by backtick key."""
        # Display the "Metal Detected" message
        self.metal_detected_label.setText("Metal Detected")
        self.metal_detected_label.show()
        # Hide the message after 3 seconds
        QTimer.singleShot(3000, self.metal_detected_label.hide)

        # Save the current frame from the webcam
        if self.current_frame is not None:
            images_dir = "images"
            if not os.path.exists(images_dir):
                os.makedirs(images_dir)  # Create the directory if it doesn't exist

            # Generate a unique filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"metal_detected_{timestamp}.png"
            image_path = os.path.join(images_dir, filename)

            # Save the image using OpenCV
            try:
                cv2.imwrite(image_path, self.current_frame)
                # QMessageBox.information(self, "Image Saved", f"Image saved to {image_path}")
            except Exception as e:
                QMessageBox.critical(self, "Save Error", f"Failed to save image: {e}")
        else:
            QMessageBox.warning(self, "No Frame", "No frame available to save.")
    # *** End of Added Section ***

    # Button and slider functions
    def toggle_auto_adjust(self):
        """Toggle the Auto Adjust feature and send corresponding serial command."""
        if self.top_button_state:
            # Currently ON, so turn it OFF
            self.top_button.setStyleSheet("""
                QPushButton { 
                    background-color: #8B0000; 
                    border: none; 
                    border-radius: 5px; 
                    font-size: 16px; 
                    color: #FFFFFF; 
                } 
                QPushButton::pressed { 
                    background-color: #4A4A4A; 
                }
            """)
            self.top_button.setText("Auto Adjust: Off")
            self.serial_coms.send_data("AUTO_ADJUST_OFF")
            self.top_label.setText("Feature Status: Inactive")
        else:
            # Currently OFF, so turn it ON
            self.top_button.setStyleSheet("""
                QPushButton { 
                    background-color: #006400; 
                    border: none; 
                    border-radius: 5px; 
                    font-size: 16px; 
                    color: #FFFFFF; 
                } 
                QPushButton::pressed { 
                    background-color: #4A4A4A; 
                }
            """)
            self.top_button.setText("Auto Adjust: On")
            self.serial_coms.send_data("AUTO_ADJUST_ON")
            self.top_label.setText("Feature Status: Active")
        self.top_button_state = not self.top_button_state

    def extend_button_clicked(self):
        """Send the 'EXTEND' command over serial and update button styles."""
        if self.connected:
            self.serial_coms.send_data("Retract")
            # Update Extend button style to indicate active state
            self.bottom_button2.setStyleSheet("""
                QPushButton { 
                    background-color: #006400; 
                    border: none; 
                    border-radius: 5px; 
                    font-size: 16px; 
                    color: #FFFFFF; 
                } 
                QPushButton::hover { 
                    background-color: #007500; 
                } 
                QPushButton::pressed { 
                    background-color: #4A4A4A; 
                }
            """)
            # Reset Retract button style
            self.bottom_button1.setStyleSheet("""
                QPushButton { 
                    background-color: #5A5A5A; 
                    border: none; 
                    border-radius: 5px; 
                    font-size: 16px; 
                    color: #FFFFFF; 
                } 
                QPushButton::hover { 
                    background-color: #6A6A6A; 
                } 
                QPushButton::pressed { 
                    background-color: #4A4A4A; 
                }
            """)
            # Enable Auto Adjust button
            self.top_button.setEnabled(True)
        else:
            QMessageBox.warning(self, "Not Connected", "Cannot send command. Not connected to serial device.")

    def retract_button_clicked(self):
        """Send the 'RETRACT' command over serial and update button styles."""
        if self.connected:
            self.serial_coms.send_data("Extend")
            # Update Retract button style to indicate active state
            self.bottom_button1.setStyleSheet("""
                QPushButton { 
                    background-color: #8B0000; 
                    border: none; 
                    border-radius: 5px; 
                    font-size: 16px; 
                    color: #FFFFFF; 
                } 
                QPushButton::hover { 
                    background-color: #9C0000; 
                } 
                QPushButton::pressed { 
                    background-color: #4A4A4A; 
                }
            """)
            # Reset Extend button style
            self.bottom_button2.setStyleSheet("""
                QPushButton { 
                    background-color: #5A5A5A; 
                    border: none; 
                    border-radius: 5px; 
                    font-size: 16px; 
                    color: #FFFFFF; 
                } 
                QPushButton::hover { 
                    background-color: #6A6A6A; 
                } 
                QPushButton::pressed { 
                    background-color: #4A4A4A; 
                }
            """)
            # Reset Auto Adjust to default state
            self.reset_auto_adjust()
        else:
            QMessageBox.warning(self, "Not Connected", "Cannot send command. Not connected to serial device.")

    def slider_value_changed(self, value):
        """Send the slider value over serial when it changes."""
        if self.connected:
            self.serial_coms.send_data(f"SLIDER_VALUE:{value}")
        # You can update UI or other components as needed

    def stop_all_buttons(self):
        """Send the 'STOP' command and reset all buttons to their default state."""
        if self.connected:
            self.serial_coms.send_data("Stop")
            # Reset Auto Adjust button
            self.reset_auto_adjust()

            # Reset Retract and Extend button styles
            self.bottom_button1.setStyleSheet("""
                QPushButton { 
                    background-color: #5A5A5A; 
                    border: none; 
                    border-radius: 5px; 
                    font-size: 16px; 
                    color: #FFFFFF; 
                } 
                QPushButton::hover { 
                    background-color: #6A6A6A; 
                } 
                QPushButton::pressed { 
                    background-color: #4A4A4A; 
                }
            """)
            self.bottom_button2.setStyleSheet("""
                QPushButton { 
                    background-color: #5A5A5A; 
                    border: none; 
                    border-radius: 5px; 
                    font-size: 16px; 
                    color: #FFFFFF; 
                } 
                QPushButton::hover { 
                    background-color: #6A6A6A; 
                } 
                QPushButton::pressed { 
                    background-color: #4A4A4A; 
                }
            """)
        else:
            QMessageBox.warning(self, "Not Connected", "Cannot send command. Not connected to serial device.")

    def update_buttons_state(self):
        """Enable or disable buttons based on the connection status."""
        self.top_button.setEnabled(self.connected and self.bottom_button2.isEnabled())
        self.bottom_button1.setEnabled(self.connected)
        self.bottom_button2.setEnabled(self.connected)
        self.stop_button.setEnabled(self.connected)  # Assuming Stop should be enabled when connected

    def reset_auto_adjust(self):
        """Reset the Auto Adjust button to its default state."""
        self.top_button.setEnabled(False)
        self.top_button.setText("Auto Adjust")
        self.top_button_state = False
        self.top_label.setText("Feature Status: Inactive")
        self.top_button.setStyleSheet("""
            QPushButton { 
                background-color: #5A5A5A; 
                border: none; 
                border-radius: 5px; 
                font-size: 16px; 
                color: #FFFFFF; 
            }
            QPushButton::pressed { 
                background-color: #4A4A4A; 
            }
        """)

    def closeEvent(self, event):
        """Handle the window close event to ensure serial port is closed and threads are stopped."""
        if self.connected:
            self.serial_coms.disconnect_serial()
        if self.mav_connected:
            self.disconnect_mavlink()
        self.stop_webcam()
        event.accept()

    # *** Added Handle Detection and Save Image with GPS ***
    def save_image_with_gps(self, detection_info):
        """Capture image from webcam, embed GPS data, and save."""
        if self.current_frame is None:
            QMessageBox.warning(self, "No Frame", "No frame available to save.")
            return

        # Get current GPS data
        gps_data = self.get_current_gps()

        # Convert OpenCV image (BGR) to PIL image (RGB)
        frame_rgb = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2RGB)
        pil_image = Image.fromarray(frame_rgb)

        # Draw GPS data and detection info on the image
        draw = ImageDraw.Draw(pil_image)
        try:
            font = ImageFont.truetype("arial.ttf", 20)
        except IOError:
            font = ImageFont.load_default()

        text = f"{gps_data}\nDetection: {detection_info}"
        draw.text((10, 10), text, fill="red", font=font)

        # Define save directory
        save_dir = "captured_images"
        os.makedirs(save_dir, exist_ok=True)

        # Define filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{save_dir}/detection_{timestamp}.jpg"
        image_path = os.path.join(save_dir, filename)

        # Save image
        try:
            pil_image.save(filename)
            QMessageBox.information(self, "Image Saved", f"Image saved as {filename}")
        except Exception as e:
            QMessageBox.critical(self, "Save Error", f"Failed to save image: {e}")
    # *** End of Added Section ***

    # *** Added MAVLink Connection Handling ***
    def open_mavlink_settings_dialog(self):
        """Open a dialog to manage MAVLink connection settings."""
        self.mavlink_settings_dialog = QDialog(self)
        dialog = self.mavlink_settings_dialog  # Alias for convenience
        dialog.setWindowTitle("MAVLink Settings")
        dialog.setStyleSheet("background-color: #3A3A3A; color: #FFFFFF; border-radius: 10px; padding: 10px;")

        layout = QVBoxLayout(dialog)

        # MAVLink IP input
        mav_ip_layout = QHBoxLayout()
        mav_ip_label = QLabel("MAVLink IP:")
        mav_ip_label.setStyleSheet("font-size: 14px;")
        self.mav_ip_input_dialog = QLineEdit("udp:127.0.0.1:14550")  # Default MAVLink IP
        self.mav_ip_input_dialog.setStyleSheet("background-color: #5A5A5A; border: none; border-radius: 5px; color: #FFFFFF;")
        mav_ip_layout.addWidget(mav_ip_label)
        mav_ip_layout.addWidget(self.mav_ip_input_dialog)
        layout.addLayout(mav_ip_layout)

        # Connection indicator
        self.mavlink_connection_indicator_dialog = QLabel()
        self.update_mavlink_connection_indicator_dialog()
        self.mavlink_connection_indicator_dialog.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.mavlink_connection_indicator_dialog)

        # Connect/Disconnect button
        self.mavlink_connection_button_dialog = QPushButton("Connect" if not self.mav_connected else "Disconnect")
        self.mavlink_connection_button_dialog.setFixedHeight(50)
        self.mavlink_connection_button_dialog.setStyleSheet("""
            QPushButton { 
                background-color: #5A5A5A; 
                border: none; 
                border-radius: 5px; 
                font-size: 16px; 
                color: #FFFFFF; 
            } 
            QPushButton::pressed { 
                background-color: #4A4A4A; 
            }
        """)
        self.mavlink_connection_button_dialog.clicked.connect(self.toggle_mavlink_connection_dialog)
        layout.addWidget(self.mavlink_connection_button_dialog)

        dialog.setLayout(layout)
        dialog.exec()

    def toggle_mavlink_connection_dialog(self):
        """Connect or disconnect based on current state from the MAVLink settings dialog."""
        if not self.mav_connected:
            # Get MAVLink IP
            mav_ip = self.mav_ip_input_dialog.text().strip()
            if not mav_ip:
                QMessageBox.warning(self, "Input Error", "Please enter a valid MAVLink IP.")
                return
            self.connect_mavlink(mav_ip)
            self.mavlink_connection_button_dialog.setEnabled(False)  # Disable to prevent multiple clicks
        else:
            self.disconnect_mavlink()

    def connect_mavlink(self, mav_ip):
        """Establish a MAVLink connection."""
        try:
            self.mav = mavutil.mavlink_connection(mav_ip)
            self.mav.wait_heartbeat(timeout=5)
            self.mav_connected = True
            self.update_mavlink_connection_indicator_dialog()
            self.mavlink_connection_button_dialog.setText("Disconnect")
            QMessageBox.information(self, "MAVLink Connected", f"Connected to MAVLink on {mav_ip}")
        except Exception as e:
            QMessageBox.warning(self, "MAVLink Connection Error", f"Failed to connect to MAVLink: {e}")
            self.mav_connected = False
            self.update_mavlink_connection_indicator_dialog()
            self.mavlink_connection_button_dialog.setEnabled(True)

    def disconnect_mavlink(self):
        """Disconnect the MAVLink connection."""
        if self.mav:
            self.mav.close()
            self.mav = None
        self.mav_connected = False
        self.update_mavlink_connection_indicator_dialog()
        self.mavlink_connection_button_dialog.setText("Connect")
        QMessageBox.information(self, "MAVLink Disconnected", "Disconnected from MAVLink.")
        self.mavlink_connection_button_dialog.setEnabled(True)

    def update_mavlink_connection_indicator_dialog(self):
        """Update the visual indicator of the MAVLink connection status in the dialog."""
        if self.mav_connected:
            self.mavlink_connection_indicator_dialog.setText("MAVLink Connection: <span style='color: green;'>⬤</span>")
        else:
            self.mavlink_connection_indicator_dialog.setText("MAVLink Connection: <span style='color: red;'>⬤</span>")
    # *** End of Added Section ***

    # *** Added Handle Detection and Save Image with GPS ***
    def save_image_with_gps(self, detection_info):
        """Capture image from webcam, embed GPS data, and save."""
        if self.current_frame is None:
            QMessageBox.warning(self, "No Frame", "No frame available to save.")
            return

        # Get current GPS data
        gps_data = self.get_current_gps()

        # Convert OpenCV image (BGR) to PIL image (RGB)
        frame_rgb = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2RGB)
        pil_image = Image.fromarray(frame_rgb)

        # Draw GPS data and detection info on the image
        draw = ImageDraw.Draw(pil_image)
        try:
            font = ImageFont.truetype("arial.ttf", 20)
        except IOError:
            font = ImageFont.load_default()

        text = f"{gps_data}\nDetection: {detection_info}"
        draw.text((10, 10), text, fill="red", font=font)

        # Define save directory
        save_dir = "captured_images"
        os.makedirs(save_dir, exist_ok=True)

        # Define filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{save_dir}/detection_{timestamp}.jpg"
        image_path = os.path.join(save_dir, filename)

        # Save image
        try:
            pil_image.save(filename)
            QMessageBox.information(self, "Image Saved", f"Image saved as {filename}")
        except Exception as e:
            QMessageBox.critical(self, "Save Error", f"Failed to save image: {e}")
    # *** End of Added Section ***

    def get_current_gps(self):
        """Fetch current GPS coordinates from MAVLink."""
        if not self.mav_connected or not self.mav:
            return "GPS: Unknown"

        try:
            msg = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if msg:
                lat = msg.lat / 1e7  # Convert to degrees
                lon = msg.lon / 1e7
                alt = msg.alt / 1000  # Convert to meters
                return f"GPS: Lat {lat:.6f}, Lon {lon:.6f}, Alt {alt:.2f}m"
            else:
                return "GPS: No Data"
        except Exception as e:
            return f"GPS: Error - {e}"

    # *** End of Added Section ***

    # *** Added Handle Detection and Save Image with GPS ***
    def on_detection_received(self, detection_info):
        """Handle detection events received from serial communication."""
        self.status_label.setText(f"Status: Detection - {detection_info}")
        self.save_image_with_gps(detection_info)
    # *** End of Added Section ***

    # *** Removed Method to View Full Screen ***
    # The view_full_screen method is no longer needed and has been removed.
    # *** End of Removed Section ***

    # *** Added Key Press Event Handling ***
    # (Already included above)
    # *** End of Added Section ***

    # SerialComs signal handlers are already connected in __init__

    # *** Added Map Marker Functionality ***
    # (Already included above)
    # *** End of Added Section ***


# Application setup
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())