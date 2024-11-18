# payload_viewer.py

import sys
import cv2
import numpy as np
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout, QMenuBar, QMenu,
    QPushButton, QLabel, QSlider, QStackedWidget, QFrame, QDialog, QFormLayout, 
    QLineEdit, QDialogButtonBox, QMessageBox, QComboBox
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QImage, QPixmap, QAction
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtCore import QUrl

from serialcoms import SerialComs

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
        top_section_widget.setStyleSheet("border-radius: 10px; padding: 10px;")

        # Initialize SerialComs
        self.serial_coms = SerialComs()
        self.serial_coms.connected.connect(self.on_connected)
        self.serial_coms.disconnected.connect(self.on_disconnected)
        self.serial_coms.error.connect(self.on_error)
        self.serial_coms.data_received.connect(self.on_data_received)
        self.serial_coms.latency_measured.connect(self.update_latency_display)

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
            QPushButton::Hover { 
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
            QPushButton::Hover { 
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
            QPushButton::Hover { 
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

        # Webcam setup
        self.cap = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.start_webcam()

        # Initially disable buttons
        self.update_buttons_state()

    def change_view(self, index):
        """Switch between different views in the stacked widget."""
        self.view_stack.setCurrentIndex(index)
        view_names = ["Webcam", "Map", "Images"]
        self.current_view_label.setText(f"Current View: {view_names[index]}")
        if index == 0:  # Webcam view
            self.start_webcam()
        else:
            self.stop_webcam()

    def start_webcam(self):
        """Start the webcam feed."""
        if self.cap is None:
            self.cap = cv2.VideoCapture(1)  # Adjust camera index as needed
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
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                image = QImage(frame, frame.shape[1], frame.shape[0], QImage.Format.Format_RGB888)
                pixmap = QPixmap.fromImage(image).scaled(1280, 720, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)
                self.webcam_view.setPixmap(pixmap)

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
        dialog = QDialog(self)
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
        baudrates = [9600, 19200, 38400, 57600, 115200]
        self.baud_combo_dialog.addItems([str(b) for b in baudrates])
        self.baud_combo_dialog.setCurrentText(str(self.serial_coms.baudrate))
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

    def update_connection_indicator_dialog(self):
        """Update the visual indicator of the connection status in the dialog."""
        if self.connected:
            self.connection_indicator_dialog.setText("Connection: <span style='color: green;'>⬤</span>")
        else:
            self.connection_indicator_dialog.setText("Connection: <span style='color: red;'>⬤</span>")

    def toggle_connection_dialog(self):
        """Connect or disconnect based on current state from the dialog."""
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

    def update_connection_indicator(self):
        """Update the visual indicator of the connection status."""
        if self.connected:
            self.connection_indicator_top.setText("Connection: <span style='color: green;'>⬤</span>")
        else:
            self.connection_indicator_top.setText("Connection: <span style='color: red;'>⬤</span>")

    def update_latency_display(self, latency):
        """Update the latency display in the connection settings."""
        if hasattr(self, 'latency_display_label'):
            self.latency_display_label.setText(f"{latency:.2f} ms")

    def on_connected(self):
        """Handle actions when a serial connection is established."""
        self.connected = True
        self.update_connection_indicator()
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
        if hasattr(self, 'connection_indicator'):
            self.update_connection_indicator()
        QMessageBox.critical(self, "Serial Error", message)
        self.connected = False
        self.update_buttons_state()
        # Reset Auto Adjust to default state
        self.reset_auto_adjust()

    def on_data_received(self, data):
        """Handle data received from the serial connection."""
        # Handle received data if needed
        print(f"Data received: {data}")

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
            self.serial_coms.send_data("EXTEND")
            # Update Extend button style to indicate active state
            self.bottom_button2.setStyleSheet("""
                QPushButton { 
                    background-color: #006400; 
                    border: none; 
                    border-radius: 5px; 
                    font-size: 16px; 
                    color: #FFFFFF; 
                } 
                QPushButton::Hover { 
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
                QPushButton::Hover { 
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
            self.serial_coms.send_data("RETRACT")
            # Update Retract button style to indicate active state
            self.bottom_button1.setStyleSheet("""
                QPushButton { 
                    background-color: #8B0000; 
                    border: none; 
                    border-radius: 5px; 
                    font-size: 16px; 
                    color: #FFFFFF; 
                } 
                QPushButton::Hover { 
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
                QPushButton::Hover { 
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
            self.serial_coms.send_data("STOP")
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
                QPushButton::Hover { 
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
                QPushButton::Hover { 
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
        self.stop_webcam()
        event.accept()

    # SerialComs signal handlers
    def on_connected(self):
        """Handle actions when a serial connection is established."""
        self.connected = True
        self.update_connection_indicator()
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
        if hasattr(self, 'connection_indicator'):
            self.update_connection_indicator()
        QMessageBox.critical(self, "Serial Error", message)
        self.connected = False
        self.update_buttons_state()
        # Reset Auto Adjust to default state
        self.reset_auto_adjust()

    def on_data_received(self, data):
        """Handle data received from the serial connection."""
        # Handle received data if needed
        print(f"Data received: {data}")

# Application setup
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
