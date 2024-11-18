# sender.py

import sys
import serial
import serial.tools.list_ports
from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, 
    QComboBox, QLineEdit, QTextEdit, QMessageBox, QGroupBox, QGridLayout
)
from PyQt6.QtCore import Qt, QThread, pyqtSignal

# Define ping and pong messages
PING_MESSAGE = "PING"
PONG_MESSAGE = "PONG"

class SerialReaderThread(QThread):
    # Define a signal to emit received messages
    message_received = pyqtSignal(str)
    error_occurred = pyqtSignal(str)

    def __init__(self, ser):
        super().__init__()
        self.ser = ser
        self._running = True

    def run(self):
        while self._running:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line:
                        self.message_received.emit(line)
                self.msleep(100)  # Sleep for 100 ms
            except serial.SerialException as e:
                self.error_occurred.emit(str(e))
                break
            except Exception as e:
                self.error_occurred.emit(str(e))
                break

    def stop(self):
        self._running = False
        self.wait()


class SenderGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Serial Communication Sender")
        self.setGeometry(100, 100, 600, 400)
        self.ser = None
        self.reader_thread = None

        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()

        # Connection Settings Group
        conn_group = QGroupBox("Connection Settings")
        conn_layout = QGridLayout()

        # Serial Port Selection
        self.port_label = QLabel("Serial Port:")
        self.port_combo = QComboBox()
        self.refresh_button = QPushButton("Refresh")
        self.refresh_button.clicked.connect(self.refresh_ports)

        # Baudrate Selection
        self.baud_label = QLabel("Baudrate:")
        self.baud_combo = QComboBox()
        baudrates = [9600, 19200, 38400, 57600, 115200]
        self.baud_combo.addItems([str(b) for b in baudrates])
        self.baud_combo.setCurrentText("9600")

        # Connect/Disconnect Button
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.toggle_connection)

        # Arrange Connection Settings
        conn_layout.addWidget(self.port_label, 0, 0)
        conn_layout.addWidget(self.port_combo, 0, 1)
        conn_layout.addWidget(self.refresh_button, 0, 2)
        conn_layout.addWidget(self.baud_label, 1, 0)
        conn_layout.addWidget(self.baud_combo, 1, 1)
        conn_layout.addWidget(self.connect_button, 1, 2)

        conn_group.setLayout(conn_layout)
        layout.addWidget(conn_group)

        # Message Sending Group
        send_group = QGroupBox("Send Messages")
        send_layout = QHBoxLayout()

        # Predefined Buttons
        self.ping_button = QPushButton("Send PING")
        self.ping_button.clicked.connect(lambda: self.send_message(PING_MESSAGE))
        self.pong_button = QPushButton("Send PONG")
        self.pong_button.clicked.connect(lambda: self.send_message(PONG_MESSAGE))

        # Custom Command
        self.custom_input = QLineEdit()
        self.custom_input.setPlaceholderText("Enter custom command")
        self.send_custom_button = QPushButton("Send Custom")
        self.send_custom_button.clicked.connect(self.send_custom_command)

        # Arrange Sending Controls
        send_layout.addWidget(self.ping_button)
        send_layout.addWidget(self.pong_button)
        send_layout.addWidget(self.custom_input)
        send_layout.addWidget(self.send_custom_button)

        send_group.setLayout(send_layout)
        layout.addWidget(send_group)

        # Received Messages Display
        recv_group = QGroupBox("Received Messages")
        recv_layout = QVBoxLayout()
        self.recv_text = QTextEdit()
        self.recv_text.setReadOnly(True)
        recv_layout.addWidget(self.recv_text)
        recv_group.setLayout(recv_layout)
        layout.addWidget(recv_group)

        self.setLayout(layout)

        # Initialize UI State
        self.update_ui_state(disconnected=True)

        # Refresh ports after defining connect_button
        self.refresh_ports()

    def refresh_ports(self):
        """Refresh the list of available serial ports."""
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        port_list = [port.device for port in ports]
        if not port_list:
            self.port_combo.addItem("No Ports Found")
            self.connect_button.setEnabled(False)
        else:
            self.port_combo.addItems(port_list)
            self.connect_button.setEnabled(True)

    def toggle_connection(self):
        """Connect or disconnect based on current state."""
        if self.ser and self.ser.is_open:
            self.disconnect_serial()
        else:
            self.connect_serial()

    def connect_serial(self):
        """Establish the serial connection."""
        selected_port = self.port_combo.currentText()
        if selected_port == "No Ports Found":
            QMessageBox.critical(self, "Connection Error", "No serial ports available.")
            return
        baudrate = int(self.baud_combo.currentText())

        try:
            self.ser = serial.Serial(selected_port, baudrate, timeout=1)
            self.reader_thread = SerialReaderThread(self.ser)
            self.reader_thread.message_received.connect(self.display_received_message)
            self.reader_thread.error_occurred.connect(self.handle_serial_error)
            self.reader_thread.start()
            self.update_ui_state(disconnected=False)
            QMessageBox.information(self, "Connected", f"Connected to {selected_port} at {baudrate} baud.")
        except serial.SerialException as e:
            QMessageBox.critical(self, "Connection Error", f"Failed to connect to {selected_port}.\nError: {e}")

    def disconnect_serial(self):
        """Close the serial connection."""
        if self.reader_thread:
            self.reader_thread.stop()
            self.reader_thread = None
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.ser = None
        self.update_ui_state(disconnected=True)
        QMessageBox.information(self, "Disconnected", "Serial connection has been closed.")

    def update_ui_state(self, disconnected):
        """Enable or disable UI elements based on connection state."""
        if disconnected:
            self.connect_button.setText("Connect")
            self.ping_button.setEnabled(False)
            self.pong_button.setEnabled(False)
            self.send_custom_button.setEnabled(False)
            self.custom_input.setEnabled(False)
        else:
            self.connect_button.setText("Disconnect")
            self.ping_button.setEnabled(True)
            self.pong_button.setEnabled(True)
            self.send_custom_button.setEnabled(True)
            self.custom_input.setEnabled(True)

    def send_message(self, message):
        """Send a predefined message over serial."""
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((message + '\n').encode('utf-8'))
                self.recv_text.append(f"Sent: {message}")
            except serial.SerialException as e:
                QMessageBox.critical(self, "Send Error", f"Failed to send message.\nError: {e}")
        else:
            QMessageBox.warning(self, "Not Connected", "Please connect to a serial port first.")

    def send_custom_command(self):
        """Send a custom command entered by the user."""
        command = self.custom_input.text().strip()
        if command:
            self.send_message(command)
            self.custom_input.clear()
        else:
            QMessageBox.warning(self, "Input Error", "Please enter a command to send.")

    def display_received_message(self, message):
        """Display received messages in the text area."""
        self.recv_text.append(f"Received: {message}")

    def handle_serial_error(self, error_message):
        """Handle serial errors."""
        QMessageBox.critical(self, "Serial Error", f"An error occurred:\n{error_message}")
        self.disconnect_serial()

    def closeEvent(self, event):
        """Handle the window close event to ensure serial port is closed."""
        if self.ser and self.ser.is_open:
            self.disconnect_serial()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = SenderGUI()
    window.show()
    sys.exit(app.exec())
