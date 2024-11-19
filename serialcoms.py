# serialcoms.py

import serial
import serial.tools.list_ports
from PyQt6.QtCore import QObject, QThread, pyqtSignal, QTimer
from time import perf_counter
import ast  # Added for parsing
import math  # Added for checking inf, -inf, and nan

# Define ping and pong messages
PING_MESSAGE = "PING"
PONG_MESSAGE = "PONG"


class SerialComs(QObject):
    # Define signals to communicate with the main application
    connected = pyqtSignal()
    disconnected = pyqtSignal()
    error = pyqtSignal(str)
    data_received = pyqtSignal(str)
    latency_measured = pyqtSignal(float)  # Signal to emit latency in milliseconds
    processed_ranges_avg = pyqtSignal(str)  # Changed to str to include "-i"
    encoder_position_received = pyqtSignal(int)  # New signal for encoder position
    extension_state_received = pyqtSignal(str)  # New signal for extension state

    def __init__(self, port=None, baudrate=None, timeout=1, parent=None):
        super().__init__(parent)
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self._running = False

        # Timer for sending periodic pings every 10 seconds
        self.ping_timer = QTimer()
        self.ping_timer.setInterval(10000)  # 10,000 ms = 10 seconds
        self.ping_timer.timeout.connect(self.send_ping)

        # Timer for waiting for pong response (5 seconds)
        self.pong_timer = QTimer()
        self.pong_timer.setSingleShot(True)
        self.pong_timer.setInterval(5000)  # 5,000 ms = 5 seconds
        self.pong_timer.timeout.connect(self.check_pong)

        self.pong_received = False
        self.ping_timestamp = None  # Store the timestamp of the last ping

    def connect_serial(self):
        """Establish the serial connection and start the reading thread."""
        if not self.port or not self.baudrate:
            self.error.emit("Port and baudrate must be set before connecting.")
            return

        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            if self.serial.is_open:
                self._running = True
                self.thread = QThread()
                self.moveToThread(self.thread)
                self.thread.started.connect(self.run)
                self.thread.start()
                self.ping_timer.start()
                self.send_ping()  # Send initial ping
                self.connected.emit()
        except serial.SerialException as e:
            self.error.emit(str(e))

    def disconnect_serial(self):
        """Close the serial connection and stop the reading thread."""
        self._running = False
        self.ping_timer.stop()
        self.pong_timer.stop()
        if self.serial and self.serial.is_open:
            self.serial.close()
        if hasattr(self, 'thread') and self.thread.isRunning():
            self.thread.quit()
            self.thread.wait()
        self.disconnected.emit()

    def send_data(self, data):
        """Send data over the serial connection."""
        if self.serial and self.serial.is_open:
            try:
                self.serial.write((data + '\n').encode('utf-8'))  # Ensure newline delimiter
            except serial.SerialException as e:
                self.error.emit(str(e))
                self.disconnect_serial()

    def send_ping(self):
        """Send a ping message to verify connection."""
        if self.serial and self.serial.is_open:
            self.pong_received = False
            self.ping_timestamp = perf_counter()  # Record the time when ping is sent
            self.send_data(PING_MESSAGE)
            self.pong_timer.start()

    def check_pong(self):
        """Check if a pong response was received."""
        if not self.pong_received:
            self.error.emit("No response received.")
            self.disconnect_serial()

    def run(self):
        """Continuously read incoming data from the serial port."""
        while self._running:
            try:
                if self.serial.in_waiting:
                    try:
                        line = self.serial.readline()
                        decoded_line = line.decode('utf-8').strip()
                    except UnicodeDecodeError as e:
                        self.error.emit(f"Unicode decode error: {e}")
                        continue  # Skip this line and continue

                    if decoded_line:
                        if decoded_line == PONG_MESSAGE:
                            self.pong_received = True
                            if self.ping_timestamp is not None:
                                latency = (perf_counter() - self.ping_timestamp) * 1000  # Convert to ms
                                self.latency_measured.emit(latency)
                                self.ping_timestamp = None  # Reset the timestamp
                        elif decoded_line == PING_MESSAGE:
                            # Respond to ping with pong
                            self.send_data(PONG_MESSAGE)
                        else:
                            # Emit any other received data
                            self.process_message(decoded_line)
                else:
                    self.thread_sleep(100)  # Prevent high CPU usage
            except serial.SerialException as e:
                self.error.emit(str(e))
                self.disconnect_serial()

    def thread_sleep(self, ms):
        """Sleep for a specified number of milliseconds."""
        QThread.msleep(ms)

    def list_available_ports(self):
        """List all available serial ports."""
        return [port.device for port in serial.tools.list_ports.comports()]

    def process_message(self, message):
        if message.startswith("DATA|"):
            # Example message: "DATA|processed_ranges:(-inf, 0.28, nan, 0.27)"
            try:
                _, data = message.split('|', 1)
                topic, value = data.split(':', 1)

                if topic == "processed_ranges":
                    # Parse the value manually, accounting for -inf, inf, and nan
                    try:
                        # Remove parentheses and split into individual items
                        ranges = value.strip("()").split(",")
                        valid_values = []
                        for r in ranges:
                            try:
                                r = float(r.strip())
                                # Ignore invalid values
                                if not math.isinf(r) and not math.isnan(r):
                                    valid_values.append(r)
                            except ValueError:
                                # Ignore parsing errors
                                continue

                        if valid_values:
                            average = sum(valid_values) / len(valid_values)
                            result = f"{average:.2f}"
                            if len(valid_values) != len(ranges):
                                result += " -i"  # Indicate some values were ignored
                            self.processed_ranges_avg.emit(result)
                            self.data_received.emit(f"Average Processed Ranges: {result}")
                            print(f"Averaged processed_ranges: {result}")
                        else:
                            self.error.emit("All processed_ranges values are invalid.")
                    except Exception as e:
                        self.error.emit(f"Error parsing processed_ranges: {value} - {str(e)}")
                elif topic == "encoder_position":
                    # Parse encoder position as integer
                    encoder_value = int(value)
                    self.encoder_position_received.emit(encoder_value)
                    self.data_received.emit(f"Encoder Position: {encoder_value}")
                    print(f"Encoder Position: {encoder_value}")
                elif topic == "extension_setting":
                    # Parse extension state as string
                    extension_state = value.strip()
                    self.extension_state_received.emit(extension_state)
                    self.data_received.emit(f"Extension State: {extension_state}")
                    print(f"Extension State: {extension_state}")
                else:
                    self.error.emit(f"Unknown topic: {topic}")
            except (ValueError, SyntaxError, TypeError) as e:
                self.error.emit(f"Error parsing data: {e}")
            return  # Exit after handling DATA| messages

        # Existing message handling for PING, PONG, etc.
        if message == PONG_MESSAGE:
            self.pong_received = True
            if self.ping_timestamp is not None:
                latency = (perf_counter() - self.ping_timestamp) * 1000  # Convert to ms
                self.latency_measured.emit(latency)
                self.ping_timestamp = None  # Reset the timestamp
        elif message == PING_MESSAGE:
            # Respond to ping with pong
            self.send_data(PONG_MESSAGE)
        else:
            # Emit any other received data
            self.data_received.emit(message)