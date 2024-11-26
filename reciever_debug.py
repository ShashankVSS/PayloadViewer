# receiver.py

import serial
import time

# Define ping and pong messages
PING_MESSAGE = "PING"
PONG_MESSAGE = "PONG"

def main():
    port = 'COM4'  # Replace with the receiver's serial port
    baudrate = 9600  # Must match the sender's baudrate
    timeout = 1  # Read timeout in seconds

    try:
        ser = serial.Serial(port, baudrate, timeout=timeout)
        print(f"Opened serial port {port} at {baudrate} baud.")
    except serial.SerialException as e:
        print(f"Failed to open serial port {port}: {e}")
        return

    try:
        while True:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8').strip()
                if line == PING_MESSAGE:
                    print("Received PING, sending PONG.")
                    ser.write((PONG_MESSAGE + '\n').encode('utf-8'))
                elif line == PONG_MESSAGE:
                    print("Received PONG.")
                elif line.startswith("TEST_COMMAND"):
                    response = "TEST_COMMAND_RESPONSE"
                    print(f"Received {line}, sending {response}.")
                    ser.write((response + '\n').encode('utf-8'))
                else:
                    # Echo back any other custom commands with a response
                    response = f"ACK:{line}"
                    print(f"Received {line}, sending {response}.")
                    ser.write((response + '\n').encode('utf-8'))
            time.sleep(0.1)  # Sleep briefly to prevent high CPU usage
    except KeyboardInterrupt:
        print("Exiting.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
