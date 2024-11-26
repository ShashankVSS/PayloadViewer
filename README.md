
# Payload Viewer and Serial Communication System

This repository contains scripts to manage a payload viewer application and serial communication between a Jetson device and another computer. 

## Project Structure

- **`payload_viewer.py`**: Main application script for viewing the payload on the primary computer.
- **`serialcoms.py`**: Handles serial communication on the primary computer.
- **`serialreciever_jetson.py`**: Script to be run on the Jetson device to receive and process serial data.
- **`requirements.txt`**: List of Python dependencies needed for this project.

## Installation

1. Clone this repository to your local machine:
   ```bash
   git clone <repository_url>
   cd <repository_name>
   ```

2. Install the required Python packages:
   ```bash
   pip install -r requirements.txt
   ```

3. Ensure any additional packages not listed in `requirements.txt` are installed manually:
   - **`pipreqs`**: Used for regenerating the `requirements.txt` file when additional dependencies are added.
     ```bash
     pip install pipreqs
     ```

## Usage

### On the Primary Computer:
1. Run the payload viewer application:
   ```bash
   python payload_viewer.py
   ```

### On the Jetson Device:
1. Ensure the Jetson device is connected to the same network as the primary computer.
2. Run the serial receiver script:
   ```bash
   python serialreciever_jetson.py
   ```

## Notes

- Make sure the Jetson device and primary computer can communicate via the chosen serial interface.
- If any additional dependencies are required, add them to the `requirements.txt` file and regenerate it using `pipreqs`:
  ```bash
  pipreqs /path/to/project
  ```
