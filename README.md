# Accelerometer Validation Test Automation

This project implements a Python-based test script for validating the behavior of an ADXL345 accelerometer on a device under test (DUT), based on a simulated embedded hardware testing scenario.

The test environment assumes:
- A test board providing power, I2C communication, and actuator control.
- A Python framework (`test_board_fwk`) that abstracts hardware interactions.
- The DUT is subjected to a series of motion configurations, and sensor readings are validated against expected g-force ranges.

## 🧠 Learning Goals
- Design embedded systems validation through software automation.
- Practice reading and interpreting accelerometer datasheets.
- Implement robust I2C communication handling and exception-safe hardware interactions in Python.

## 🛠️ Key Features
- Configures ADXL345 to operate at the maximum data rate within constraints.
- Performs a built-in self-test and validates result ranges.
- Monitors accelerometer data during motion:
  - `slow_climb`: Validate tilt limits for Y and Z axes.
  - `sharp_turn`: Validate acceleration on X and Y axes.
  - `quick_drop`: Validate negative Z-axis acceleration.
- Always logs test result with detailed timing and failure details.
- Graceful exception handling to ensure safe cleanup and reporting.

## ⚠️ Note
This code is intended as a conceptual demonstration of embedded test automation. This script is **non-executable in its current form**. It relies on a simulated hardware interaction layer (`test_board_fwk`) not included in this repository. The intent of this project is to showcase:

- Python-based test sequencing for embedded hardware
- ADXL345 accelerometer configuration and validation
- System-level test planning and exception management


