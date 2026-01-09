# Jetson-Scripts

This repository features Python scripts for GPIO and PWM control on the NVIDIA Jetson Nano. 

## Project Backstory

These scripts were originally developed as part of a project to create a **robotic manipulator arm**. 

The initial design phase explored using **520 high-power DC geared motors with Hall encoders** for actuation. While the final implementation of the arm transitioned to using **LX16A series bus servos** for their daisy-chaining capabilities and precision, this repository preserves the code used during the prototyping and testing phase. 

It specifically focuses on:
- Validating the Jetson Nano's GPIO hardware.
- Experimenting with smooth motion control using PWM pulses and PID loops.

## Repository Contents

### `servo_control.py`
A script designed to control standard PWM servos (or compatible motor drivers). Key features include:
- **PWM Generation**: Configured for 50Hz signals (standard adaptable servo frequency).
- **PID Control**: Implements a `PIDController` class to manage movement, ensuring smooth transitions between angles rather than abrupt jumps.
- **Adjustable Parameters**: Includes constants for modifying Pulse Width (MIN/MAX/MID) and PID gains (KP/KI/KD).

### `test_GPIO.py`
A lightweight diagnostic tool to verify board connectivity.
- Setup specific GPIO pins (default: Pin 33).
- Toggles output HIGH/LOW to test hardware response.

## Prerequisites

- **Hardware**: NVIDIA Jetson Nano
- **Software**: Python 3
- **Libraries**: `Jetson.GPIO`

## Usage

1. **Setup Permissions**: Ensure your user has permissions to access the GPIO pins (usually requires adding the user to the `gpio` group).
   ```bash
   sudo groupadd -f -r gpio
   sudo usermod -a -G gpio $USER
   ```

2. **Run Scripts**:
   ```bash
   python3 servo_control.py
   # or
   python3 test_GPIO.py
   ```
