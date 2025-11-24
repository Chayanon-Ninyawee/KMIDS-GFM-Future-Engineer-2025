# Modules (`modules/`)

This directory contains foundational hardware and communication interfaces, primarily designed for running on the embedded microcontroller (e.g., Raspberry Pi Pico) but also applicable to the main processor.

These components handle low-level device control, sensor data acquisition, and inter-process communication (IPC) through I2C.

______________________________________________________________________

## Components Overview

This directory is divided into sub-modules based on the hardware system or communication protocol they manage:

| Module Directory | Description | API Reference Link |
| :--- | :--- | :--- |
| **`controllers/bno085`** | **IMU Driver.** Provides a high-level wrapper for the BNO085 Inertial Measurement Unit (IMU), handling initialization, calibration, and continuous reading of fused orientation (Euler angles) and acceleration data. | [controllers/bno085/README.md](controllers/bno085/README.md) |
| **`controllers/motor`** | **Motor & Encoder Control.** Contains drivers for DC motors (`motor_controller.h`), quadrature encoders (`encoder_controller.h`), and the closed-loop PID-based speed regulator (`motor_speed_controller.h`). | [controllers/motor/README.md](controllers/motor/README.md) |
| **`controllers/servo`** | **Servo Control.** Provides precise PWM control for standard hobby RC servos, mapping angles to pulse widths using hardware PWM. | [controllers/servo/README.md](controllers/servo/README.md) |
| **`i2c_slave`** | **I2C Communication.** Implements the I2C Slave protocol, managing a shared memory buffer for inter-process communication between the microcontroller (Slave) and the main computer (Master). | [i2c_slave/README.md](i2c_slave/README.md) |

______________________________________________________________________

## Building

The build configuration for all modules is managed by the `CMakeLists.txt` files within the respective subdirectories.
