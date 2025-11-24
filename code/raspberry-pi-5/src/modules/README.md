# Modules (`modules/`)

This directory contains high-level hardware interfaces and drivers designed to manage the robot's primary sensor and actuation systems. These modules encapsulate device initialization, multi-threading, data acquisition loops, and provide thread-safe access to sensor data.

All modules share a common pattern: they run a dedicated background thread to poll data and use a ring buffer to store recent samples, allowing the main application to consume data asynchronously.

______________________________________________________________________

## Components Overview

Use the links below to navigate to the detailed documentation for each hardware interface module.

| Module Directory | Description | API Reference Link |
| :--- | :--- | :--- |
| **`camera`** | Manages frame acquisition from the camera device in a background thread, providing thread-safe access to the latest video frames. | [camera/README.md](camera/README.md) |
| **`i2c_master`** | Provides the low-level I2C communication interface for the Raspberry Pi to interact with the Pico microcontrollers. | [i2c_master/README.md](i2c_master/README.md) |
| **`lidar`** | Initializes, controls, and acquires continuous scan data from the SLAMTEC LIDAR device in a separate scanning thread. | [lidar/README.md](lidar/README.md) |
| **`pico2`** | High-level interface to the Pico 2 microcontroller over I2C, handling continuous polling of IMU and encoder data, and transmitting motor control commands. | [pico2/README.md](pico2/README.md) |

______________________________________________________________________

## Building

The build configuration for all modules is handled by the `CMakeLists.txt` file in this directory.
