# Shared Components (`shared/`)

This directory contains common definitions, constants, and fundamental data structures that are used across multiple primary components or subsystems in the project. The contents are organized into functional subdirectories.

______________________________________________________________________

## Subdirectories Overview

| Subdirectory | Description | Core Content | Detailed Reference |
| :--- | :--- | :--- | :--- |
| **`i2c`** | Definitions related to Inter-Integrated Circuit (I2C) communication protocols and memory addresses for hardware components. | `pico_i2c_mem_addr.h` | [i2c/README.md](i2c/README.md) |
| **`types`** | Fundamental, low-level data structure definitions, such as Inertial Measurement Unit (IMU) measurements. | `imu_struct.h` | [types/README.md](types/README.md) |

______________________________________________________________________

## Detailed Reference

### `shared/i2c`

This directory provides header files defining specific I2C memory maps and register addresses, commonly required when interacting with microcontrollers or peripheral hardware.

The primary file here is `pico_i2c_mem_addr.h`, which contains constants and definitions for the I2C interface on the Pico microcontroller.

### `shared/types`

This directory provides foundational structures that represent raw sensor data or common mathematical entities, essential for system operation.

The primary file here is `imu_struct.h`, which defines the structures used to hold raw or processed Inertial Measurement Unit data.
