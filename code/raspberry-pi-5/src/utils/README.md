# Utilities (`utils/`)

This directory contains reusable, domain-specific utilities and foundational components used throughout the project, such as data structures, logging mechanisms, and control algorithms.

Each subdirectory contains a component and its associated `README.md` file, providing detailed documentation, usage examples, and API references.

______________________________________________________________________

## Components Overview

Use the links below to navigate to the detailed documentation for each utility.

| Component Directory | Description | API Reference Link |
| :--- | :--- | :--- |
| **`direction`** | Handles cardinal directions, rotation, relative sides, and path segmentation concepts. | [direction/README.md](direction/README.md) |
| **`logger`** | Provides a thread-safe implementation for binary logging of sensor data streams. | [logger/README.md](logger/README.md) |
| **`log_reader`** | A utility class for parsing and reading entries from the standard binary log files created by the `logger`. | [log_reader/README.md](log_reader/README.md) |
| **`pid_controller`** | A simple Proportional-Integral-Derivative (PID) controller class for closed-loop control applications. | [pid_controller/README.md](pid_controller/README.md) |
| **`ring_buffer`** | A generic, fixed-size circular buffer (ring buffer) template class for storing recent historical data. | [ring_buffer/README.md](ring_buffer/README.md) |

______________________________________________________________________

## Building

The build configuration for all utilities is handled by the `CMakeLists.txt` file in this directory.
