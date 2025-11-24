# Source Code (`src/`)

This directory contains the main application components, architectural layers, and foundational building blocks of the robot's operating system. It is organized into distinct directories based on functionality, promoting modularity and clear separation of concerns.

The root of this directory contains the primary entry point for the application.

______________________________________________________________________

## Architecture Overview

The codebase is logically organized into the following layers and core files:

| Directory/File | Layer Type | Purpose | API Reference Link |
| :--- | :--- | :--- | :--- |
| **`main.cpp`** | **Application Entry** | Contains the top-level application initialization, main execution loop, system startup logic, and coordination of data flow between the modules and processors. | N/A |
| **`modules`** | Hardware Interface | Contains high-level drivers and device managers that interact directly with the hardware (e.g., Camera, LIDAR, Pico). These components handle device communication and run dedicated background threads for data acquisition. | [modules/README.md](modules/README.md) |
| **`utils`** | Foundational | Contains reusable, domain-specific utilities like data logging, general-purpose algorithms (e.g., PID controllers), and foundational data structures. | [utils/README.md](utils/README.md) |
| **`shared`** | Cross-Cutting | Contains components that are shared across different execution platforms (Raspberry Pi 5 and Raspberry Pi Pico), such as common constants and header definitions. | [shared/README.md](shared/README.md) |

______________________________________________________________________

## Building

The build process for all components within `src/` is managed by the top-level project's CMake configuration, which recursively includes the `CMakeLists.txt` files found in the subdirectories.
