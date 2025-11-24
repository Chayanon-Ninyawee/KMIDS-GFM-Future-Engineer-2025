# Source Code (`src/`)

This directory contains the main application components, architectural layers, and foundational building blocks of the robot's operating system. It is organized into distinct directories based on functionality, promoting modularity and clear separation of concerns.

Each subdirectory contains specialized components and is managed by its own `README.md` file, which provides detailed documentation and API references.

______________________________________________________________________

## Architecture Overview

The codebase is logically organized into the following layers:

| Directory | Layer Type | Purpose | API Reference Link |
| :--- | :--- | :--- | :--- |
| **`types`** | Foundational | Defines all fundamental C++ structures (`structs`) used throughout the project for data exchange, ensuring consistent and type-safe interfaces between layers. | [types/README.md](types/README.md) |
| **`utils`** | Foundational | Contains reusable, domain-specific utilities like data logging, general-purpose algorithms (e.g., PID controllers, Ring Buffers), and geometric/directional concepts. | [utils/README.md](utils/README.md) |
| **`modules`** | Hardware Interface | Contains high-level drivers and device managers that interact directly with the hardware (e.g., Camera, LIDAR, Pico 2). These components handle device communication and run dedicated background threads for data acquisition. | [modules/README.md](modules/README.md) |
| **`processors`** | Core Logic | Contains algorithms for processing and fusing raw sensor data into meaningful environmental awareness, such as line extraction, object classification, and motion compensation. | [processors/README.md](processors/README.md) |
| **`shared`** | Cross-Cutting | Contains components that are shared across both Raspberry Pi 5 and Raspberry Pi Pico 2 (e.g., common constants, error handling patterns). | [shared/README.md](shared/README.md) |

______________________________________________________________________

## Building

The build process for all components within `src/` is managed by the top-level project's CMake configuration, which recursively includes the `CMakeLists.txt` files found in the subdirectories.
