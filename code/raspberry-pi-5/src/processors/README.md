# Processors (`processors/`)

This directory houses the core data processing and fusion algorithms responsible for interpreting raw sensor data into meaningful, actionable information about the robot's environment, such as object locations, wall segments, and path classification.

Each subdirectory contains a component and its associated `README.md` file, providing detailed documentation, usage examples, and API references.

______________________________________________________________________

## Components Overview

Use the links below to navigate to the detailed documentation for each processor component.

| Component Directory | Description | API Reference Link |
| :--- | :--- | :--- |
| **`camera`** | Handles visual processing, including color filtering in HSV space, contour detection, and calculating the horizontal angle of detected blocks. | [camera/README.md](camera/README.md) |
| **`lidar`** | Focuses on geometric processing of LIDAR scans, including line segment extraction, wall classification, and identifying traffic light clusters. | [lidar/README.md](lidar/README.md) |
| **`combined`** | Implements sensor fusion, synchronizing camera frames and LIDAR scans, combining visual (color) and spatial (position) data, and classifying detected objects relative to the robot's path. | [combined/README.md](combined/README.md) |

______________________________________________________________________

## Building

The build configuration for all processors is handled by the `CMakeLists.txt` file in this directory.
