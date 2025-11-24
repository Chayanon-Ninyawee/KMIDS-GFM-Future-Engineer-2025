# Core Data Structures (types/)

This directory houses the foundational C++ header files that define the primary data structures used across the entire robotics system. These structures typically contain sensor measurements, timestamps, and core state representations (e.g., pose, odometry).

## Structure Definitions Overview

| File | Primary Structures | Description |
| :---- | :---- | :---- |
| **camera_struct.h** | TimedFrame | Encapsulates an OpenCV image frame (cv::Mat) paired with a monotonic timestamp. |
| **lidar_struct.h** | RawLidarNode, TimedLidarData | Definitions for single LIDAR scan points and the full timestamped vector of a complete LIDAR sweep. |
| **pico2_struct.h** | TimedPico2Data | A combined sensor sample structure containing IMU (accelerometer/Euler angles) and encoder data. |
| **robot_pose_struct.h** | RobotDeltaPose | Defines a change in the robot's pose (delta X, delta Y, delta Heading) often calculated from odometry. |
