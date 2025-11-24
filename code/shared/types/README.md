## `imu_struct.h` Reference: Inertial Measurement Unit (IMU) Data

This header defines fundamental structures used to represent raw sensor data derived from an Inertial Measurement Unit, specifically covering acceleration and orientation (Euler angles).

______________________________________________________________________

### Structure: `ImuAccel`

Represents the instantaneous acceleration measured along the three primary axes ($X, Y, Z$) of the sensor.

| Field | Type | Description |
| :--- | :--- | :--- |
| **`x`** | `float` | Acceleration along the X-axis (e.g., in $\\text{m}/\\text{s}^2$ or G-forces). |
| **`y`** | `float` | Acceleration along the Y-axis. |
| **`z`** | `float` | Acceleration along the Z-axis. |

______________________________________________________________________

### Structure: `ImuEuler`

Represents the orientation of the sensor in terms of Euler angles: Heading (Yaw), Roll, and Pitch.

| Field | Type | Description |
| :--- | :--- | :--- |
| **`h`** | `float` | **Heading (Yaw)**: Rotation around the vertical Z-axis. |
| **`r`** | `float` | **Roll**: Rotation around the longitudinal X-axis. |
| **`p`** | `float` | **Pitch**: Rotation around the lateral Y-axis. |
