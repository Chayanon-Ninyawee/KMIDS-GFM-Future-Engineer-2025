## `pico_i2c_mem_addr.h` Reference: Pico I2C Memory Map

This header defines the fixed memory map and addressing scheme used for I2C communication with the Pico microcontroller, allowing external devices to read sensor data and issue commands.

The Pico's shared I2C memory block has a total capacity of **256 bytes**.

______________________________________________________________________

### Global Constants and Sizes (`pico_i2c_mem_addr`)

| Constant | Value | Description |
| :--- | :--- | :--- |
| **`MEM_SIZE`** | $256$ | Total size of the shared I2C memory buffer (bytes). |
| **`COMMAND_SIZE`** | $1$ | Size of the Command register (1 byte). |
| **`STATUS_SIZE`** | $1$ | Size of the Status register (1 byte). |
| **`ACCEL_DATA_SIZE`** | `sizeof(ImuAccel)` | Size of the IMU acceleration data structure. |
| **`EULER_ANGLE_SIZE`** | `sizeof(ImuEuler)` | Size of the IMU Euler angle data structure. |
| **`IMU_DATA_SIZE`** | `ACCEL_DATA_SIZE + EULER_ANGLE_SIZE` | Total size of the combined IMU data payload. |
| **`ENCODER_ANGLE_SIZE`** | `sizeof(double)` | Size of the encoder angle data (typically 8 bytes). |
| **`MOVEMENT_INFO_SIZE`** | `sizeof(double) + sizeof(float)` | Total size for motor speed and steering percentage information. |

### Memory Addresses

The following addresses define the start point for reading or writing specific data blocks within the $256$-byte memory map.

| Address Constant | Address Value | Size (Bytes) | Content |
| :--- | :--- | :--- | :--- |
| **`COMMAND_ADDR`** | $0$ | $1$ | Write address for issuing commands (see `Command` enum). |
| **`STATUS_ADDR`** | $1$ | $1$ | Read address for the system status byte (see `StatusFlags` struct). |
| **`IMU_DATA_ADDR`** | $2$ | `IMU_DATA_SIZE` | Read address for combined IMU data (Accel + Euler). |
| **`ENCODER_ANGLE_ADDR`**| $2$ + `IMU_DATA_SIZE` | `ENCODER_ANGLE_SIZE` | Read address for the current encoder angle. |
| **`MOVEMENT_INFO_ADDR`**| $\\dots$ | `MOVEMENT_INFO_SIZE` | Read/Write address for current motor speed and steering targets. |

______________________________________________________________________

### Commands Namespace

The `pico_i2c_mem_addr::Command` namespace defines the available commands that can be written to the `COMMAND_ADDR` ($0$).

| Enum Value | Hex Value | Description |
| :--- | :--- | :--- |
| **`NO_COMMAND`** | `0x00` | Default state; no active command is being processed. |
| **`RESTART`** | `0x01` | Initiates a software restart of the Pico system. |
| **`CALIB_NO_OFFSET`** | `0x02` | Starts IMU calibration process without applying an offset correction. |
| **`CALIB_WITH_OFFSET`** | `0x03` | Starts IMU calibration process and calculates/applies offset correction. |
| **`SKIP_CALIB`** | `0x04` | Skips the automatic IMU calibration sequence on startup. |

______________________________________________________________________

### Status Flags

The `StatusFlags` structure represents the single status byte read from `STATUS_ADDR` ($1$). It uses bit-fields to pack multiple boolean states into one byte.

| Field | Bits | Description |
| :--- | :--- | :--- |
| **`is_running`** | $1$ | Bit 0: $1$ if the main system loop is active and running. |
| **`imu_ready`** | $1$ | Bit 1: $1$ if the IMU has been initialized and is ready to stream data. |
| **`reserved`** | $6$ | Bits 2-7: Reserved for future use. Always $0$. |
