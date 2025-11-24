## `bno085_controller.h` Reference: BNO085 IMU Driver

This header defines the `Bno085Controller` class, a specialized wrapper for the Hillcrest BNO085 Inertial Measurement Unit (IMU). This class simplifies initialization, configuration, data acquisition, and calibration/tare operations via the I2C interface, providing processed orientation (Euler angles) and acceleration data.

This driver is intended for use on microcontrollers (like the Raspberry Pi Pico) to interface with the IMU.

______________________________________________________________________

### Class: `Bno085Controller`

#### Public Methods

| Method | Description | Parameters | Returns |
| :--- | :--- | :--- | :--- |
| **`Bno085Controller(...)`** | **Constructor.** Sets up the instance variables for the I2C port, pins, and address. | `i2cPort`: Pointer to the I2C instance (e.g., `i2c0`). <br> `sdaPin`: GPIO pin for I2C data. <br> `sclPin`: GPIO pin for I2C clock. <br> `address`: I2C address (default: `0x4a`). | N/A |
| **`bool begin()`** | **Initialization.** Initializes the I2C hardware and communicates with the BNO085 chip. **Must be called first.** | N/A | `true` if IMU is detected and initialized; `false` otherwise. |
| **`bool enableRotation(uint interval_ms = 10)`** | **Sensor Enable.** Requests data from the fused Rotation Vector sensor, which provides accurate, drift-compensated orientation (useful for Euler angles). | `interval_ms`: Data reporting frequency in milliseconds. | `true` if successful. |
| **`bool enableAccelerometer(uint interval_ms = 10)`** | **Sensor Enable.** Requests raw data from the Accelerometer sensor. | `interval_ms`: Data reporting frequency in milliseconds. | `true` if successful. |
| **`bool update(ImuAccel &accel, ImuEuler &euler)`** | **Data Poll.** Checks the IMU for new sensor events. If new data is available, it populates the provided `ImuAccel` and `ImuEuler` structures. | `accel`: Reference to store the latest accelerometer data. <br> `euler`: Reference to store the latest Euler angle data (roll, pitch, yaw). | `true` if new data was read; `false` otherwise. |
| **`bool tareNow(bool zAxis = false, sh2_TareBasis_t basis = SH2_TARE_BASIS_ROTATION_VECTOR)`** | **Tare (Zeroing).** Immediately resets the current orientation reference. This effectively sets the current heading to 0. | `zAxis`: If `true`, only the Z-axis (yaw) is tared. <br> `basis`: The sensor data to use as the tare basis. | `true` if successful. |
| **`bool saveTare()`** | **Non-Volatile Storage.** Saves the current tare configuration to the IMU's non-volatile memory (NVM). | N/A | `true` if successful. |
| **`bool clearTare()`** | **Reset NVM Tare.** Clears any previously saved tare settings from NVM. | N/A | `true` if successful. |
| **`bool setCalibrationConfig(uint8_t sensors)`** | **Calibration Setup.** Configures which internal sensors (e.g., accelerometer, gyroscope, magnetometer) the BNO085 should automatically calibrate. | `sensors`: Bitmask of sensors to include in calibration. | `true` if successful. |
| **`bool saveCalibration()`** | **Non-Volatile Storage.** Saves the current calibration settings to NVM. | N/A | `true` if successful. |
| **`bool wasReset()`** | **Status Check.** Checks if the BNO085 chip has reset (e.g., due to brownout or power cycle) since the last time this function was called. | N/A | `true` if a reset event occurred. |

#### Private Members

| Member | Type | Description |
| :--- | :--- | :--- |
| **`i2c_`, `sdaPin_`, `sclPin_`, `address_`** | Internal | I2C communication configuration details. |
| **`imu_`** | `BNO08x` | The internal object handling the low-level SH-2 protocol communication with the sensor. |

