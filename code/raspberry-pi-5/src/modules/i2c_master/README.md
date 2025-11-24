## `I2cMaster.h` Reference: Pi-side I2C Master

This header defines the `I2cMaster` class, which serves as the primary communication bridge between the Raspberry Pi (Master) and the Pico 2 microcontroller (Slave) via the I2C protocol. It abstracts the raw register reads and writes into type-safe methods for commanding the Pico and retrieving sensor data.

______________________________________________________________________

### Class: `I2cMaster`

| Feature | Description |
| :--- | :--- |
| **Role** | I2C Master, responsible for initiating all communication. |
| **Slave** | Designed to communicate with the Pico 2 microcontroller using the address scheme defined in `pico_i2c_mem_addr.h`. |
| **Functionality** | Provides high-level methods to send commands, read status flags, and exchange sensor data (IMU, Encoder) and control targets (Motor Speed, Steering). |

#### Constructors and Initialization

| Method | Description |
| :--- | :--- |
| **`explicit I2cMaster(uint8_t slave_addr)`** | **Constructor.** Initializes the I2C master device file descriptor on the Pi using the provided slave address. |
| **`~I2cMaster()`** | **Destructor.** Closes the I2C device file descriptor. |
| **`bool isInitialized() const`** | Checks if the underlying I2C device handle was successfully opened and initialized. |

#### 1. Command Operations

Methods for sending control signals to the Pico slave using the single-byte command register.

| Method | Description | Parameters | Returns |
| :--- | :--- | :--- | :--- |
| **`bool sendCommand(uint8_t command)`** | Sends a single command byte to the Pico (e.g., to initiate restart or calibration). | `command`: The command byte (from `pico_i2c_mem_addr::Command`). | `true` if successful, `false` otherwise. |
| **`bool readCommand(uint8_t &outCommand)`** | Reads the last command byte received or processed by the Pico. | `outCommand`: Reference to store the read command byte. | `true` if successful, `false` otherwise. |

#### 2. Status Operations

Methods for reading the single-byte status register, which contains system and sensor readiness flags.

| Method | Description | Parameters | Returns |
| :--- | :--- | :--- | :--- |
| **`bool readStatus(uint8_t &outStatus)`** | Reads the raw status byte from the slave. | `outStatus`: Reference to store the raw status byte. | `true` if successful, `false` otherwise. |
| **`bool getIsRunning(bool &outIsRunning)`** | Reads the status register and extracts the system running flag. | `outIsRunning`: Reference to store the boolean running state. | `true` if successful, `false` otherwise. |
| **`bool getImuReady(bool &outImuReady)`** | Reads the status register and extracts the IMU ready flag. | `outImuReady`: Reference to store the boolean IMU ready state. | `true` if successful, `false` otherwise. |

#### 3. IMU Operations

Methods for reading and writing the structured IMU data (Accelerometer and Euler Angles).

| Method | Description | Parameters | Returns |
| :--- | :--- | :--- | :--- |
| **`bool readImu(ImuAccel &outAccel, ImuEuler &outEuler)`** | Reads the full IMU data block from the slave (located at `IMU_DATA_ADDR`). | `outAccel`: Reference to store accelerometer data. <br> `outEuler`: Reference to store Euler angles. | `true` if successful, `false` otherwise. |
| **`bool writeImu(const ImuAccel &accel, const ImuEuler &euler)`** | Writes the full IMU data block to the slave (used for debugging or initialization). | `accel`: Accelerometer data to write. <br> `euler`: Euler angles to write. | `true` if successful, `false` otherwise. |

#### 4. Encoder Operations

Methods for exchanging the robot's encoder angle data (represented as a `double`).

| Method | Description | Parameters | Returns |
| :--- | :--- | :--- | :--- |
| **`bool readEncoder(double &outAngle)`** | Reads the current encoder angle from the slave (located at `ENCODER_ANGLE_ADDR`). | `outAngle`: Reference to store the encoder angle. | `true` if successful, `false` otherwise. |
| **`bool writeEncoder(double angle)`** | Writes the encoder angle to the slave. | `angle`: Encoder angle to write. | `true` if successful, `false` otherwise. |

#### 5. Movement Operations

Methods for exchanging control targets (Motor Speed and Steering Percentage).

| Method | Description | Parameters | Returns |
| :--- | :--- | :--- | :--- |
| **`bool readMovementInfo(double &outMotorSpeed, float &outSteeringPercent)`** | Reads the motor speed and steering percentage from the slave (located at `MOVEMENT_INFO_ADDR`). | `outMotorSpeed`: Reference to store motor speed. <br> `outSteeringPercent`: Reference to store steering percentage. | `true` if successful, `false` otherwise. |
| **`bool writeMovementInfo(double motorSpeed, float steeringPercent)`** | Writes the desired motor speed and steering percentage to the slave. | `motorSpeed`: Motor speed to write. <br> `steeringPercent`: Steering percentage to write. | `true` if successful, `false` otherwise. |

#### Private Members (Implementation Details)

| Member | Type | Description |
| :--- | :--- | :--- |
| **`fd_`** | `int` | The file descriptor for the I2C device node (e.g., `/dev/i2c-1`). |
| **`slaveAddr_`** | `uint8_t` | The configured I2C address of the Pico slave. |
| **`writeRegister(uint8_t reg, const uint8_t *data, size_t len)`** | Low-level helper to write a block of data starting at a specific memory register address. |
| **`readRegister(uint8_t reg, uint8_t *data, size_t len)`** | Low-level helper to read a block of data starting from a specific memory register address. |
