## `i2c_slave.h` Reference: Inter-Process Communication (I2C Slave)

This header defines the communication interface and data exchange protocol for the I2C Slave device, typically implemented on a microcontroller like the Raspberry Pi Pico. It allows an I2C Master (e.g., a Raspberry Pi companion computer) to read sensor data (IMU, encoder) and write control commands to the microcontroller using an emulated shared memory architecture.

The core communication mechanism is a memory-mapped I2C buffer, where each address corresponds to a specific data point (e.g., motor speed, IMU yaw).

______________________________________________________________________

### Namespace: `i2c_slave`

#### Data Structures

| Struct Name | Description | Members |
| :--- | :--- | :--- |
| **`context_t`** | **I2C Memory Context.** Stores the entire shared memory block and tracks the state of the current I2C transaction, including the address the master is currently accessing. | `uint8_t mem[pico_i2c_mem_addr::MEM_SIZE]`: The shared memory buffer. <br> `uint8_t mem_address`: The register address pointer set by the master. <br> `bool mem_address_written`: Flag indicating if the master has set the memory address pointer. |
| **`context`** | **Global Instance.** The externally declared instance of `context_t` used by the I2C event handler to manage shared memory access. | N/A (Global variable) |

#### Core Initialization and Handling

| Function Signature | Description |
| :--- | :--- |
| **`void contextInit()`** | **Context Setup.** Resets the internal state of the `context` structure, clearing the memory buffer and state flags to prepare for transactions. |
| **`void i2cInit(i2c_inst_t *i2c, uint8_t slaveAddr, uint sdaPin, uint sclPin, uint baudrate)`** | **Hardware Initialization.** Initializes the physical I2C peripheral on the microcontroller, sets the slave address, configures the GPIO pins, and sets the communication baudrate. |
| **`void handler(i2c_inst_t *i2c, i2c_slave_event_t event)`** | **Event Handler.** This function is registered as the I2C interrupt handler. It processes all incoming I2C events (e.g., write-request, read-request, stop) and manages reads/writes to the `context.mem` buffer based on the current `mem_address`. |

#### Data Access Helpers

These functions provide thread-safe, high-level access to the memory-mapped data in the shared `context.mem` buffer, abstracting away the raw byte access and memory addressing details.

| Category | Function Signature | Description |
| :--- | :--- | :--- |
| **Command** | **`uint8_t getCommand()`** | Retrieves the latest control command sent by the master. |
| | **`void setCommand(uint8_t command)`** | Stores a new command byte in the I2C memory for the master to retrieve (primarily used for acknowledging commands). |
| **Status** | **`void setIsRunning(bool isRunning)`** | Sets the system's operational status flag. |
| | **`bool getIsRunning()`** | Retrieves the system's operational status flag. |
| | **`void setIsImuReady(bool ready)`** | Sets the status of the IMU sensor readiness. |
| | **`bool getIsImuReady()`** | Retrieves the status of the IMU sensor readiness. |
| **IMU Data** | **`void setImuData(const ImuAccel &accel, const ImuEuler &euler)`** | Writes the current `ImuAccel` and `ImuEuler` sensor readings into the memory buffer for the master to read. |
| | **`void getImuData(ImuAccel &outAccel, ImuEuler &outEuler)`** | Reads IMU data structures from the memory buffer (less common for the slave, but provided). |
| **Encoder** | **`void setEncoderAngle(double angle)`** | Writes the latest encoder angle (degrees or ticks) into the memory buffer. |
| | **`void getEncoderAngle(double &outAngle)`** | Reads the encoder angle from the memory buffer. |
| **Movement Info**| **`void setMovementInfo(double motorSpeed, float steeringPercent)`**| Writes the current measured motor speed (RPS) and steering output into the memory buffer. |
| | **`void getMovementInfo(double &outMotorSpeed, float &outSteeringPercent)`**| Reads the current motor speed and steering output from the memory buffer. |

