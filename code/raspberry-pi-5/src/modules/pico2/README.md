## `Pico2Module.h` Reference: High-Level Pico2 I2C Interface

This header defines the `Pico2Module` class, which provides a high-level, thread-safe interface for managing data and commands with a Pico 2 microcontroller over I2C. It features continuous background polling of sensor data (IMU, Encoder) and buffering for asynchronous consumption.

______________________________________________________________________

### Class: `Pico2Module`

| Feature | Description |
| :--- | :--- |
| **I2C Master** | Manages communication via the internal `I2cMaster` instance. |
| **High-Frequency Polling** | Runs a background thread (`pollingLoop`) that reads sensor data at a fixed rate ($\\approx 120 \\text{ Hz}$). |
| **Data Buffering** | Stores timestamped samples (`TimedPico2Data`) in a `RingBuffer` to decouple I2C polling from application logic. |
| **Thread Safety** | Uses `std::mutex` and `std::condition_variable` to synchronize access and enable blocking reads. |

#### Constructors and Destructor

| Method | Description | Parameters |
| :--- | :--- | :--- |
| **`explicit Pico2Module(uint8_t i2cAddress)`** | **Constructor (No Logging).** Sets up the internal `I2cMaster` instance with the specified address. | `i2cAddress`: I2C address of the Pico2 (default: `0x39`). |
| **`Pico2Module(Logger *logger, uint8_t i2cAddress)`** | **Constructor (With Logging).** Same as above, but stores a pointer to an external `Logger` instance for optional sample logging. | `logger`: Pointer to a `Logger` instance (can be `nullptr`). <br> `i2cAddress`: I2C address of the Pico2. |
| **`~Pico2Module()`** | **Destructor.** Ensures the background polling thread is stopped and joined by automatically calling `shutdown()`. | N/A |

#### Initialization and Status

| Method | Description | Returns |
| :--- | :--- | :--- |
| **`bool initialize()`** | Opens the I2C bus, confirms device communication, and launches the high-frequency background `pollingLoop` thread. | `true` if initialization and thread start are successful. |
| **`void shutdown()`** | Stops the background polling thread, waits for it to finish, and closes the I2C interface. | N/A |
| **`bool isReady() const`** | Checks if the module has been successfully initialized and the polling thread is actively running. | `true` if active and initialized. |
| **`bool isImuReady() const`** | Returns the last known state of the IMU ready flag as reported by the Pico2 status register. | `true` if the IMU reports readiness. |

#### Command (Actuator) Methods

| Method | Description | Parameters | Returns |
| :--- | :--- | :--- | :--- |
| **`bool setMovementInfo(float motorSpeed, float steeringPercent)`** | Writes the motor speed and steering targets to the Pico2's I2C registers. | `motorSpeed`: Target motor speed (type depends on Pico2 firmware). <br> `steeringPercent`: Steering command, typically $\\in [-100.0, 100.0]$. | `true` if the I2C write was successful. |

#### Data Retrieval (Thread-Safe)

| Method | Description | Parameters | Returns |
| :--- | :--- | :--- | :--- |
| **`bool getData(TimedPico2Data &outData) const`** | **Non-blocking read.** Retrieves the latest IMU/encoder sample from the internal ring buffer. | `outData`: Output structure filled with the most recent sample. | `true` if a sample is available, `false` if the buffer is empty. |
| **`size_t bufferSize() const`** | Returns the current count of samples stored in the ring buffer. | N/A | Current number of samples. |
| **`bool getAllTimedData(std::vector<TimedPico2Data> &outData) const`** | Retrieves **all** samples currently in the buffer in chronological order. Does not empty the buffer. | `outData`: Vector filled with all stored samples. | `true` if at least one sample was retrieved. |
| **`bool waitForData(TimedPico2Data &outData)`** | **Blocking read.** Suspends the caller until a new sample is produced by the polling thread, utilizing a condition variable. | `outData`: Output structure filled with the newly captured sample. | `true` if new data was retrieved successfully. |

#### Logging Control

| Method | Description |
| :--- | :--- |
| **`void startLogging()`** | Enables forwarding of captured Pico2 samples to the configured `Logger` instance. |
| **`void stopLogging()`** | Disables sample logging. |

#### Private Members (Implementation Details)

| Member | Type | Description |
| :--- | :--- | :--- |
| **`pollingLoop()`** | `void` | The background thread function that performs continuous I2C read operations. |
| **`master_`** | `I2cMaster` | The underlying I2C communication handler. |
| **`running_`** | `std::atomic<bool>` | Atomic flag to control the `pollingLoop` execution state. |
| **`pollingThread_`** | `std::thread` | The dedicated thread running `pollingLoop`. |
| **`dataMutex_`** | `std::mutex` | Mutex protecting shared resources (`dataBuffer_`, `dataUpdated_`). |
| **`dataUpdated_`** | `std::condition_variable` | Used to signal consumer threads whenever a new sample is captured. |
| **`status_`** | `pico_i2c_mem_addr::StatusFlags` | Stores the last read status flags from the Pico2 for quick access (e.g., in `isImuReady()`). |
| **`dataBuffer_`** | `RingBuffer<TimedPico2Data>` | Circular buffer for storing the latest $120$ samples. |
