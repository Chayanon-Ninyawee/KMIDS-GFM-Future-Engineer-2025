## `LidarModule.h` Reference: Threaded SLAMTEC LIDAR Manager

This header defines the `LidarModule` class, which handles the complex tasks of initializing, controlling, and acquiring data from a SLAMTEC (Slamware) LIDAR device. It uses a dedicated thread for continuous scanning and a ring buffer for thread-safe data access.

______________________________________________________________________

### Class: `LidarModule`

| Feature | Description |
| :--- | :--- |
| **Driver** | Wraps the external `sl::ILidarDriver` and `sl::IChannel` (serial communication). |
| **Threaded Scan** | Runs a background thread (`scanLoop`) to handle the blocking nature of data acquisition. |
| **Data Buffer** | Stores recent complete scans in a `RingBuffer<TimedLidarData>`. |
| **Thread Safety** | Uses a `std::mutex` and `std::condition_variable` to synchronize access between the capture thread and consumer threads. |

#### Constructors and Initialization

| Method | Description | Parameters |
| :--- | :--- | :--- |
| **`LidarModule(...)`** | **Constructor (No Logging).** Sets up internal parameters. Device initialization requires calling `initialize()`. | `serialPort`: Path to the serial device (default: `"/dev/ttyAMA0"`). <br> `baudRate`: Communication speed (default: $1000000$). |
| **`LidarModule(Logger *logger, ...)`** | **Constructor (With Logging).** Same as above, but stores a pointer to an external `Logger` instance for optional data logging. | `logger`: Pointer to a `Logger` instance. <br> `serialPort`, `baudRate`: (Same as above). |
| **`~LidarModule()`** | **Destructor.** Safely stops scanning by calling `stop()` and releases the driver resources via `shutdown()`. | N/A |
| **`bool initialize()`** | Establishes the serial connection, creates the LIDAR driver instance, and prepares the device for motor start/scanning. | N/A |
| **`void shutdown()`** | Stops the LIDAR motor, disconnects the serial channel, and cleans up the driver instance. Safe to call multiple times. | N/A |

#### Scanning and Control

| Method | Description | Returns |
| :--- | :--- | :--- |
| **`bool start()`** | Starts the LIDAR motor, instructs the driver to begin fetching scans, and launches the `scanLoop` thread. | `true` if scanning and thread launch are successful. |
| **`void stop()`** | Halts the LIDAR motor, stops the driver's scan process, and waits for the background thread to finish execution. | N/A |
| **`bool printDeviceInfo()`** | Queries the connected LIDAR device for identifying information (model ID, firmware version, etc.) and prints it to the console. | `true` if the information was successfully retrieved. |
| **`static void printScanData(...)`** | **Static Utility.** Prints a vector of raw LIDAR nodes to the console for debugging purposes. | N/A |

#### Data Retrieval (Thread-Safe)

| Method | Description | Parameters | Returns |
| :--- | :--- | :--- | :--- |
| **`bool getData(TimedLidarData &outTimedLidarData) const`** | **Non-blocking read.** Retrieves the most recently completed scan frame from the internal ring buffer. | `outTimedLidarData`: Output structure to receive the scan points and timestamp. | `true` if a scan is available, `false` otherwise. |
| **`bool waitForData(TimedLidarData &outTimedLidarData)`** | **Blocking read.** Suspends the calling thread until a **new** scan is completed and pushed to the buffer. | `outTimedLidarData`: Output structure to receive the newly captured scan. | `true` if new data was successfully retrieved. |
| **`size_t bufferSize() const`** | Returns the number of scan frames currently held in the internal `RingBuffer`. | N/A | Size of the buffer. |
| **`bool getAllTimedLidarData(...) const`** | Retrieves **all** scan frames currently stored in the buffer, ordered from oldest to newest scan. | `outTimedLidarData`: Vector to be filled with all buffered frames. | `true` if the buffer is non-empty. |

#### Logging Control

| Method | Description |
| :--- | :--- |
| **`void startLogging()`** | Enables forwarding of captured scan frames to the `Logger` instance provided during construction. |
| **`void stopLogging()`** | Disables scan frame logging. |

#### Private Members (Implementation Details)

| Member | Type | Description |
| :--- | :--- | :--- |
| **`scanLoop()`** | `void` | The function running in the background thread for continuous data acquisition. |
| **`lidarDriver_`** | `sl::ILidarDriver*` | Pointer to the SLAMTEC driver interface. |
| **`serialChannel_`** | `sl::IChannel*` | Pointer to the serial communication handler. |
| **`lidarDataMutex_`** | `std::mutex` | Mutex protecting access to the `lidarDataBuffer_` and `lidarDataUpdated_`. |
| **`lidarDataUpdated_`** | `std::condition_variable` | Used to signal consumer threads whenever a new scan is ready. |
| **`lidarDataBuffer_`** | `RingBuffer<TimedLidarData>` | The circular buffer holding recent scan history. |
