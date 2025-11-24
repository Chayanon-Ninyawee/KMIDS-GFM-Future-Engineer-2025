## `logger.h` Reference: Thread-Safe Binary Logger

This header defines the `Logger` class, a utility for thread-safely writing structured, timestamped sensor and stream data to a single binary file.

______________________________________________________________________

### Class: `Logger`

The `Logger` class provides a mechanism to serialize and save different types of sensor data into a single binary file. Access is synchronized using a `std::mutex` to ensure safe operation when logging is performed from multiple concurrent threads.

#### Logging Format

Every entry written by `writeData` strictly follows this binary format:

| Field | Type | Description |
| :--- | :--- | :--- |
| **Timestamp** | `uint64_t` | The timestamp in nanoseconds. |
| **Data Size** | `size_t` | The number of data bytes following this field. |
| **Data Payload** | `data bytes` | The raw binary payload of size `dataSize`. |

#### Public Methods

| Method | Description |
| :--- | :--- |
| **`Logger(const std::string &filename)`** | **Constructor.** Opens the specified output binary file. Throws `std::runtime_error` if the file cannot be opened. |
| **`~Logger()`** | **Destructor.** Safely closes the log file stream. |
| **`void writeData(uint64_t timestamp_ns, const void *data, size_t dataSize)`** | Writes a block of raw data (`data`) of size (`dataSize`) prefixed by the given `timestamp_ns`. This operation is guarded by a mutex. |
| **`static std::string generateTimestampedFolder(const std::string &baseFolder = "logs")`** | Generates a new, unique folder path based on the current system time (e.g., `logs/YYYYMMDD_HHMMSS`). Creates the directory structure if it does not exist using `std::filesystem::create_directories`. Returns the full path of the created folder. |

#### Private Members

| Member | Type | Description |
| :--- | :--- | :--- |
| `std::ofstream file` | `std::ofstream` | The file stream responsible for writing the binary log data. |
| `std::mutex mtx` | `std::mutex` | The synchronization primitive used to guarantee thread-safe writes to the file stream. |
