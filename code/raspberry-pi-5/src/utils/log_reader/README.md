## `log_reader.h` Reference: Binary Log File Handling

This header defines the necessary structure and class for reading and parsing generic entries from a binary log file.

______________________________________________________________________

### Structures

#### `LogEntry`

Represents a single generic log entry extracted from the file.

| Field | Type | Description |
| :--- | :--- | :--- |
| **`timestamp`** | `uint64_t` | Timestamp measured in **nanoseconds**. |
| **`data`** | `std::vector<uint8_t>` | **Raw data bytes** associated with the entry. |

______________________________________________________________________

### Class: `LogReader`

A utility class designed for reading and parsing *all* entries contained within a binary log file.

> **File Format Assumption:**
> The binary file is expected to be a direct concatenation of entries. Each entry must start with a `uint64_t` timestamp, followed immediately by a variable length of raw data bytes.

#### Public Methods

| Method | Description |
| :--- | :--- |
| **`LogReader(const std::string &filename)`** | Constructs a `LogReader` object, specifying the full path to the binary log file. |
| **`bool readAll(std::vector<LogEntry> &entries)`** | Reads all entries from the file. Successfully parsed `LogEntry` objects are appended to the `entries` vector. Returns `true` on successful read of the entire file, or `false` if the file could not be opened/read. |

#### Private Members

- `std::string filePath_`: Stores the path of the binary log file being processed.
