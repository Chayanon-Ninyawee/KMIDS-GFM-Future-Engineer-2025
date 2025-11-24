#pragma once
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

/**
 * @brief Represents a single generic log entry
 */
struct LogEntry {
    uint64_t timestamp;         ///< Timestamp in nanoseconds
    std::vector<uint8_t> data;  ///< Raw data bytes
};

/**
 * @brief A utility class for reading and parsing all entries from a binary log file.
 *
 * The file format is assumed to be a concatenation of entries, where each entry
 * begins with a uint64_t timestamp, followed by a variable length of raw data.
 */
class LogReader
{
public:
    /**
     * @brief Constructs a LogReader object.
     * @param filename The full path to the binary log file to be processed.
     */
    LogReader(const std::string &filename);

    /**
     * @brief Reads all log entries from the file and appends them to the output vector.
     *
     * @param entries A reference to a vector where successfully parsed LogEntry objects will be stored.
     * @return true if the file was successfully opened and read entirely, false otherwise (e.g., file not found).
     */
    bool readAll(std::vector<LogEntry> &entries);

private:
    std::string filePath_;
};
