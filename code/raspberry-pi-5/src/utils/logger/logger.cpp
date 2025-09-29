#include "logger.h"

Logger::Logger(const std::string &filename) {
    std::filesystem::path filePath(filename);
    auto dir = filePath.parent_path();
    if (!dir.empty() && !std::filesystem::exists(dir)) {
        if (!std::filesystem::create_directories(dir)) {
            throw std::runtime_error("Failed to create directory: " + dir.string());
        }
    }

    file.open(filename, std::ios::binary | std::ios::out);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open log file: " + filename);
    }
}

Logger::~Logger() {
    if (file.is_open()) {
        file.close();
    }
}

void Logger::writeData(uint64_t timestamp_ns, const void *data, size_t dataSize) {
    std::lock_guard<std::mutex> lock(mtx);

    file.write(reinterpret_cast<const char *>(&timestamp_ns), sizeof(timestamp_ns));
    file.write(reinterpret_cast<const char *>(&dataSize), sizeof(dataSize));
    if (dataSize > 0 && data != nullptr) {
        file.write(reinterpret_cast<const char *>(data), dataSize);
    }
}
