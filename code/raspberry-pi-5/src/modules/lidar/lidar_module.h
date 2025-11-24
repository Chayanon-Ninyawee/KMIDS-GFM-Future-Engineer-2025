#pragma once

#include "sl_lidar.h"
#include "sl_lidar_driver.h"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "lidar_struct.h"
#include "logger.h"
#include "ring_buffer.hpp"

/**
 * @brief Lidar module that manages scanning and data acquisition from a SLAMTEC LIDAR device.
 *
 * Runs a background thread that continuously collects scan data from the LIDAR.
 * Provides thread-safe access to the latest scan points and timestamps.
 * Handles initialization, shutdown, and motor control of the LIDAR hardware.
 */
class LidarModule
{
public:
    /**
     * @brief Construct the Lidar module.
     *
     * Sets up internal state but does not initialize or start scanning.
     *
     * @param serialPort Path to the serial port used to communicate with the LIDAR
     *        (default "/dev/ttyAMA0").
     * @param baudRate Baud rate for the LIDAR communication
     *        (default 1000000; older devices may use 460800).
     */
    LidarModule(const char *serialPort = "/dev/ttyAMA0", int baudRate = 1000000);

    /**
     * @brief Construct the Lidar module with logging support.
     *
     * Stores the provided logger instance and prepares internal state.
     * The device is not initialized until initialize() is called.
     *
     * @param logger Pointer to a Logger instance for optional scan logging.
     * @param serialPort Path to the serial port used to communicate with the LIDAR.
     * @param baudRate Baud rate for the LIDAR communication.
     */
    LidarModule(Logger *logger, const char *serialPort = "/dev/ttyAMA0", int baudRate = 1000000);

    /**
     * @brief Destroy the Lidar module.
     *
     * Ensures scanning is stopped, the LIDAR is shut down, and all resources are released.
     */
    ~LidarModule();

    /**
     * @brief Initialize the LIDAR driver and establish a connection to the device.
     *
     * Opens the serial communication channel, initializes the SLAMTEC driver,
     * and prepares the device for scanning.
     *
     * @return true if initialization succeeds, false otherwise.
     */
    bool initialize();

    /**
     * @brief Shut down the LIDAR and clean up all device resources.
     *
     * Safe to call even if the device is not initialized.
     * Stops the motor and releases the driver.
     */
    void shutdown();

    /**
     * @brief Start LIDAR scanning and begin data acquisition.
     *
     * Launches the background scan thread and begins collecting frames.
     *
     * @return true if scanning starts successfully, false otherwise.
     */
    bool start();

    /**
     * @brief Stop LIDAR scanning and halt the motor.
     *
     * Waits for the background scan thread to terminate.
     */
    void stop();

    /**
     * @brief Get the latest LIDAR scan data.
     *
     * Thread-safe. Copies the latest scan points and timestamp into the provided
     * output parameter.
     *
     * @param[out] outTimedLidarData Structure to receive the most recent scan data.
     *
     * @return true if data is available, false if no scan has been captured yet.
     */
    bool getData(TimedLidarData &outTimedLidarData) const;

    /**
     * @brief Wait until new LIDAR scan data is available, then return it.
     *
     * Blocks the calling thread until a new scan frame is captured.
     * Once available, copies the scan data and timestamp into the output parameter.
     *
     * @param[out] outTimedLidarData Structure to receive the newly captured scan data.
     *
     * @return true if new data was successfully retrieved.
     */
    bool waitForData(TimedLidarData &outTimedLidarData);

    /**
     * @brief Get the current number of scan frames stored in the buffer.
     *
     * Thread-safe. Returns how many scan entries are stored in the internal ring buffer.
     *
     * @return Number of scan frames in the buffer.
     */
    size_t bufferSize() const;

    /**
     * @brief Retrieve all scan frames currently stored in the buffer.
     *
     * Thread-safe. Frames are returned in order from oldest to newest.
     *
     * @param[out] outTimedLidarData Vector to receive all buffered scan frames.
     *
     * @return true if the buffer contains at least one frame, false if empty.
     */
    bool getAllTimedLidarData(std::vector<TimedLidarData> &outTimedLidarData) const;

    /**
     * @brief Enable logging of scan frames.
     *
     * When enabled, captured scan frames are passed to the provided Logger
     * instance, if any.
     */
    void startLogging();

    /**
     * @brief Disable logging of scan frames.
     *
     * No further scan data will be forwarded to the Logger.
     */
    void stopLogging();

    /**
     * @brief Print information about the connected LIDAR device.
     *
     * Queries the device for model, firmware, hardware revision, etc.
     *
     * @return true if device information was successfully retrieved and printed.
     */
    bool printDeviceInfo();

    /**
     * @brief Print a vector of raw LIDAR scan nodes to the console.
     *
     * Useful for debugging raw scan output.
     *
     * @param nodeDataVector Vector containing RawLidarNode entries to print.
     */
    static void printScanData(const std::vector<RawLidarNode> &nodeDataVector);

private:
    /**
     * @brief Background thread function that continuously scans the LIDAR.
     *
     * Captures scan frames in a loop, stores them in the ring buffer,
     * notifies waiting threads, and optionally logs data.
     *
     * Runs until stop() is called.
     */
    void scanLoop();

    sl::ILidarDriver *lidarDriver_ = nullptr;
    sl::IChannel *serialChannel_ = nullptr;

    const char *serialPort_;
    int baudRate_;

    bool initialized_ = false;

    std::thread lidarThread_;
    std::atomic<bool> running_ = false;

    mutable std::mutex lidarDataMutex_;
    std::condition_variable lidarDataUpdated_;

    RingBuffer<TimedLidarData> lidarDataBuffer_{10};

    Logger *logger_ = nullptr;
    bool logging_ = false;
};
