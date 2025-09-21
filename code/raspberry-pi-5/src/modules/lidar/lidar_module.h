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
 * Runs a background thread that continuously collects LIDAR scan data.
 * Provides thread-safe access to the latest scan points and timestamps.
 * Handles initialization, shutdown, and motor control for the LIDAR.
 */
class LidarModule
{
public:
    /**
     * @brief Construct the Lidar module.
     *
     * Sets up internal state but does not start scanning.
     *
     * @param serialPort Path to the serial port used to communicate with the LIDAR (default "/dev/ttyAMA0").
     * @param baudRate Baud rate for the LIDAR communication (default 1000000, and was 460800).
     */
    LidarModule(const char *serialPort = "/dev/ttyAMA0", int baudRate = 1000000);

    LidarModule(Logger *logger, const char *serialPort = "/dev/ttyAMA0", int baudRate = 1000000);

    /**
     * @brief Destroy the Lidar module.
     *
     * Stops scanning, shuts down the device, and releases all resources.
     */
    ~LidarModule();

    /**
     * @brief Initialize the LIDAR driver and connect to the device.
     *
     * @return true if initialization succeeds, false otherwise.
     */
    bool initialize();

    /**
     * @brief Stop the LIDAR and clean up resources.
     */
    void shutdown();

    /**
     * @brief Start LIDAR scanning and begin data acquisition.
     *
     * @return true if scanning starts successfully, false otherwise.
     */
    bool start();

    /**
     * @brief Stop LIDAR scanning and halt the motor.
     */
    void stop();

    /**
     * @brief Get the latest LIDAR scan data.
     *
     * Thread-safe. Copies the latest scan data (points and timestamp) into the provided
     * output parameter.
     *
     * @param[out] outTimedLidarData Structure to receive the latest scan points and timestamp.
     *
     * @return true if data is available, false if no scan has been captured yet.
     */
    bool getData(TimedLidarData &outTimedLidarData) const;

    /**
     * @brief Wait until new LIDAR scan data is available, then return it.
     *
     * This function blocks until a new scan is captured, then copies the scan data
     * (points and timestamp) into the provided output parameter.
     *
     * @param[out] outTimedLidarData Structure to receive the latest scan points and timestamp.
     *
     * @return true if data was successfully retrieved.
     */
    bool waitForData(TimedLidarData &outTimedLidarData);

    size_t bufferSize() const;

    bool getAllTimedLidarData(std::vector<TimedLidarData> &outTimedLidarData) const;

    /**
     * @brief Print information about the connected LIDAR device.
     *
     * @return true if device info was successfully printed.
     */
    bool printDeviceInfo();

    /**
     * @brief Print a vector of LIDAR scan nodes to the console.
     *
     * @param nodeDataVector Vector of RawLidarNode containing the scan data to print.
     */
    static void printScanData(const std::vector<RawLidarNode> &nodeDataVector);

private:
    /**
     * @brief Background thread function for continuous scanning.
     *
     * Captures LIDAR scan frames in a loop and updates the latest data.
     */
    void scanLoop();

    sl::ILidarDriver *lidarDriver_;
    sl::IChannel *serialChannel_;
    const char *serialPort_;
    int baudRate_;

    bool initialized_;

    std::thread lidarThread_;
    std::atomic<bool> running_;

    mutable std::mutex lidarDataMutex_;
    std::condition_variable lidarDataUpdated_;

    RingBuffer<TimedLidarData> lidarDataBuffer_{10};

    Logger *logger_;
};
