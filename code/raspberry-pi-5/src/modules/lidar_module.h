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

/**
 * LidarController class handles initialization, control, and data acquisition
 * from a SLAMTEC LIDAR device.
 */
class LidarModule
{
public:
    /**
     * Constructor to initialize LidarController with the serial port and baud rate.
     * @param serialPort - The serial port path (e.g., "/dev/ttyAMA0").
     * @param baudRate - Baud rate for communication with the LIDAR.
     */
    LidarModule(const char *serialPort = "/dev/ttyAMA0", int baudRate = 460800);

    /**
     * Destructor to clean up resources and shut down the LIDAR.
     */
    ~LidarModule();

    /**
     * Initializes the LIDAR driver and establishes a connection with the device.
     * @return true if initialization succeeds, false otherwise.
     */
    bool initialize();

    /**
     * Shuts down the LIDAR, stops scanning, and cleans up all resources.
     */
    void shutdown();

    /**
     * Starts scanning using the LIDAR. Initiates motor spin and begins data acquisition.
     * @return true if the scan starts successfully, false otherwise.
     */
    bool start();

    /**
     * Stops the LIDAR scanning and halts the motor.
     */
    void stop();

    bool getData(std::vector<RawLidarNode> &outLidarData, std::chrono::steady_clock::time_point &outTimestamp);

    /**
     * Prints the LIDAR scan data to the console.
     * @param nodeDataVector - A vector of NodeData containing the scan results to be printed.
     */
    static void printScanData(const std::vector<RawLidarNode> &nodeDataVector);

private:
    /**
     * @brief The function running in the background thread.
     *
     * Continuously captures frames from the camera.
     */
    void scanLoop();

    sl::ILidarDriver *lidarDriver;  ///< Pointer to the LIDAR driver instance.
    sl::IChannel *serialChannel;    ///< Pointer to the communication channel.
    const char *serialPort;         ///< Serial port used for LIDAR connection.
    int baudRate;                   ///< Baud rate for the communication.

    std::thread lidarThread_;
    std::atomic<bool> running_;

    std::mutex lidarDataMutex_;
    std::condition_variable lidarDataUpdated_;

    std::vector<RawLidarNode> latestLidarData_;
    std::chrono::steady_clock::time_point latestTimestamp_;
};
