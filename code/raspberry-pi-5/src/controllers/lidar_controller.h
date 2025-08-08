#pragma once

#include "sl_lidar.h"
#include "sl_lidar_driver.h"
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "lidar_struct.h"

namespace lidar_controller
{

/**
 * LidarController class handles initialization, control, and data acquisition
 * from a SLAMTEC LIDAR device.
 */
class LidarController
{
public:
    /**
     * Constructor to initialize LidarController with the serial port and baud rate.
     * @param serialPort - The serial port path (e.g., "/dev/ttyAMA0").
     * @param baudRate - Baud rate for communication with the LIDAR.
     */
    LidarController(const char *serialPort = "/dev/ttyAMA0", int baudRate = 460800);

    /**
     * Destructor to clean up resources and shut down the LIDAR.
     */
    ~LidarController();

    /**
     * Initializes the LIDAR driver and establishes a connection with the device.
     * @return true if initialization succeeds, false otherwise.
     */
    bool initialize();

    /**
     * Starts scanning using the LIDAR. Initiates motor spin and begins data acquisition.
     * @return true if the scan starts successfully, false otherwise.
     */
    bool startScanning();

    /**
     * Stops the LIDAR scanning and halts the motor.
     */
    void stopScanning();

    /**
     * Retrieves scan data from the LIDAR, including angles and distances.
     * @return A vector of NodeData containing scan results.
     */
    std::vector<NodeData> getScanData();

    /**
     * Shuts down the LIDAR, stops scanning, and cleans up all resources.
     */
    void shutdown();

    /**
     * Prints the LIDAR scan data to the console.
     * @param nodeDataVector - A vector of NodeData containing the scan results to be printed.
     */
    static void printScanData(const std::vector<NodeData> &nodeDataVector);

private:
    sl::ILidarDriver *lidarDriver;  ///< Pointer to the LIDAR driver instance.
    sl::IChannel *serialChannel;    ///< Pointer to the communication channel.
    const char *serialPort;         ///< Serial port used for LIDAR connection.
    int baudRate;                   ///< Baud rate for the communication.
};

}  // namespace lidar_controller
