#include "lidar_module.h"

using namespace sl;

LidarModule::LidarModule(const char *serialPort, int baudRate)
    : serialPort(serialPort)
    , baudRate(baudRate)
    , lidarDriver(nullptr)
    , serialChannel(nullptr) {}

LidarModule::~LidarModule() {
    shutdown();
}

bool LidarModule::initialize() {
    lidarDriver = *createLidarDriver();
    if (!lidarDriver) {
        std::cerr << "Failed to create SLAMTEC LIDAR driver." << std::endl;
        return false;
    }

    serialChannel = *createSerialPortChannel(serialPort, baudRate);
    if (!serialChannel) {
        std::cerr << "Failed to create serial port channel." << std::endl;
        delete lidarDriver;
        lidarDriver = nullptr;
        return false;
    }

    sl_result result = lidarDriver->connect(serialChannel);
    if (SL_IS_FAIL(result)) {
        std::cerr << "Failed to connect to the LIDAR." << std::endl;
        delete serialChannel;
        serialChannel = nullptr;
        delete lidarDriver;
        lidarDriver = nullptr;
        return false;
    }

    sl_lidar_response_device_info_t deviceInfo;
    result = lidarDriver->getDeviceInfo(deviceInfo);
    if (SL_IS_OK(result)) {
        std::cout << "LIDAR Device Info:" << std::endl;
        std::cout << " - Model: " << static_cast<int>(deviceInfo.model) << std::endl;
        std::cout << " - Firmware Version: " << (deviceInfo.firmware_version >> 8) << "." << (deviceInfo.firmware_version & 0xFF)
                  << std::endl;
        std::cout << " - Hardware Version: " << static_cast<int>(deviceInfo.hardware_version) << std::endl;
        std::cout << " - Serial Number: ";
        for (int i = 0; i < 16; ++i) {
            printf("%02X", deviceInfo.serialnum[i]);
        }
        std::cout << std::endl;
    } else {
        std::cerr << "Failed to retrieve device information." << std::endl;
        return false;
    }

    return true;
}

void LidarModule::shutdown() {
    stop();

    if (lidarDriver) {
        lidarDriver->disconnect();
        delete lidarDriver;
        lidarDriver = nullptr;
    }

    if (serialChannel) {
        delete serialChannel;
        serialChannel = nullptr;
    }
}

bool LidarModule::start() {
    if (!lidarDriver) return false;

    lidarDriver->setMotorSpeed();
    sl_result result = lidarDriver->startScan(0, 1);
    if (SL_IS_FAIL(result)) {
        std::cerr << "Failed to start scan." << std::endl;
        lidarDriver->setMotorSpeed(0);
        return false;
    }

    return true;
}

std::vector<RawLidarNode> LidarModule::getData() {
    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t nodeCount = sizeof(nodes) / sizeof(sl_lidar_response_measurement_node_hq_t);

    sl_result result = lidarDriver->grabScanDataHq(nodes, nodeCount);
    if (SL_IS_FAIL(result)) return {};

    std::vector<RawLidarNode> nodeDataVector(nodeCount);

    lidarDriver->ascendScanData(nodes, nodeCount);
    for (size_t i = 0; i < nodeCount; ++i) {
        float angle = nodes[i].angle_z_q14 * 90.f / (1 << 14);
        float distance = nodes[i].dist_mm_q2 / 1000.f / (1 << 2);
        nodeDataVector[i] = {angle, distance};
    }

    return nodeDataVector;
}

void LidarModule::stop() {
    if (lidarDriver) {
        lidarDriver->stop();
        lidarDriver->setMotorSpeed(0);
    }
}

void LidarModule::printScanData(const std::vector<RawLidarNode> &nodeDataVector) {}
