#include "lidar_module.h"

LidarModule::LidarModule(const char *serialPort, int baudRate)
    : serialPort_(serialPort)
    , baudRate_(baudRate)
    , logger_(nullptr) {}

LidarModule::LidarModule(Logger *logger, const char *serialPort, int baudRate)
    : serialPort_(serialPort)
    , baudRate_(baudRate)
    , logger_(logger) {}

LidarModule::~LidarModule() {
    shutdown();
}

bool LidarModule::initialize() {
    if (initialized_) {
        std::cout << "[LidarModule] Already initialized." << std::endl;
        return true;
    }

    lidarDriver_ = *sl::createLidarDriver();
    if (!lidarDriver_) {
        std::cerr << "[LidarModule] Failed to create SLAMTEC LIDAR driver." << std::endl;
        return false;
    }

    serialChannel_ = *sl::createSerialPortChannel(serialPort_, baudRate_);
    if (!serialChannel_) {
        std::cerr << "[LidarModule] Failed to create serial port channel." << std::endl;
        delete lidarDriver_;
        lidarDriver_ = nullptr;
        return false;
    }

    sl_result result = lidarDriver_->connect(serialChannel_);
    if (SL_IS_FAIL(result)) {
        std::cerr << "[LidarModule] Failed to connect to the LIDAR." << std::endl;
        delete serialChannel_;
        serialChannel_ = nullptr;
        delete lidarDriver_;
        lidarDriver_ = nullptr;
        return false;
    }

    initialized_ = true;
    return true;
}

bool LidarModule::start() {
    if (!initialized_) {
        std::cerr << "[LidarModule] Failed to start. Not initialized." << std::endl;
        return false;
    }
    if (running_) {
        std::cout << "[LidarModule] Already started." << std::endl;
        return true;
    }

    lidarDriver_->setMotorSpeed();
    sl_result result = lidarDriver_->startScan(0, 1);
    if (SL_IS_FAIL(result)) {
        std::cerr << "[LidarModule] Failed to start scan." << std::endl;
        lidarDriver_->setMotorSpeed(0);
        return false;
    }

    running_ = true;
    lidarThread_ = std::thread(&LidarModule::scanLoop, this);
    return true;
}

void LidarModule::stop() {
    if (!running_) {
        std::cout << "[LidarModule] Not started." << std::endl;
        return;
    }

    running_ = false;
    if (lidarThread_.joinable()) {
        lidarThread_.join();
    }

    if (lidarDriver_) {
        lidarDriver_->stop();
        lidarDriver_->setMotorSpeed(0);
    }
}

void LidarModule::shutdown() {
    if (running_) stop();

    if (lidarDriver_) {
        lidarDriver_->disconnect();
        delete lidarDriver_;
        lidarDriver_ = nullptr;
    }

    if (serialChannel_) {
        delete serialChannel_;
        serialChannel_ = nullptr;
    }

    initialized_ = false;
}

bool LidarModule::getData(TimedLidarData &outTimedLidarData) const {
    std::lock_guard<std::mutex> lock(lidarDataMutex_);

    if (lidarDataBuffer_.empty()) return false;

    outTimedLidarData = lidarDataBuffer_.latest().value();
    return true;
}

size_t LidarModule::bufferSize() const {
    std::lock_guard<std::mutex> lock(lidarDataMutex_);
    return lidarDataBuffer_.size();
}

bool LidarModule::getAllTimedLidarData(std::vector<TimedLidarData> &outTimedLidarData) const {
    std::lock_guard<std::mutex> lock(lidarDataMutex_);

    if (lidarDataBuffer_.empty()) return false;

    outTimedLidarData = lidarDataBuffer_.getAll();
    return true;
}

bool LidarModule::waitForData(TimedLidarData &outTimedLidarData) {
    std::unique_lock<std::mutex> lock(lidarDataMutex_);
    lidarDataUpdated_.wait(lock, [this] { return !lidarDataBuffer_.empty(); });

    outTimedLidarData = lidarDataBuffer_.latest().value();
    return true;
}

void LidarModule::scanLoop() {
    while (running_) {
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t count = sizeof(nodes) / sizeof(nodes[0]);

        if (SL_IS_FAIL(lidarDriver_->grabScanDataHq(nodes, count))) {
            std::cerr << "[LidarModule] Timeout error" << std::endl;
        }

        lidarDriver_->ascendScanData(nodes, count);

        std::vector<RawLidarNode> temp(count);
        for (size_t i = 0; i < count; ++i) {
            float angle = nodes[i].angle_z_q14 * 90.f / (1 << 14);
            float distance = nodes[i].dist_mm_q2 / 1000.f / (1 << 2);
            uint8_t quality = nodes[i].quality;

            temp[i] = {angle, distance, quality};
        }

        TimedLidarData timedScan{std::move(temp), std::chrono::steady_clock::now()};

        if (logger_ and logging_) {
            uint64_t ts = std::chrono::duration_cast<std::chrono::nanoseconds>(timedScan.timestamp.time_since_epoch()).count();
            logger_->writeData(ts, timedScan.lidarData.data(), timedScan.lidarData.size() * sizeof(RawLidarNode));
        }

        {
            std::lock_guard<std::mutex> lock(lidarDataMutex_);
            lidarDataBuffer_.push(std::move(timedScan));
            lidarDataUpdated_.notify_all();
        }
    }
}

void LidarModule::startLogging() {
    logging_ = true;
}

void LidarModule::stopLogging() {
    logging_ = false;
}

bool LidarModule::printDeviceInfo() {
    sl_lidar_response_device_info_t info;
    if (SL_IS_FAIL(lidarDriver_->getDeviceInfo(info))) {
        std::cerr << "[LidarModule] Failed to retrieve device information." << std::endl;
        return false;
    }

    std::cout << "LIDAR Device Info:" << std::endl;
    std::cout << " - Model: " << static_cast<int>(info.model) << std::endl;
    std::cout << " - Firmware: " << (info.firmware_version >> 8) << "." << (info.firmware_version & 0xFF) << std::endl;
    std::cout << " - Hardware: " << static_cast<int>(info.hardware_version) << std::endl;
    std::cout << " - Serial: ";
    for (int i = 0; i < 16; ++i)
        printf("%02X", info.serialnum[i]);
    std::cout << std::endl;
    return true;
}

void LidarModule::printScanData(const std::vector<RawLidarNode> &nodeDataVector) {
    for (const auto &node : nodeDataVector) {
        printf("Angle: %.3f\tDistance: %.3f\tQuality: %u\n", node.angle, node.distance, node.quality);
    }
    std::cout << "Node Count: " << nodeDataVector.size() << std::endl;
}
