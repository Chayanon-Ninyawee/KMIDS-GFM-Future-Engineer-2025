#include "pico2_module.h"
#include "pico2_struct.h"

#include <cmath>
#include <iostream>

Pico2Module::Pico2Module(uint8_t i2cAddress)
    : master_(i2cAddress)
    , running_(false)
    , logger_(nullptr) {}

Pico2Module::Pico2Module(Logger *logger, uint8_t i2cAddress)
    : master_(i2cAddress)
    , running_(false)
    , logger_(logger) {}

Pico2Module::~Pico2Module() {
    shutdown();
}

bool Pico2Module::initialize() {
    if (running_) {
        std::cout << "[Pico2Module] Already initialized." << std::endl;
        return true;
    }

    if (!master_.isInitialized()) {
        std::cerr << "[Pico2Module] Failed to initialize I2C master." << std::endl;
        return false;
    }

    running_ = true;
    pollingThread_ = std::thread(&Pico2Module::pollingLoop, this);
    return true;
}

void Pico2Module::shutdown() {
    if (!running_) return;

    running_ = false;
    if (pollingThread_.joinable()) {
        pollingThread_.join();
    }
}

bool Pico2Module::isReady() const {
    return status_.is_running;
}

bool Pico2Module::isImuReady() const {
    return status_.imu_ready;
}

bool Pico2Module::setMovementInfo(float motorSpeed, float steeringPercent) {
    return master_.writeMovementInfo(motorSpeed, steeringPercent);
}

bool Pico2Module::getData(TimedPico2Data &outData) const {
    std::lock_guard<std::mutex> lock(dataMutex_);
    if (dataBuffer_.empty()) return false;

    outData = dataBuffer_.latest().value();
    return true;
}

bool Pico2Module::waitForData(TimedPico2Data &outData) {
    std::unique_lock<std::mutex> lock(dataMutex_);
    dataUpdated_.wait(lock, [this] { return !dataBuffer_.empty(); });

    outData = dataBuffer_.latest().value();
    return true;
}

bool Pico2Module::getAllTimedData(std::vector<TimedPico2Data> &outData) const {
    std::lock_guard<std::mutex> lock(dataMutex_);
    if (dataBuffer_.empty()) return false;

    outData = dataBuffer_.getAll();
    return true;
}

size_t Pico2Module::bufferSize() const {
    std::lock_guard<std::mutex> lock(dataMutex_);
    return dataBuffer_.size();
}

void Pico2Module::pollingLoop() {
    using namespace std::chrono;
    const auto interval = milliseconds(8);  // ~120 Hz

    while (running_) {
        auto start = steady_clock::now();

        // Refresh status
        uint8_t statusByte = 0;
        if (master_.readStatus(statusByte)) {
            status_ = *reinterpret_cast<pico_i2c_mem_addr::StatusFlags *>(&statusByte);
        }

        ImuAccel accel{};
        ImuEuler euler{};
        double encoderAngle = 0.0;

        bool imuOk = master_.readImu(accel, euler);
        bool encOk = master_.readEncoder(encoderAngle);

        static double lastValidEncoderAngle = 0.0;

        if (imuOk && encOk) {
            // Wrap Euler heading into [0,360)
            euler.h = std::fmod(-euler.h, 360.0f);
            if (euler.h < 0) euler.h += 360.0f;

            // Use previous value if encoderAngle is 0
            if (encoderAngle == 0.0)
                encoderAngle = lastValidEncoderAngle;
            else
                lastValidEncoderAngle = encoderAngle;

            TimedPico2Data sample{steady_clock::now(), accel, euler, encoderAngle};

            if (logger_) {
                uint64_t ts = std::chrono::duration_cast<std::chrono::nanoseconds>(sample.timestamp.time_since_epoch()).count();
                logger_->writeData(ts, &sample.accel, sizeof(sample.accel));
                logger_->writeData(ts, &sample.euler, sizeof(sample.euler));
                logger_->writeData(ts, &sample.encoderAngle, sizeof(sample.encoderAngle));
            }

            {
                std::lock_guard<std::mutex> lock(dataMutex_);
                dataBuffer_.push(std::move(sample));
                dataUpdated_.notify_all();
            }
        }

        auto elapsed = steady_clock::now() - start;
        if (elapsed < interval) {
            std::this_thread::sleep_for(interval - elapsed);
        }
    }
}
