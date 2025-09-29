#pragma once

#include "i2c_master.h"
#include "logger.h"
#include "pico2_struct.h"
#include "ring_buffer.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

/**
 * @brief High-level interface to the Pico2 module via I2C.
 *
 * The Pico2Module continuously polls IMU and encoder data in a background
 * thread at ~120 Hz and stores timestamped samples in a ring buffer.
 * The buffer can then be consumed by the main application at a different rate
 * (e.g., 60 Hz). Heading values are normalized to [0,360).
 *
 * Typical usage:
 *  1. Construct Pico2Module
 *  2. Call initialize()
 *  3. Retrieve data using getData(), waitForData(), or getAllTimedData()
 *  4. Call shutdown() before destruction
 */
class Pico2Module
{
public:
    /**
     * @brief Construct a new Pico2Module.
     *
     * @param i2cAddress I2C address of the Pico2 device (default 0x39).
     */
    explicit Pico2Module(uint8_t i2cAddress = 0x39);

    /**
     * @brief Construct a new Pico2Module with optional logging.
     *
     * This constructor allows you to pass a Logger instance to enable
     * logging within the Pico2Module. If you do not need logging, you
     * can pass nullptr.
     *
     * @param logger Pointer to a Logger instance, or nullptr if logging is not needed.
     * @param i2cAddress I2C address of the Pico2 device (default 0x39).
     */
    Pico2Module(Logger *logger, uint8_t i2cAddress = 0x39);

    /**
     * @brief Destructor. Ensures polling thread is stopped.
     */
    ~Pico2Module();

    /**
     * @brief Initialize communication and start background polling.
     * @return true if initialization succeeded, false otherwise.
     */
    bool initialize();

    /**
     * @brief Stop background polling and close I2C.
     */
    void shutdown();

    /**
     * @brief Check if the Pico2 module is initialized and running.
     */
    bool isReady() const;

    /**
     * @brief Check if the IMU reports as ready.
     */
    bool isImuReady() const;

    /**
     * @brief Write motor speed and steering command to Pico2.
     *
     * @param motorSpeed Desired motor speed (unit depends on Pico2 firmware).
     * @param steeringPercent Steering command in percent (-100..100).
     * @return true if command was written successfully.
     */
    bool setMovementInfo(float motorSpeed, float steeringPercent);

    /**
     * @brief Retrieve the most recent data sample.
     *
     * @param outData Filled with the latest sample.
     * @return true if valid data was available.
     */
    bool getData(TimedPico2Data &outData) const;

    /**
     * @brief Copy all buffered samples into a vector.
     *
     * @param outData Vector filled with samples in chronological order.
     * @return true if data was available, false if buffer was empty.
     */
    bool getAllTimedData(std::vector<TimedPico2Data> &outData) const;

    /**
     * @brief Get the current number of stored samples.
     */
    size_t bufferSize() const;

    /**
     * @brief Block until a new sample is available.
     *
     * @param outData Filled with the new sample.
     * @return true if data was retrieved, false if module stopped.
     */
    bool waitForData(TimedPico2Data &outData);

    void startLogging();

    void stopLogging();

private:
    /// Background polling loop (runs at ~120 Hz).
    void pollingLoop();

    I2cMaster master_;

    std::atomic<bool> running_ = false;
    std::thread pollingThread_;

    mutable std::mutex dataMutex_;
    std::condition_variable dataUpdated_;

    pico_i2c_mem_addr::StatusFlags status_;

    RingBuffer<TimedPico2Data> dataBuffer_{120};

    Logger *logger_;
    bool logging_ = false;
};
