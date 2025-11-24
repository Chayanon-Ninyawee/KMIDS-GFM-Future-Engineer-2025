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
 * @brief High-level interface to the Pico2 module over I2C.
 *
 * The Pico2Module continuously polls IMU and encoder data in a background
 * thread at ~120 Hz and stores timestamped samples in a ring buffer.
 * The buffer can then be consumed by the main application at a different rate
 * (e.g., 60 Hz). Heading values are normalized to [0, 360).
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
     * Sets up the internal state but does not start communication or the
     * background polling thread.
     *
     * @param i2cAddress I2C address of the Pico2 device (default 0x39).
     */
    explicit Pico2Module(uint8_t i2cAddress = 0x39);

    /**
     * @brief Construct a new Pico2Module with optional logging support.
     *
     * Allows passing a Logger instance to enable internal logging such as
     * recording samples or status transitions. If nullptr is passed, logging
     * is disabled.
     *
     * @param logger Pointer to a Logger instance, or nullptr if logging is not required.
     * @param i2cAddress I2C address of the Pico2 device (default 0x39).
     */
    Pico2Module(Logger *logger, uint8_t i2cAddress = 0x39);

    /**
     * @brief Destructor. Ensures polling thread is stopped.
     *
     * Automatically calls shutdown() if the module is still running.
     */
    ~Pico2Module();

    /**
     * @brief Initialize I2C communication and start background polling.
     *
     * Opens the I2C bus, checks device status, and launches the polling thread.
     *
     * @return true if initialization was successful, false otherwise.
     */
    bool initialize();

    /**
     * @brief Stop the background polling thread and close the I2C interface.
     *
     * Safe to call multiple times. After shutdown(), isReady() returns false.
     */
    void shutdown();

    /**
     * @brief Check whether the module is initialized and the polling thread is running.
     *
     * @return true if initialize() succeeded and polling is active.
     */
    bool isReady() const;

    /**
     * @brief Check whether the IMU status flag reports that the IMU is ready.
     *
     * Reads the last fetched status from the Pico2 polling loop.
     *
     * @return true if IMU reports as ready.
     */
    bool isImuReady() const;

    /**
     * @brief Write motor speed and steering commands to the Pico2 device.
     *
     * @param motorSpeed Desired motor speed (unit depends on Pico2 firmware).
     * @param steeringPercent Steering command in percent, range -100..100.
     * @return true if the command was successfully written via I2C.
     */
    bool setMovementInfo(float motorSpeed, float steeringPercent);

    /**
     * @brief Retrieve the most recent data sample.
     *
     * Thread-safe. Copies the latest IMU/encoder sample into @p outData.
     *
     * @param[out] outData Filled with the most recent sample.
     * @return true if a sample was available, false if buffer is empty.
     */
    bool getData(TimedPico2Data &outData) const;

    /**
     * @brief Retrieve all available buffered samples in chronological order.
     *
     * Thread-safe. Does not clear the buffer.
     *
     * @param[out] outData Vector that receives all stored samples.
     * @return true if at least one sample was available, false if buffer was empty.
     */
    bool getAllTimedData(std::vector<TimedPico2Data> &outData) const;

    /**
     * @brief Get the current number of stored samples in the buffer.
     *
     * Thread-safe.
     *
     * @return Current number of buffered samples.
     */
    size_t bufferSize() const;

    /**
     * @brief Block until a new sample is available.
     *
     * Waits for the polling thread to produce new data. Copies it into
     * @p outData once available.
     *
     * @param[out] outData Filled with the newly captured sample.
     * @return true if data was retrieved, false if the module was shut down.
     */
    bool waitForData(TimedPico2Data &outData);

    /**
     * @brief Enable logging of Pico2 samples.
     *
     * When enabled, each captured sample is forwarded to the Logger
     * provided during construction.
     */
    void startLogging();

    /**
     * @brief Disable logging of Pico2 samples.
     */
    void stopLogging();

private:
    /**
     * @brief Background polling loop running at ~120 Hz.
     *
     * Communicates with the Pico2 over I2C to read IMU and encoder data,
     * updates status flags, pushes samples into the ring buffer, and
     * notifies threads waiting for new data.
     *
     * Runs until shutdown() is called.
     */
    void pollingLoop();

    I2cMaster master_;

    std::atomic<bool> running_ = false;
    std::thread pollingThread_;

    mutable std::mutex dataMutex_;
    std::condition_variable dataUpdated_;

    pico_i2c_mem_addr::StatusFlags status_{};

    RingBuffer<TimedPico2Data> dataBuffer_{120};

    Logger *logger_ = nullptr;
    bool logging_ = false;
};
