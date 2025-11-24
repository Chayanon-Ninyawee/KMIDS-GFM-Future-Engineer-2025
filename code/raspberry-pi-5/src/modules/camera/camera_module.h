#pragma once

#include "lccv.hpp"
#include <atomic>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <thread>

#include "camera_struct.h"
#include "logger.h"
#include "ring_buffer.hpp"

/**
 * @brief Camera module that captures frames in a background thread.
 *
 * Uses lccv::PiCamera to get frames from the camera device.
 * Runs a thread that keeps capturing frames, storing the latest one.
 * Provides thread-safe access to the latest frame and timestamp.
 */
class CameraModule
{
public:
    /**
     * @brief Type of callback used to configure the camera before starting capture.
     *
     * The callback receives a reference to the internal lccv::PiCamera instance,
     * allowing the user to modify camera options like resolution, format, etc.
     */
    using CameraOptionCallback = std::function<void(lccv::PiCamera &)>;

    /**
     * @brief Create the camera module.
     *
     * Sets up internal state but does not start capturing.
     */
    CameraModule(CameraOptionCallback callback);

    /**
     * @brief Create the camera module with logging support.
     *
     * Initializes the camera module and stores the provided logger instance.
     * Capture does not begin until start() is called.
     *
     * @param logger Pointer to a Logger instance for optional frame logging.
     * @param callback Callback used to configure the internal lccv::PiCamera
     *        before capture starts.
     */
    CameraModule(Logger *logger, CameraOptionCallback callback);

    /**
     * @brief Destroy the camera module.
     *
     * Stops capturing and cleans up resources.
     */
    ~CameraModule();

    /**
     * @brief Change camera settings at runtime.
     *
     * Applies a new configuration to the internal lccv::PiCamera instance
     * by invoking the provided callback. This allows adjusting camera
     * options such as resolution, format, or exposure without recreating
     * the CameraModule.
     *
     * @param callback Function that receives a reference to the internal
     *        lccv::PiCamera and modifies its settings.
     */
    void changeSetting(CameraOptionCallback callback);

    /**
     * @brief Start capturing frames in a background thread.
     *
     * Initializes the camera if needed, applies the configuration callback,
     * and launches the capture loop in a dedicated thread.
     *
     * @return true if the camera started successfully, false on failure.
     */
    bool start();

    /**
     * @brief Stop capturing frames and wait for the capture thread to exit.
     *
     * Safe to call even if the camera is not currently running.
     * Cleans up internal resources associated with frame capture.
     */
    void stop();

    /**
     * @brief Get the latest captured frame and its timestamp.
     *
     * Thread-safe retrieval of the most recent frame stored in the internal buffer.
     *
     * @param[out] outTimedFrame A structure containing the frame image and the
     *             timestamp at which it was captured.
     *
     * @return true if a frame is available, false otherwise.
     */
    bool getFrame(TimedFrame &outTimedFrame) const;

    /**
     * @brief Get the current number of frames stored in the buffer.
     *
     * This function is thread-safe and returns how many frames are
     * currently stored in the internal RingBuffer.
     *
     * @return The number of frames currently in the buffer.
     */
    size_t bufferSize() const;

    /**
     * @brief Retrieve all frames currently stored in the buffer along with their timestamps.
     *
     * This function is thread-safe. The frames are returned in order
     * from oldest to newest.
     *
     * @param[out] outTimedFrames Vector to be filled with all frames and their timestamps.
     * @return true if the buffer contains at least one frame, false if empty.
     */
    bool getAllTimedFrame(std::vector<TimedFrame> &outTimedFrames) const;

    /**
     * @brief Block until a new frame is available, then return it.
     *
     * This call blocks the caller until the camera thread captures a frame more
     * recent than the previously retrieved one. Once available, the frame and its
     * timestamp are copied into the output structure.
     *
     * @param[out] outTimedFrame A structure containing the newly captured frame and
     *             its timestamp.
     *
     * @return true if a new frame was retrieved successfully, false otherwise.
     */
    bool waitForFrame(TimedFrame &outTimedFrame);

    /**
     * @brief Enable frame logging.
     *
     * When logging is enabled, captured frames (and/or metadata) are passed
     * to the Logger provided during construction, if any.
     */
    void startLogging();

    /**
     * @brief Disable frame logging.
     *
     * Stops sending captured frame information to the Logger.
     */
    void stopLogging();

private:
    /**
     * @brief Background thread function responsible for continuous capture.
     *
     * Continuously grabs frames from the lccv::PiCamera, stores them in the
     * internal ring buffer, signals waiting threads, and optionally logs data.
     *
     * This function runs until stop() is called.
     */
    void captureLoop();

    lccv::PiCamera cam_;

    std::thread cameraThread_;
    std::atomic<bool> running_ = false;

    mutable std::mutex frameMutex_;
    std::condition_variable frameUpdated_;

    RingBuffer<TimedFrame> frameBuffer_{30};

    Logger *logger_;
    bool logging_ = false;
};
