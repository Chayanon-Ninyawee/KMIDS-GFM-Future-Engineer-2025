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
     * @brief Start capturing frames in a separate thread.
     */
    bool start();

    /**
     * @brief Stop capturing frames and wait for the thread to finish.
     */
    void stop();

    /**
     * @brief Get the latest captured frame and timestamp.
     *
     * This is thread-safe.
     *
     * FIXME:
     * @param[out] outFrame The frame image.
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
     * @brief Waits until a new frame is available, then returns it.
     *
     * This function blocks the calling thread until a new frame is captured.
     * Once available, it copies the latest frame and its timestamp into the output parameters.
     *
     * FIXME:
     * @param[out] outFrame The most recently captured frame.
     * @param[out] outTimestamp The timestamp when the frame was captured.
     *
     * @return true if a frame was successfully retrieved.
     */
    bool waitForFrame(TimedFrame &outTimedFrame);

    void startLogging();

    void stopLogging();

private:
    /**
     * @brief The function running in the background thread.
     *
     * Continuously captures frames from the camera.
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
