#pragma once

#include "lccv.hpp"

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <thread>

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
     * @brief Destroy the camera module.
     *
     * Stops capturing and cleans up resources.
     */
    ~CameraModule();

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
     * @param[out] outFrame The frame image.
     * @param[out] outTimestamp The time when the frame was captured.
     *
     * @return true if a frame is available, false otherwise.
     */
    bool getFrame(cv::Mat &outFrame, std::chrono::steady_clock::time_point &outTimestamp);

    /**
     * @brief Waits until a new frame is available, then returns it.
     *
     * This function blocks the calling thread until a new frame is captured.
     * Once available, it copies the latest frame and its timestamp into the output parameters.
     *
     * @param[out] outFrame The most recently captured frame.
     * @param[out] outTimestamp The timestamp when the frame was captured.
     *
     * @return true if a frame was successfully retrieved.
     */
    bool waitForFrame(cv::Mat &outFrame, std::chrono::steady_clock::time_point &outTimestamp);

private:
    /**
     * @brief The function running in the background thread.
     *
     * Continuously captures frames from the camera.
     */
    void captureLoop();

    lccv::PiCamera cam_;

    std::thread cameraThread_;
    std::atomic<bool> running_;

    std::mutex frameMutex_;
    std::condition_variable frameUpdated_;

    cv::Mat latestFrame_;
    std::chrono::steady_clock::time_point latestTimestamp_;
};
