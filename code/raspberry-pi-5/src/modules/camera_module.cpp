#include "camera_module.h"
#include <iostream>

CameraModule::CameraModule(CameraOptionCallback callback)
    : running_(false) {
    callback(cam_);
}

CameraModule::~CameraModule() {
    if (running_) stop();
}

bool CameraModule::start() {
    if (running_) {
        std::cout << "[CameraModule] Already started." << std::endl;
        return false;
    }

    if (not cam_.startVideo()) {
        return false;
    }

    running_ = true;

    cameraThread_ = std::thread(&CameraModule::captureLoop, this);

    return true;
}

void CameraModule::stop() {
    if (!running_) {
        std::cout << "[CameraModule] Not running." << std::endl;
        return;
    }

    running_ = false;
    if (cameraThread_.joinable()) {
        cameraThread_.join();
    }

    cam_.stopVideo();
}

bool CameraModule::getFrame(cv::Mat &outFrame, std::chrono::steady_clock::time_point &outTimestamp) {
    std::lock_guard<std::mutex> lock(frameMutex_);
    if (latestFrame_.empty()) return false;

    outFrame = latestFrame_.clone();
    outTimestamp = latestTimestamp_;
    return true;
}

bool CameraModule::waitForFrame(cv::Mat &outFrame, std::chrono::steady_clock::time_point &outTimestamp) {
    std::unique_lock<std::mutex> lock(frameMutex_);
    frameUpdated_.wait(lock, [&] { return !latestFrame_.empty(); });

    outFrame = latestFrame_.clone();
    outTimestamp = latestTimestamp_;
    return true;
}

void CameraModule::captureLoop() {
    while (running_) {
        cv::Mat frame;
        if (!cam_.getVideoFrame(frame, 1000)) {
            std::cerr << "Timeout error" << std::endl;
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(frameMutex_);
            latestFrame_ = frame;
            latestTimestamp_ = std::chrono::steady_clock::now();
        }

        frameUpdated_.notify_all();
    }
}
