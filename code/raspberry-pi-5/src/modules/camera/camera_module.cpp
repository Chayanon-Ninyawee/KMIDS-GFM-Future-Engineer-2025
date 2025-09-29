#include "camera_module.h"

CameraModule::CameraModule(CameraOptionCallback callback)
    : logger_(nullptr) {
    callback(cam_);
}

CameraModule::CameraModule(Logger *logger, CameraOptionCallback callback)
    : logger_(logger) {
    callback(cam_);
}

CameraModule::~CameraModule() {
    if (running_) stop();
}

void CameraModule::changeSetting(CameraOptionCallback callback) {
    if (running_) stop();
    callback(cam_);

    std::cout << "cam.options->awb_gain_r = " << cam_.options->awb_gain_r << ";\n"
              << "cam.options->awb_gain_b = " << cam_.options->awb_gain_b << ";\n\n"
              << "cam.options->brightness = " << cam_.options->brightness << ";\n"
              << "cam.options->sharpness = " << cam_.options->sharpness << ";\n"
              << "cam.options->saturation = " << cam_.options->saturation << ";\n"
              << "cam.options->contrast = " << cam_.options->contrast << ";\n"
              << "cam.options->gain = " << cam_.options->gain << ";\n"
              << std::endl;

    start();
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

bool CameraModule::getFrame(TimedFrame &outTimedFrame) const {
    std::lock_guard<std::mutex> lock(frameMutex_);
    if (frameBuffer_.empty()) return false;

    outTimedFrame = frameBuffer_.latest().value();
    return true;
}

size_t CameraModule::bufferSize() const {
    std::lock_guard<std::mutex> lock(frameMutex_);
    return frameBuffer_.size();
}

bool CameraModule::getAllTimedFrame(std::vector<TimedFrame> &outTimedFrames) const {
    std::lock_guard<std::mutex> lock(frameMutex_);

    if (frameBuffer_.empty()) return false;

    outTimedFrames = frameBuffer_.getAll();
    return true;
}

bool CameraModule::waitForFrame(TimedFrame &outTimedFrame) {
    std::unique_lock<std::mutex> lock(frameMutex_);
    frameUpdated_.wait(lock, [&] { return !frameBuffer_.empty(); });

    outTimedFrame = frameBuffer_.latest().value();
    return true;
}

void CameraModule::captureLoop() {
    while (running_) {
        cv::Mat frame;
        if (!cam_.getVideoFrame(frame, 1000)) {
            std::cerr << "[CameraModule] Timeout error" << std::endl;
            continue;
        }

        cv::rotate(frame, frame, cv::ROTATE_180);

        TimedFrame timedFrame{std::move(frame), std::chrono::steady_clock::now()};

        if (logger_ and logging_) {
            std::vector<uchar> buffer;
            cv::imencode(".png", timedFrame.frame, buffer);

            uint64_t ts = std::chrono::duration_cast<std::chrono::nanoseconds>(timedFrame.timestamp.time_since_epoch()).count();
            logger_->writeData(ts, buffer.data(), buffer.size());
        }

        {
            std::lock_guard<std::mutex> lock(frameMutex_);
            frameBuffer_.push(std::move(timedFrame));
        }

        frameUpdated_.notify_all();
    }
}

void CameraModule::startLogging() {
    logging_ = true;
}

void CameraModule::stopLogging() {
    logging_ = false;
}
