#include "camera_module.h"

#include <csignal>
#include <iostream>

namespace controls = libcamera::controls;

const uint32_t camWidth = 1296;
const uint32_t camHeight = 972;

volatile std::sig_atomic_t stop_flag = 0;

void signalHandler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received.\n";
    stop_flag = 1;
}

std::atomic<bool> paused(false);

void inputThread() {
    std::string command;
    while (true) {
        std::getline(std::cin, command);
        if (command == "p") {
            paused = true;
            std::cout << "Paused\n";
        } else if (command == "r") {
            paused = false;
            std::cout << "Resumed\n";
        } else if (command == "quit") {
            stop_flag = true;
            break;
        }
    }
}

int main() {
    std::signal(SIGINT, signalHandler);

    auto cameraOptionCallback = [](lccv::PiCamera &cam) {
        libcamera::ControlList &camControls = cam.getControlList();

        cam.options->video_width = camWidth;
        cam.options->video_height = camHeight;
        cam.options->framerate = 30.0f;

        camControls.set(controls::AnalogueGainMode, controls::AnalogueGainModeEnum::AnalogueGainModeManual);
        camControls.set(controls::ExposureTimeMode, controls::ExposureTimeModeEnum::ExposureTimeModeManual);
        camControls.set(controls::AwbEnable, false);

        cam.options->awb_gain_r = 0.90;
        cam.options->awb_gain_b = 1.25;

        cam.options->brightness = 0.0;
        cam.options->sharpness = 1.0;
        cam.options->saturation = 1.5;
        cam.options->contrast = 1.0;
        cam.options->shutter = 10000;
        cam.options->gain = 2.0;  // If can't seperate red and pink try changing gain
    };

    CameraModule camera(cameraOptionCallback);

    cv::Mat camImage;
    std::chrono::steady_clock::time_point latestFrameTimestamp;

    cv::namedWindow("Video", cv::WINDOW_FULLSCREEN);

    camera.start();

    std::thread(inputThread).detach();

    while (!stop_flag) {
        if (!paused) {
            if (camera.getFrame(camImage, latestFrameTimestamp)) {
                cv::imshow("Video", camImage);
            }
        }

        cv::waitKey(10);
    }

    camera.stop();

    cv::destroyAllWindows();
}
