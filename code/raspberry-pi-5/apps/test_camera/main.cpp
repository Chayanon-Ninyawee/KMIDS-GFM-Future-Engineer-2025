#include "camera_module.h"
#include "camera_processor.h"
#include "libcamera/control_ids.h"

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

CameraModule *globalCamera = nullptr;

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
        } else {
            // Example: set brightness 0.5
            std::istringstream iss(command);
            std::string cmd, name;
            float value;
            iss >> cmd >> name >> value;
            if (cmd == "set" && !name.empty()) {
                if (globalCamera) {
                    globalCamera->changeSetting([=](lccv::PiCamera &cam) {
                        if (name == "awb_r")
                            cam.options->awb_gain_r = value;
                        else if (name == "awb_b")
                            cam.options->awb_gain_b = value;
                        else if (name == "brightness")
                            cam.options->brightness = value;
                        else if (name == "sharpness")
                            cam.options->sharpness = value;
                        else if (name == "saturation")
                            cam.options->saturation = value;
                        else if (name == "contrast")
                            cam.options->contrast = value;
                        else if (name == "gain")
                            cam.options->gain = value;
                        else if (name == "shutter")
                            cam.options->shutter = value;
                        else
                            std::cout << "Unknown setting: " << name << "\n";
                    });
                    std::cout << "Updated " << name << " to " << value << "\n";
                }
            }
        }
    }
}

std::vector<cv::Point> polygonPoints;
bool selectMode = false;

// Mouse callback
void mouseCallback(int event, int x, int y, int, void *) {
    if (!selectMode) return;
    if (event == cv::EVENT_LBUTTONDOWN) {
        polygonPoints.emplace_back(x, y);
    }
}

// Compute two HSV bounds for pixels inside polygon
void computeHSVBounds(const cv::Mat &frame, cv::Scalar &lower1, cv::Scalar &upper1, cv::Scalar &lower2, cv::Scalar &upper2) {
    if (polygonPoints.empty()) return;

    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    // Polygon mask
    cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8UC1);
    std::vector<std::vector<cv::Point>> pts{polygonPoints};
    cv::fillPoly(mask, pts, cv::Scalar(255));

    std::vector<int> Hs, Ss, Vs;
    for (int y = 0; y < frame.rows; ++y) {
        for (int x = 0; x < frame.cols; ++x) {
            if (mask.at<uchar>(y, x)) {
                cv::Vec3b pixel = hsv.at<cv::Vec3b>(y, x);
                if (pixel[2] <= 1) continue;

                Hs.push_back(pixel[0]);
                Ss.push_back(pixel[1]);
                Vs.push_back(pixel[2]);
            }
        }
    }

    if (Hs.empty()) return;

    // Saturation and Value
    int minS = *std::min_element(Ss.begin(), Ss.end());
    int maxS = *std::max_element(Ss.begin(), Ss.end());
    int minV = *std::min_element(Vs.begin(), Vs.end());
    int maxV = *std::max_element(Vs.begin(), Vs.end());

    // Hue wrap-around
    int minH = *std::min_element(Hs.begin(), Hs.end());
    int maxH = *std::max_element(Hs.begin(), Hs.end());

    int directSpan = maxH - minH;
    int wrapSpan = (minH + 180) - maxH;

    if (wrapSpan < directSpan) {
        // Wrap-around case → two ranges
        lower1 = cv::Scalar(0, minS, minV);
        upper1 = cv::Scalar(minH, maxS, maxV);
        lower2 = cv::Scalar(maxH, minS, minV);
        upper2 = cv::Scalar(180, maxS, maxV);
    } else {
        // Normal case → single range, second unused
        lower1 = cv::Scalar(minH, minS, minV);
        upper1 = cv::Scalar(maxH, maxS, maxV);
        lower2 = cv::Scalar(maxH, maxS, maxV);
        upper2 = cv::Scalar(maxH, maxS, maxV);
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
        cam.options->awb_gain_b = 1.5;

        cam.options->brightness = 0.1;
        cam.options->sharpness = 1;
        cam.options->saturation = 1.5;
        cam.options->contrast = 1;
        cam.options->gain = 5;
        cam.options->shutter = 30000;
    };

    CameraModule camera(cameraOptionCallback);

    TimedFrame timedFrame;
    std::vector<TimedFrame> timedFrames;

    bool isDisplayingMask = false;
    cv::Mat displayingFrame;

    cv::namedWindow("Video", cv::WINDOW_FULLSCREEN);
    cv::setMouseCallback("Video", mouseCallback);

    cv::namedWindow("Delayed Video", cv::WINDOW_FULLSCREEN);

    camera.start();
    globalCamera = &camera;  // so inputThread can access

    std::thread(inputThread).detach();

    while (!stop_flag) {
        if (!paused) {
            if (camera.getFrame(timedFrame)) {
                if (not isDisplayingMask) {
                    displayingFrame = timedFrame.frame;
                } else {
                    auto colorMasks = camera_processor::filterColors(timedFrame);

                    displayingFrame = cv::Mat::zeros(timedFrame.frame.size(), timedFrame.frame.type());
                    // if (!colorMasks.green.mask.empty()) {
                    //     // Copy the pixels from the original image where mask is true
                    //     timedFrame.frame.copyTo(displayingFrame, colorMasks.green.mask);
                    // }
                    // if (!colorMasks.red.mask.empty()) {
                    //     // Copy the pixels from the original image where mask is true
                    //     timedFrame.frame.copyTo(displayingFrame, colorMasks.red.mask);
                    // }

                    camera_processor::drawColorMasks(displayingFrame, colorMasks);
                }

                cv::imshow("Video", displayingFrame);
            }

            if (camera.getAllTimedFrame(timedFrames) && camera.bufferSize() > 29) {
                // cv::imshow("Delayed Video", timedFrames[0].frame);

                // auto duration = timedFrames[29].timestamp - timedFrames[0].timestamp;
                // std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() << " ms" << std::endl;
            }
        }

        int key = cv::waitKey(15);
        if (key == 'm') {
            isDisplayingMask = not isDisplayingMask;
        } else if (key == 'i') {
            selectMode = true;
            polygonPoints.clear();
            std::cout << "Click points to define polygon, then press 'c' to confirm.\n";

            paused = true;
        } else if (key == 'c') {
            selectMode = false;
            cv::Scalar lower1, upper1, lower2, upper2;
            computeHSVBounds(displayingFrame, lower1, upper1, lower2, upper2);
            std::cout << "Lower1 HSV: " << lower1 << "\n";
            std::cout << "Upper1 HSV: " << upper1 << "\n";
            std::cout << "Lower2 HSV: " << lower2 << "\n";
            std::cout << "Upper2 HSV: " << upper2 << "\n";

            paused = false;
        }
    }

    camera.stop();

    cv::destroyAllWindows();
}
