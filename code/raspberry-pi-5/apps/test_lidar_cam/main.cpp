#include "camera_module.h"
#include "camera_processor.h"
#include "combined_processor.h"
#include "lidar_module.h"
#include "lidar_processor.h"

#include <chrono>
#include <csignal>

namespace controls = libcamera::controls;

const uint32_t camWidth = 1296;
const uint32_t camHeight = 972;
const float camHFov = 104.0f;

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

void drawLineFromAngle(cv::Mat &img, int cx, int cy, float angle, const cv::Scalar &color, int thickness = 2) {
    // Extend line length far beyond image size
    float length = std::max(img.cols, img.rows) * 2.0f;

    // Compute end point
    int x2 = static_cast<int>(cx + length * std::sin(angle * M_PI / 180.0f));
    int y2 = static_cast<int>(cy - length * std::cos(angle * M_PI / 180.0f));  // y-axis inverted in images

    // Draw line
    cv::line(img, cv::Point(cx, cy), cv::Point(x2, y2), color, thickness);
}

int main() {
    std::signal(SIGINT, signalHandler);

    LidarModule lidar;

    if (!lidar.initialize()) return -1;
    lidar.printDeviceInfo();
    if (!lidar.start()) return -1;

    auto cameraOptionCallback = [](lccv::PiCamera &cam) {
        libcamera::ControlList &camControls = cam.getControlList();

        cam.options->video_width = camWidth;
        cam.options->video_height = camHeight;
        cam.options->framerate = 30.0f;

        camControls.set(controls::AnalogueGainMode, controls::AnalogueGainModeEnum::AnalogueGainModeManual);
        camControls.set(controls::ExposureTimeMode, controls::ExposureTimeModeEnum::ExposureTimeModeAuto);
        camControls.set(controls::AwbEnable, false);

        cam.options->awb_gain_r = 0.83;
        cam.options->awb_gain_b = 1.5;

        cam.options->brightness = 0.1;
        cam.options->sharpness = 1;
        cam.options->saturation = 1.5;
        cam.options->contrast = 1;
        cam.options->gain = 5;
    };
    CameraModule camera(cameraOptionCallback);

    camera.start();

    cv::namedWindow("Lidar View", cv::WINDOW_FULLSCREEN);
    cv::namedWindow("Video", cv::WINDOW_FULLSCREEN);

    std::thread(inputThread).detach();

    std::optional<RotationDirection> robotTurnDirection;

    while (!stop_flag) {
        cv::waitKey(10);

        if (!paused) {
            std::vector<TimedLidarData> timedLidarDatas;
            if (not lidar.getAllTimedLidarData(timedLidarDatas)) continue;

            std::vector<TimedFrame> timedFrames;
            if (not camera.getAllTimedFrame(timedFrames)) continue;

            auto &timedLidarData = timedLidarDatas.back();
            auto &timedFrame = timedFrames.back();

            auto filteredLidarData = lidar_processor::filterLidarData(timedLidarData);

            float heading = 0.0f;

            auto lineSegments = lidar_processor::getLines(filteredLidarData, {0.0f, 0.0f, 0.0f}, 0.05f, 10, 0.10f, 0.10f, 18.0f, 0.20f);
            auto relativeWalls =
                lidar_processor::getRelativeWalls(lineSegments, Direction::fromHeading(heading), heading, 0.30f, 25.0f, 0.22f);

            auto newRobotTurnDirecton = lidar_processor::getTurnDirection(relativeWalls);
            if (newRobotTurnDirecton) robotTurnDirection = newRobotTurnDirecton;

            auto resolveWalls = lidar_processor::resolveWalls(relativeWalls);
            auto parkingWalls = lidar_processor::getParkingWalls(lineSegments, Direction::fromHeading(heading), heading, 0.25f);
            auto trafficLightPoints = lidar_processor::getTrafficLightPoints(filteredLidarData, resolveWalls, robotTurnDirection);

            auto colorMasks = camera_processor::filterColors(timedFrame);
            auto blockAngles = camera_processor::computeBlockAngles(colorMasks, camWidth, camHFov);

            auto trafficLightInfos = combined_processor::combineTrafficLightInfo(blockAngles, trafficLightPoints, {0.0f, 0.0f, 0.0f});

            const float SCALE = 6.0f;

            cv::Mat lidarMat(800, 800, CV_8UC3, cv::Scalar(0, 0, 0));
            lidar_processor::drawLidarData(lidarMat, timedLidarData, SCALE);

            if (resolveWalls.leftWall) {
                cv::Scalar color(0, 0, 255);
                lidar_processor::drawLineSegment(lidarMat, *resolveWalls.leftWall, SCALE, color);
            }
            if (resolveWalls.rightWall) {
                cv::Scalar color(0, 255, 255);
                lidar_processor::drawLineSegment(lidarMat, *resolveWalls.rightWall, SCALE, color);
            }
            if (resolveWalls.frontWall) {
                cv::Scalar color(0, 255, 0);
                lidar_processor::drawLineSegment(lidarMat, *resolveWalls.frontWall, SCALE, color);
            }
            if (resolveWalls.backWall) {
                cv::Scalar color(255, 255, 0);
                lidar_processor::drawLineSegment(lidarMat, *resolveWalls.backWall, SCALE, color);
            }

            if (resolveWalls.farLeftWall) {
                cv::Scalar color(0, 0, 100);
                lidar_processor::drawLineSegment(lidarMat, *resolveWalls.farLeftWall, SCALE, color);
            }
            if (resolveWalls.farRightWall) {
                cv::Scalar color(0, 100, 100);
                lidar_processor::drawLineSegment(lidarMat, *resolveWalls.farRightWall, SCALE, color);
            }

            for (auto &parkingWall : parkingWalls) {
                cv::Scalar color(146, 22, 199);
                lidar_processor::drawLineSegment(lidarMat, parkingWall, SCALE, color);
            }

            for (auto &trafficLightPoint : trafficLightPoints) {
                lidar_processor::drawTrafficLightPoint(lidarMat, trafficLightPoint, SCALE);
            }

            for (auto &trafficLightInfo : trafficLightInfos) {
                combined_processor::drawTrafficLightInfo(lidarMat, trafficLightInfo, SCALE);
            }

            cv::Mat cameraMat = cv::Mat::zeros(timedFrame.frame.size(), timedFrame.frame.type());
            camera_processor::drawColorMasks(cameraMat, colorMasks);

            if (robotTurnDirection) {
                if (*robotTurnDirection == RotationDirection::CLOCKWISE) {
                    std::cout << "CLOCKWISE" << std::endl;
                } else if (*robotTurnDirection == RotationDirection::COUNTER_CLOCKWISE) {
                    std::cout << "COUNTER_CLOCKWISE" << std::endl;
                }
            } else {
                std::cout << "N/A" << std::endl;
            }

            for (const auto &block : blockAngles) {
                // Choose color for drawing
                cv::Scalar lineColor = (block.color == camera_processor::Color::RED) ? cv::Scalar(0, 0, 255)   // Red in BGR
                                                                                     : cv::Scalar(0, 255, 0);  // Green in BGR

                // Draw a line from the centroid along the angle
                drawLineFromAngle(lidarMat, 400, 400 - (800 / 6.0f * 0.15), block.angle, lineColor, 2);

                std::cout << "Angle: " << block.angle << "\n";
            }

            cv::imshow("Lidar View", lidarMat);
            cv::imshow("Video", cameraMat);
        }
    }

    lidar.stop();
    lidar.shutdown();

    camera.stop();

    cv::destroyAllWindows();
}
