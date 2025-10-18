#include "lidar_module.h"
#include "lidar_processor.h"

#include <chrono>
#include <csignal>

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

    LidarModule lidar;

    TimedLidarData timedLidarData;
    std::chrono::steady_clock::time_point latestTimestamp;

    std::vector<TimedLidarData> timedLidarDatas;

    if (!lidar.initialize()) return -1;
    lidar.printDeviceInfo();
    if (!lidar.start()) return -1;

    cv::namedWindow("Lidar View", cv::WINDOW_FULLSCREEN);

    std::thread(inputThread).detach();

    std::optional<RotationDirection> robotTurnDirection;

    while (!stop_flag) {
        if (!paused) {
            if (lidar.getData(timedLidarData)) {
                if (timedLidarData.timestamp != latestTimestamp) {
                    latestTimestamp = timedLidarData.timestamp;

                    // LidarModule::printScanData(timedLidarData.lidarData);

                    // auto filteredLidarData = lidar_processor::filterLidarData(timedLidarData);
                    auto filteredLidarData = timedLidarData;

                    float heading = 0.0f;

                    auto lineSegments =
                        lidar_processor::getLines(filteredLidarData, {0.0f, 0.0f, 0.0f}, 0.05f, 10, 0.10f, 0.10f, 18.0f, 0.20f);
                    auto relativeWalls =
                        lidar_processor::getRelativeWalls(lineSegments, Direction::fromHeading(heading), heading, 0.30f, 25.0f, 0.22f);

                    auto newRobotTurnDirecton = lidar_processor::getTurnDirection(relativeWalls);
                    if (newRobotTurnDirecton) robotTurnDirection = newRobotTurnDirecton;

                    auto resolveWalls = lidar_processor::resolveWalls(relativeWalls);

                    auto lineSegmentsForParking =
                        lidar_processor::getLines(filteredLidarData, {0.0f, 0.0f, 0.0f}, 0.02f, 10, 0.10f, 0.10f, 18.0f, 0.20f);

                    auto parkingWalls =
                        lidar_processor::getParkingWalls(lineSegmentsForParking, Direction::fromHeading(heading), heading, 0.30f);
                    auto trafficLightPoints =
                        lidar_processor::getTrafficLightPoints(filteredLidarData, resolveWalls, {0.0f, 0.0f, 0.0f}, robotTurnDirection);

                    const float SCALE = 6.0f;

                    cv::Mat lidarMat(800, 800, CV_8UC3, cv::Scalar(0, 0, 0));
                    lidar_processor::drawLidarData(lidarMat, timedLidarData, SCALE);

                    // for (size_t i = 0; i < lineSegments.size(); ++i) {
                    //     const auto &lineSegment = lineSegments[i];

                    //     std::srand(static_cast<unsigned int>(i));
                    //     cv::Scalar color(rand() % 256, rand() % 256, rand() % 256);

                    //     // lidar_processor::drawLineSegment(lidarMat, lineSegment, SCALE);
                    //     lidar_processor::drawLineSegment(lidarMat, lineSegment, SCALE, color);
                    // }

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

                    if (robotTurnDirection) {
                        if (*robotTurnDirection == RotationDirection::CLOCKWISE) {
                            std::cout << "CLOCKWISE" << std::endl;
                        } else if (*robotTurnDirection == RotationDirection::COUNTER_CLOCKWISE) {
                            std::cout << "COUNTER_CLOCKWISE" << std::endl;
                        }
                    } else {
                        std::cout << "N/A" << std::endl;
                    }

                    cv::imshow("Lidar View", lidarMat);
                }
            }

            // if (lidar.getAllTimedLidarData(timedLidarDatas) && lidar.bufferSize() > 9) {
            //     LidarModule::printScanData(timedLidarDatas[0].lidarData);

            //     auto duration = timedLidarDatas[9].timestamp - timedLidarDatas[0].timestamp;
            //     std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() << " ms" << std::endl;
            // }
        }

        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
        cv::waitKey(10);
    }

    lidar.stop();
    lidar.shutdown();

    cv::destroyAllWindows();
}
