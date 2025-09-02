#include <cstring>
#include <filesystem>
#include <iostream>
#include <vector>

#include "camera_processor.h"
#include "camera_struct.h"
#include "combined_processor.h"
#include "lidar_processor.h"
#include "lidar_struct.h"
#include "log_reader.h"
#include "pico2_struct.h"

namespace fs = std::filesystem;

const uint32_t camWidth = 1296;
const uint32_t camHeight = 972;
const float camHFov = 104.0f;

TimedLidarData reconstructTimedLidar(const LogEntry &entry) {
    std::vector<RawLidarNode> nodes(entry.data.size() / sizeof(RawLidarNode));
    if (!entry.data.empty()) {
        std::memcpy(nodes.data(), entry.data.data(), entry.data.size());
    }

    // Convert timestamp in nanoseconds back to steady_clock::time_point
    auto ts = std::chrono::nanoseconds(entry.timestamp);
    std::chrono::steady_clock::time_point timestamp(ts);

    return TimedLidarData{std::move(nodes), timestamp};
}

// FIXME: Temp code
std::vector<TimedPico2Data> mergePico2Entries(const std::vector<LogEntry> &pico2Entries) {
    std::vector<TimedPico2Data> timedPico2Datas;

    if (pico2Entries.size() % 3 != 0) {
        throw std::runtime_error("Pico2 log size is not a multiple of 3");
    }

    for (size_t i = 0; i + 2 < pico2Entries.size(); i += 3) {
        const auto &accelEntry = pico2Entries[i];
        const auto &eulerEntry = pico2Entries[i + 1];
        const auto &encoderEntry = pico2Entries[i + 2];

        // Check that timestamps match
        if (accelEntry.timestamp != eulerEntry.timestamp || accelEntry.timestamp != encoderEntry.timestamp) {
            throw std::runtime_error("Mismatched timestamps in Pico2 log");
        }

        TimedPico2Data sample{};
        std::memcpy(&sample.accel, accelEntry.data.data(), sizeof(ImuAccel));
        std::memcpy(&sample.euler, eulerEntry.data.data(), sizeof(ImuEuler));
        std::memcpy(&sample.encoderAngle, encoderEntry.data.data(), sizeof(double));
        sample.timestamp = std::chrono::steady_clock::time_point(std::chrono::nanoseconds(accelEntry.timestamp));

        timedPico2Datas.push_back(sample);
    }

    return timedPico2Datas;
}

// FIXME: This work for this log since i wrote the code in the pico2_module incorrectly
TimedPico2Data reconstructTimedPico2(const LogEntry &entry1, const LogEntry &entry2, const LogEntry &entry3) {
    TimedPico2Data pico2Data{};

    // Accel
    std::memcpy(&pico2Data.accel, entry1.data.data(), sizeof(ImuAccel));

    // Euler
    std::memcpy(&pico2Data.euler, entry2.data.data(), sizeof(ImuEuler));

    // Encoder angle
    std::memcpy(&pico2Data.encoderAngle, entry3.data.data(), sizeof(double));

    // Timestamp
    auto ts = std::chrono::nanoseconds(entry1.timestamp);
    pico2Data.timestamp = std::chrono::steady_clock::time_point(ts);

    return pico2Data;
}

TimedFrame reconstructTimedFrame(const LogEntry &entry) {
    if (entry.data.empty()) {
        throw std::runtime_error("Empty image entry data");
    }

    // Decode image from memory
    cv::Mat frame = cv::imdecode(entry.data, cv::IMREAD_UNCHANGED);
    if (frame.empty()) {
        throw std::runtime_error("Failed to decode image from entry data");
    }

    // Convert timestamp in nanoseconds back to steady_clock::time_point
    auto ts = std::chrono::nanoseconds(entry.timestamp);
    std::chrono::steady_clock::time_point timestamp(ts);

    return TimedFrame{std::move(frame), timestamp};
}

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <log_folder>" << std::endl;
        return 1;
    }

    std::string folderPath = argv[1];

    // Build file paths
    std::string lidarLogFile = (fs::path(folderPath) / "lidar.bin").string();
    std::string pico2File = (fs::path(folderPath) / "pico2.bin").string();
    std::string cameraLogFile = (fs::path(folderPath) / "camera.bin").string();

    // ---- Lidar ----
    LogReader lidarReader(lidarLogFile);
    std::vector<LogEntry> lidarEntries;
    if (!lidarReader.readAll(lidarEntries)) {
        std::cerr << "Failed to read Lidar log file: " << lidarLogFile << std::endl;
        return 1;
    }
    std::cout << "Loaded " << lidarEntries.size() << " Lidar log entries." << std::endl;

    // ---- Pico2 ----
    LogReader pico2Reader(pico2File);
    std::vector<LogEntry> pico2Entries;
    if (!pico2Reader.readAll(pico2Entries)) {
        std::cerr << "Failed to read Pico2 log file: " << pico2File << std::endl;
        return 1;
    }
    std::cout << "Loaded " << pico2Entries.size() << " Pico2 log entries." << std::endl;

    std::vector<LogEntry> cameraEntries;  // declare here, always in scope
    bool hasCamera = false;

    // ---- Camera (optional) ----
    if (fs::exists(cameraLogFile)) {
        LogReader cameraReader(cameraLogFile);
        if (!cameraReader.readAll(cameraEntries)) {
            std::cerr << "Failed to read Camera log file: " << cameraLogFile << std::endl;
            return 1;
        }
        std::cout << "Loaded " << cameraEntries.size() << " Camera log entries." << std::endl;
        hasCamera = true;
    } else {
        std::cout << "No camera log file found, skipping." << std::endl;
    }

    // Windows: only open Camera View if we have camera data
    cv::namedWindow("Lidar View", cv::WINDOW_FULLSCREEN);
    if (hasCamera) {
        cv::namedWindow("Camera View", cv::WINDOW_FULLSCREEN);
    }

    std::optional<float> initialHeading;
    std::optional<RotationDirection> robotTurnDirection;

    size_t lidarIdx = 0;
    size_t pico2Idx = 0;
    size_t cameraIdx = 0;

    auto findClosestIndex = [](const std::vector<LogEntry> &entries, size_t startIdx, uint64_t targetTs) -> size_t {
        size_t idx = startIdx;

        // Move forward if next entry is closer
        while (idx + 1 < entries.size() && std::abs(static_cast<int64_t>(entries[idx + 1].timestamp - targetTs)) <
                                               std::abs(static_cast<int64_t>(entries[idx].timestamp - targetTs)))
        {
            idx++;
        }

        // Move backward if previous entry is closer
        while (idx > 0 && std::abs(static_cast<int64_t>(entries[idx - 1].timestamp - targetTs)) <
                              std::abs(static_cast<int64_t>(entries[idx].timestamp - targetTs)))
        {
            idx--;
        }

        return idx;
    };

    // FIXME: Temp
    std::vector<TimedPico2Data> timedPico2Datas = mergePico2Entries(pico2Entries);
    auto findClosestIndexPico2 = [](const std::vector<TimedPico2Data> &entries, size_t startIdx, uint64_t targetTs) -> size_t {
        size_t idx = startIdx;

        auto toNs = [](const std::chrono::steady_clock::time_point &tp) -> uint64_t {
            return std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch()).count();
        };

        while (idx + 1 < entries.size() && std::abs(static_cast<int64_t>(toNs(entries[idx + 1].timestamp) - targetTs)) <
                                               std::abs(static_cast<int64_t>(toNs(entries[idx].timestamp) - targetTs)))
        {
            idx++;
        }

        while (idx > 0 && std::abs(static_cast<int64_t>(toNs(entries[idx - 1].timestamp) - targetTs)) <
                              std::abs(static_cast<int64_t>(toNs(entries[idx].timestamp) - targetTs)))
        {
            idx--;
        }

        return idx;
    };

    while (true) {
        int key = cv::waitKey(0);
        if (key == 27) break;  // ESC

        if (key == 81) {  // left
            if (lidarIdx > 0) lidarIdx--;
        } else if (key == 83) {  // right
            if (lidarIdx + 1 < lidarEntries.size()) lidarIdx++;
        } else {
            continue;
        }

        const auto &lidarEntry = lidarEntries[lidarIdx];
        TimedLidarData timedLidarData = reconstructTimedLidar(lidarEntry);
        uint64_t lidarTime = lidarEntry.timestamp;

        // Synchronize Pico2
        // FIXME:
        pico2Idx = findClosestIndexPico2(timedPico2Datas, pico2Idx, lidarTime);
        const auto &timedPico2Data = timedPico2Datas[pico2Idx];

        if (not initialHeading) initialHeading = timedPico2Data.euler.h;

        float heading = timedPico2Data.euler.h - initialHeading.value_or(0.0f);
        heading = std::fmod(heading, 360.0f);
        if (heading < 0.0f) heading += 360.0f;

        // Synchronize Camera (if available)
        TimedFrame timedFrame;
        camera_processor::ColorMasks colorMasks;
        std::vector<camera_processor::BlockAngle> blockAngles;
        if (hasCamera) {
            cameraIdx = findClosestIndex(cameraEntries, cameraIdx, lidarTime);
            if (cameraIdx < cameraEntries.size()) {
                timedFrame = reconstructTimedFrame(cameraEntries[cameraIdx]);
                colorMasks = camera_processor::filterColors(timedFrame);
                blockAngles = camera_processor::computeBlockAngles(colorMasks, camWidth, camHFov);
            }
        }
        // ---- Lidar processing ----

        auto filteredLidarData = lidar_processor::filterLidarData(timedLidarData);

        // auto deltaPose = combined_processor::aproximateRobotPose(filteredLidarData, timedPico2Datas);

        auto lineSegments = lidar_processor::getLines(filteredLidarData, {0.0f, 0.0f, 0.0f}, 0.05f, 10, 0.10f, 0.10f, 18.0f, 0.20f);
        auto relativeWalls = lidar_processor::getRelativeWalls(lineSegments, Direction::fromHeading(heading), heading, 0.30f, 25.0f, 0.22f);

        auto newRobotTurnDirecton = lidar_processor::getTurnDirection(relativeWalls);
        if (newRobotTurnDirecton) robotTurnDirection = newRobotTurnDirecton;

        auto resolveWalls = lidar_processor::resolveWalls(relativeWalls);
        auto parkingWalls = lidar_processor::getParkingWalls(lineSegments, Direction::fromHeading(heading), heading, 0.25f);
        auto trafficLightPoints = lidar_processor::getTrafficLightPoints(filteredLidarData, resolveWalls, robotTurnDirection);

        auto trafficLightInfos = combined_processor::combineTrafficLightInfo(blockAngles, trafficLightPoints);

        const float SCALE = 6.0f;

        cv::Mat lidarMat(800, 800, CV_8UC3, cv::Scalar(0, 0, 0));
        lidar_processor::drawLidarData(lidarMat, timedLidarData, SCALE);

        // Draw walls, parking, traffic lights as before
        if (resolveWalls.leftWall) lidar_processor::drawLineSegment(lidarMat, *resolveWalls.leftWall, SCALE, {0, 0, 255});
        if (resolveWalls.rightWall) lidar_processor::drawLineSegment(lidarMat, *resolveWalls.rightWall, SCALE, {0, 255, 255});
        if (resolveWalls.frontWall) lidar_processor::drawLineSegment(lidarMat, *resolveWalls.frontWall, SCALE, {0, 255, 0});
        if (resolveWalls.backWall) lidar_processor::drawLineSegment(lidarMat, *resolveWalls.backWall, SCALE, {255, 255, 0});
        if (resolveWalls.farLeftWall) lidar_processor::drawLineSegment(lidarMat, *resolveWalls.farLeftWall, SCALE, {0, 0, 100});
        if (resolveWalls.farRightWall) lidar_processor::drawLineSegment(lidarMat, *resolveWalls.farRightWall, SCALE, {0, 100, 100});

        for (auto &parkingWall : parkingWalls)
            lidar_processor::drawLineSegment(lidarMat, parkingWall, SCALE, {146, 22, 199});

        for (auto &trafficLightPoint : trafficLightPoints)
            lidar_processor::drawTrafficLightPoint(lidarMat, trafficLightPoint, SCALE);

        for (auto &trafficLightInfo : trafficLightInfos)
            combined_processor::drawTrafficLightInfo(lidarMat, trafficLightInfo, SCALE);

        if (robotTurnDirection) {
            if (*robotTurnDirection == RotationDirection::CLOCKWISE)
                std::cout << "CLOCKWISE" << std::endl;
            else if (*robotTurnDirection == RotationDirection::COUNTER_CLOCKWISE)
                std::cout << "COUNTER_CLOCKWISE" << std::endl;
        } else {
            std::cout << "N/A" << std::endl;
        }

        std::cout << "Heading: " << heading << std::endl;

        cv::imshow("Lidar View", lidarMat);

        // Only show camera view if available
        if (hasCamera && !timedFrame.frame.empty()) {
            cv::Mat cameraMat = timedFrame.frame.clone();
            camera_processor::drawColorMasks(cameraMat, colorMasks);
            cv::imshow("Camera View", cameraMat);
        }
    }

    cv::destroyAllWindows();
    return 0;
}
