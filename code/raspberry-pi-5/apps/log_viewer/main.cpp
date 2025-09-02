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

TimedPico2Data reconstructTimedPico2(const LogEntry &entry) {
    TimedPico2Data pico2Data{};

    struct {
        ImuAccel accel;
        ImuEuler euler;
        double encoderAngle;
    } payload;

    std::memcpy(&payload, entry.data.data(), sizeof(payload));

    // Assign fields
    pico2Data.accel = payload.accel;
    pico2Data.euler = payload.euler;
    pico2Data.encoderAngle = payload.encoderAngle;

    // Reconstruct timestamp
    auto ts = std::chrono::nanoseconds(entry.timestamp);
    pico2Data.timestamp = std::chrono::steady_clock::time_point(ts);

    return pico2Data;
}

std::vector<TimedPico2Data> reconstructPico2RingBufferVector(
    const std::vector<LogEntry> &pico2Entries,
    size_t currentIdx,
    size_t windowSize = 120
) {
    std::vector<TimedPico2Data> result;
    if (pico2Entries.empty() || currentIdx >= pico2Entries.size()) {
        return result;
    }

    // Compute start index (clamp to 0)
    size_t startIdx = (currentIdx >= windowSize - 1) ? currentIdx - (windowSize - 1) : 0;

    result.reserve(currentIdx - startIdx + 1);

    for (size_t i = startIdx; i <= currentIdx; ++i) {
        result.push_back(reconstructTimedPico2(pico2Entries[i]));
    }

    return result;
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

    // 1/30 s in nanoseconds
    constexpr uint64_t stepNs = static_cast<uint64_t>(1e9 / 30.0);

    // Current playback time starts at first lidar timestamp
    uint64_t currentTime = lidarEntries.front().timestamp;

    while (true) {
        int key = cv::waitKey(0);
        if (key == 27) break;  // ESC

        if (key == 81) {  // left arrow
            if (currentTime > stepNs) currentTime -= stepNs;
        } else if (key == 83) {  // right arrow
            currentTime += stepNs;
        } else {
            continue;
        }

        // ---- LIDAR ----
        lidarIdx = findClosestIndex(lidarEntries, lidarIdx, currentTime);
        if (lidarIdx >= lidarEntries.size()) continue;
        const auto &lidarEntry = lidarEntries[lidarIdx];
        TimedLidarData timedLidarData = reconstructTimedLidar(lidarEntry);

        // ---- Pico2 ----
        pico2Idx = findClosestIndex(pico2Entries, pico2Idx, currentTime);
        if (pico2Idx >= pico2Entries.size()) continue;
        const auto &pico2Entry = pico2Entries[pico2Idx];
        TimedPico2Data timedPico2Data = reconstructTimedPico2(pico2Entry);
        auto timedPico2Datas = reconstructPico2RingBufferVector(pico2Entries, pico2Idx);

        if (not initialHeading) initialHeading = timedPico2Data.euler.h;

        float heading = timedPico2Data.euler.h - initialHeading.value_or(0.0f);
        heading = std::fmod(heading, 360.0f);
        if (heading < 0.0f) heading += 360.0f;

        // ---- Camera ----
        TimedFrame timedFrame;
        camera_processor::ColorMasks colorMasks;
        std::vector<camera_processor::BlockAngle> blockAngles;
        if (hasCamera) {
            cameraIdx = findClosestIndex(cameraEntries, cameraIdx, currentTime);
            if (cameraIdx < cameraEntries.size()) {
                timedFrame = reconstructTimedFrame(cameraEntries[cameraIdx]);
                colorMasks = camera_processor::filterColors(timedFrame);
                blockAngles = camera_processor::computeBlockAngles(colorMasks, camWidth, camHFov);

                // use blockAngles here...
            }
        }

        // ---- Lidar processing ----
        auto filteredLidarData = lidar_processor::filterLidarData(timedLidarData);

        auto deltaPose = combined_processor::aproximateRobotPose(filteredLidarData, timedPico2Datas);

        std::cout << "Encoder: " << timedPico2Data.encoderAngle << std::endl;

        std::cout << "[DeltaPose] ΔX: " << deltaPose.deltaX << " m, ΔY: " << deltaPose.deltaY << " m, ΔH: " << deltaPose.deltaH << " deg"
                  << std::endl;

        auto lineSegments = lidar_processor::getLines(filteredLidarData, deltaPose, 0.05f, 10, 0.10f, 0.10f, 18.0f, 0.20f);
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
