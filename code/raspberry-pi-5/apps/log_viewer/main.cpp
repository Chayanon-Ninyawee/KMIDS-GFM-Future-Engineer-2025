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
const float camHFov = 110.0f;

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
    size_t windowSize = 30
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

void drawLineFromAngle(cv::Mat &img, int cx, int cy, float angle, const cv::Scalar &color, int thickness = 2) {
    // Extend line length far beyond image size
    float length = std::max(img.cols, img.rows) * 2.0f;

    // Compute end point
    int x2 = static_cast<int>(cx + length * std::sin(angle * M_PI / 180.0f));
    int y2 = static_cast<int>(cy - length * std::cos(angle * M_PI / 180.0f));  // y-axis inverted in images

    // Draw line
    cv::line(img, cv::Point(cx, cy), cv::Point(x2, y2), color, thickness);
}

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <log_folder>" << std::endl;
        return 1;
    }

    std::string folderPath = argv[1];

    // Detect challenge type
    bool isOpenChallenge = fs::exists(fs::path(folderPath) / "openChallenge.bin");
    bool isScanMap = fs::exists(fs::path(folderPath) / "scanMap.bin");
    bool isObstacleChallenge = fs::exists(fs::path(folderPath) / "obstacleChallenge.bin");

    // Count how many challenge logs exist
    int challengeCount = (isOpenChallenge ? 1 : 0) + (isScanMap ? 1 : 0) + (isObstacleChallenge ? 1 : 0);

    if (challengeCount == 0) {
        std::cerr << "Error: No challenge log file found in folder: " << folderPath << std::endl;
        return 1;
    }
    if (challengeCount > 1) {
        std::cerr << "Error: More than one challenge log file found in folder: " << folderPath << std::endl;
        return 1;
    }

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

    // ---- Camera (only for scanMap & obstacleChallenge) ----
    std::vector<LogEntry> cameraEntries;
    bool hasCamera = false;

    if (!isOpenChallenge) {
        if (fs::exists(cameraLogFile)) {
            LogReader cameraReader(cameraLogFile);
            if (!cameraReader.readAll(cameraEntries)) {
                std::cerr << "Failed to read Camera log file: " << cameraLogFile << std::endl;
                return 1;
            }
            std::cout << "Loaded " << cameraEntries.size() << " Camera log entries." << std::endl;
            hasCamera = true;
        } else {
            std::cerr << "Expected camera log file, but not found: " << cameraLogFile << std::endl;
            return 1;
        }
    } else {
        std::cout << "OpenChallenge detected → no camera log expected." << std::endl;
    }

    // ---- Main loop logs ----
    std::vector<LogEntry> openChallengeEntries;
    std::vector<LogEntry> scanMapEntries;
    std::vector<LogEntry> obstacleChallengeEntries;

    std::vector<std::chrono::steady_clock::time_point> loopTimestamps;

    if (isOpenChallenge) {
        LogReader openChallengeReader((fs::path(folderPath) / "openChallenge.bin").string());
        if (!openChallengeReader.readAll(openChallengeEntries)) {
            std::cerr << "Failed to read openChallenge log" << std::endl;
            return 1;
        }
        std::cout << "Loaded " << openChallengeEntries.size() << " openChallenge entries." << std::endl;

        for (const auto &entry : openChallengeEntries) {
            auto tp = std::chrono::steady_clock::time_point(std::chrono::nanoseconds(entry.timestamp));
            loopTimestamps.push_back(tp);
        }
    }

    if (isScanMap) {
        LogReader scanMapReader((fs::path(folderPath) / "scanMap.bin").string());
        if (!scanMapReader.readAll(scanMapEntries)) {
            std::cerr << "Failed to read scanMap log" << std::endl;
            return 1;
        }
        std::cout << "Loaded " << scanMapEntries.size() << " scanMap entries." << std::endl;

        for (const auto &entry : scanMapEntries) {
            auto tp = std::chrono::steady_clock::time_point(std::chrono::nanoseconds(entry.timestamp));
            loopTimestamps.push_back(tp);
        }
    }

    if (isObstacleChallenge) {
        LogReader obstacleChallengeReader((fs::path(folderPath) / "obstacleChallenge.bin").string());
        if (!obstacleChallengeReader.readAll(obstacleChallengeEntries)) {
            std::cerr << "Failed to read obstacleChallenge log" << std::endl;
            return 1;
        }
        std::cout << "Loaded " << obstacleChallengeEntries.size() << " obstacleChallenge entries." << std::endl;

        for (const auto &entry : obstacleChallengeEntries) {
            auto tp = std::chrono::steady_clock::time_point(std::chrono::nanoseconds(entry.timestamp));
            loopTimestamps.push_back(tp);
        }
    }

    // Windows: only open Camera View if we have camera data
    cv::namedWindow("Lidar View", cv::WINDOW_FULLSCREEN);
    if (hasCamera) {
        cv::namedWindow("Camera View", cv::WINDOW_FULLSCREEN);
        cv::setMouseCallback("Camera View", mouseCallback);
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

    size_t currentTimeIdx = 0;
    int playDirection = 0;  // -1 = backward, +1 = forward, 0 = stopped
    auto lastPressTime = std::chrono::steady_clock::now();

    while (true) {
        int key = cv::waitKey(1);
        if (key == 'q') break;

        // --- Handle taps ---
        if (key == 'j') {  // step one frame back (single frame mode)
            if (currentTimeIdx > 0) currentTimeIdx--;
        } else if (key == 'l') {  // step one frame forward
            if (currentTimeIdx < loopTimestamps.size() - 1) currentTimeIdx++;
        }

        // --- Handle continuous ---
        if (key == 'a') {
            playDirection = -1;
            lastPressTime = std::chrono::steady_clock::now();
        } else if (key == 'd') {
            playDirection = +1;
            lastPressTime = std::chrono::steady_clock::now();
        } else if (key == -1) {
            // check how long since last press
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastPressTime).count();

            if (elapsed > 50) {
                // more than ~50ms without real press → stop auto-move
                playDirection = 0;
            }
        }

        if (playDirection == -1 && currentTimeIdx > 0)
            currentTimeIdx--;
        else if (playDirection == +1 && currentTimeIdx < loopTimestamps.size() - 1)
            currentTimeIdx++;
        else if (key == -1 && playDirection == 0)
            continue;
        uint64_t currentTime =
            std::chrono::duration_cast<std::chrono::nanoseconds>(loopTimestamps[currentTimeIdx].time_since_epoch()).count();

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

                if (key == 'i') {
                    selectMode = true;
                    polygonPoints.clear();
                    std::cout << "Click points to define polygon, then press 'c' to confirm.\n";

                    continue;
                } else if (key == 'c') {
                    selectMode = false;
                    cv::Scalar lower1, upper1, lower2, upper2;
                    computeHSVBounds(timedFrame.frame, lower1, upper1, lower2, upper2);
                    std::cout << "Lower1 HSV: " << lower1 << "\n";
                    std::cout << "Upper1 HSV: " << upper1 << "\n";
                    std::cout << "Lower2 HSV: " << lower2 << "\n";
                    std::cout << "Upper2 HSV: " << upper2 << "\n";

                    continue;
                }
            }
        }

        // ---- Lidar processing ----
        auto filteredLidarData = lidar_processor::filterLidarData(timedLidarData);

        auto deltaPose = combined_processor::aproximateRobotPose(filteredLidarData, timedPico2Datas);

        // std::cout << "Encoder: " << timedPico2Data.encoderAngle << std::endl;

        // std::cout << "[DeltaPose] ΔX: " << deltaPose.deltaX << " m, ΔY: " << deltaPose.deltaY << " m, ΔH: " << deltaPose.deltaH << " deg"
        //           << std::endl;

        auto lineSegments = lidar_processor::getLines(filteredLidarData, deltaPose, 0.05f, 10, 0.10f, 0.10f, 18.0f, 0.20f);
        auto relativeWalls = lidar_processor::getRelativeWalls(lineSegments, Direction::fromHeading(heading), heading, 0.30f, 25.0f, 0.22f);

        auto newRobotTurnDirecton = lidar_processor::getTurnDirection(relativeWalls);
        if (newRobotTurnDirecton) robotTurnDirection = newRobotTurnDirecton;

        auto resolveWalls = lidar_processor::resolveWalls(relativeWalls);
        auto parkingWalls = lidar_processor::getParkingWalls(lineSegments, Direction::fromHeading(heading), heading, 0.25f);
        auto trafficLightPoints = lidar_processor::getTrafficLightPoints(filteredLidarData, resolveWalls, deltaPose, robotTurnDirection);

        auto trafficLightInfos = combined_processor::combineTrafficLightInfo(blockAngles, trafficLightPoints);

        if (robotTurnDirection) {
            auto classifiedLights = combined_processor::classifyTrafficLights(
                trafficLightInfos,
                resolveWalls,
                *robotTurnDirection,
                Segment::fromHeading(heading)
            );

            for (const auto &ct : classifiedLights) {
                std::cout << "Traffic Light at LiDAR position (" << ct.info.lidarPosition.x << ", " << ct.info.lidarPosition.y << ")"
                          << " mapped to Segment " << static_cast<int>(ct.location.segment) << ", Location "
                          << static_cast<int>(ct.location.location) << ", WallSide "
                          << (ct.location.side == WallSide::INNER ? "INNER" : "OUTER") << std::endl;
            }
        }

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

        for (const auto &block : blockAngles) {
            // Choose color for drawing
            cv::Scalar lineColor = (block.color == camera_processor::Color::RED) ? cv::Scalar(0, 0, 255)   // Red in BGR
                                                                                 : cv::Scalar(0, 255, 0);  // Green in BGR
            drawLineFromAngle(lidarMat, 400, 400 - (800 / 6.0f * 0.15), block.angle, lineColor, 2);
        }

        if (robotTurnDirection) {
            if (*robotTurnDirection == RotationDirection::CLOCKWISE)
                std::cout << "CLOCKWISE" << std::endl;
            else if (*robotTurnDirection == RotationDirection::COUNTER_CLOCKWISE)
                std::cout << "COUNTER_CLOCKWISE" << std::endl;
        } else {
            std::cout << "N/A" << std::endl;
        }

        // std::cout << "Heading: " << heading << std::endl;

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
