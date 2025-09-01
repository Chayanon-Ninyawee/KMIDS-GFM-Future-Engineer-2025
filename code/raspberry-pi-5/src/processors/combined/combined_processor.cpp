#include "combined_processor.h"

#include <cmath>
#include <iostream>

namespace combined_processor
{

constexpr float WHEEL_DIAMETER = 0.055f;

RobotDeltaPose aproximateRobotPose(const TimedLidarData &timedLidarData, const std::vector<TimedPico2Data> &timedPico2Datas) {
    RobotDeltaPose deltaPose{0.0f, 0.0f, 0.0f};

    if (timedPico2Datas.empty()) return deltaPose;

    // Find index of latest Pico2 sample before LIDAR timestamp
    int index = -1;
    for (int i = static_cast<int>(timedPico2Datas.size()) - 1; i >= 0; --i) {
        if (timedPico2Datas[i].timestamp <= timedLidarData.timestamp) {
            index = i;
            break;  // stop at first valid sample
        }
    }
    if (index < 0) return deltaPose;

    float lastHeading = timedPico2Datas[index].euler.h;

    // Initialize robot pose relative to LIDAR timestamp
    float totalDeltaX = 0.0f;
    float totalDeltaY = 0.0f;
    float totalDeltaH = 0.0f;

    // Loop from index+1 to latest Pico2 data
    for (size_t i = index + 1; i < timedPico2Datas.size(); ++i) {
        const auto &prev = timedPico2Datas[i - 1];
        const auto &curr = timedPico2Datas[i];

        // Calculate heading difference
        float dHeading = curr.euler.h - prev.euler.h;
        dHeading = std::fmod(dHeading + 180.0f, 360.0f) - 180.0f;

        totalDeltaH += dHeading;
        totalDeltaH = std::fmod(totalDeltaH, 360.0f);
        if (totalDeltaH < 0.0f) totalDeltaH += 360.0f;

        // Calculate encoder delta distance
        double dDistance = (curr.encoderAngle - prev.encoderAngle) * (M_PI * WHEEL_DIAMETER) / 360.0;

        // Convert relative movement to x/y
        float headingRad = dHeading * M_PI / 180.0f;
        totalDeltaX += dDistance * std::sin(headingRad);
        totalDeltaY += dDistance * std::cos(headingRad);
    }

    deltaPose.deltaX = totalDeltaX;
    deltaPose.deltaY = totalDeltaY;
    deltaPose.deltaH = totalDeltaH;

    return deltaPose;
}

std::optional<SyncedLidarCamera> syncLidarCamera(
    const std::vector<TimedFrame> &timedFrames,
    const std::vector<TimedLidarData> &timedLidarDatas,
    std::chrono::milliseconds cameraDelay
) {
    if (timedFrames.empty() || timedLidarDatas.empty()) return std::nullopt;

    for (const auto &frame : timedFrames) {
        auto adjustedTime = frame.timestamp + cameraDelay;

        // Find lidar data closest in time
        const TimedLidarData *bestMatch = nullptr;
        auto bestDiff = std::chrono::milliseconds::max();

        for (const auto &lidar : timedLidarDatas) {
            auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(adjustedTime - lidar.timestamp);
            if (std::abs(diff.count()) < bestDiff.count()) {
                bestDiff = diff;
                bestMatch = &lidar;
            }
        }

        if (bestMatch) {
            return SyncedLidarCamera{frame, *bestMatch};
        }
    }

    return std::nullopt;
}

std::vector<TrafficLightInfo> combineTrafficLightInfo(
    const std::vector<camera_processor::BlockAngle> &blockAngles,
    const std::vector<cv::Point2f> &lidarPoints,
    cv::Point2f cameraOffset,
    float maxAngleDiff
) {
    std::vector<TrafficLightInfo> results;
    std::vector<cv::Point2f> availableLidar = lidarPoints;

    for (const auto &block : blockAngles) {
        size_t bestIndex = std::numeric_limits<size_t>::max();
        float smallestDiff = std::numeric_limits<float>::max();
        float closestDistance = std::numeric_limits<float>::max();

        // Loop over all available LiDAR points
        for (size_t i = 0; i < availableLidar.size(); ++i) {
            const auto &lp = availableLidar[i];
            float dx = lp.x - cameraOffset.x;
            float dy = lp.y - cameraOffset.y;

            float lidarAngle = std::atan2(dy, dx);  // radians
            float lidarAngleDeg = lidarAngle * 180.0f / M_PI;

            float angleDiff = std::abs(90.0f - block.angle - lidarAngleDeg);

            if (angleDiff <= maxAngleDiff) {
                float distanceAlongRay = std::sqrt(dx * dx + dy * dy);  // distance from camera to LiDAR point

                // Pick the closest along the ray (smallest distance)
                if (distanceAlongRay < closestDistance || angleDiff < smallestDiff) {
                    closestDistance = distanceAlongRay;
                    smallestDiff = angleDiff;
                    bestIndex = i;
                }
            }
        }

        if (bestIndex != std::numeric_limits<size_t>::max()) {
            results.push_back(TrafficLightInfo{availableLidar[bestIndex], block});
            availableLidar.erase(availableLidar.begin() + bestIndex);  // consume
        }
    }

    return results;
}

void drawTrafficLightInfo(cv::Mat &img, const TrafficLightInfo &info, float scale, int radius) {
    CV_Assert(!img.empty());
    CV_Assert(img.type() == CV_8UC3);

    cv::Point center(img.cols / 2, img.rows / 2);

    int x = static_cast<int>(center.x + info.lidarPosition.x * (img.cols / scale));
    int y = static_cast<int>(center.y - info.lidarPosition.y * (img.rows / scale));

    // Choose color based on camera block
    cv::Scalar color;
    switch (info.cameraBlock.color) {
    case camera_processor::Color::Red:
        color = cv::Scalar(0, 0, 255);  // BGR
        break;
    case camera_processor::Color::Green:
        color = cv::Scalar(0, 255, 0);
        break;
    default:
        color = cv::Scalar(255, 255, 255);  // fallback white
        break;
    }

    cv::circle(img, cv::Point(x, y), radius, color, cv::FILLED);
}

}  // namespace combined_processor
