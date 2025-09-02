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
    const RobotDeltaPose &robotDeltaPose,
    cv::Point2f cameraOffset,
    float maxAngleDiff
) {
    std::vector<TrafficLightInfo> trafficLightInfos;
    std::vector<cv::Point2f> avaliableLidarPoints = lidarPoints;

    // Apply delta transform: translate (-deltaX, -deltaY) and rotate (-deltaH)
    float radH = robotDeltaPose.deltaH * static_cast<float>(M_PI) / 180.0f;
    float cosH = std::cos(radH);
    float sinH = std::sin(radH);

    avaliableLidarPoints.reserve(lidarPoints.size());
    for (const auto &p : lidarPoints) {
        // Translate
        float xt = p.x - robotDeltaPose.deltaX;
        float yt = p.y - robotDeltaPose.deltaY;

        // Rotate around (0,0)
        cv::Point2f transformed{xt * cosH - yt * sinH, xt * sinH + yt * cosH};
        avaliableLidarPoints.push_back(transformed);
    }

    for (const auto &block : blockAngles) {
        size_t bestIndex = std::numeric_limits<size_t>::max();
        float smallestDiff = std::numeric_limits<float>::max();
        float closestDistance = std::numeric_limits<float>::max();

        // Loop over all available LiDAR points
        for (size_t i = 0; i < avaliableLidarPoints.size(); ++i) {
            const auto &lp = avaliableLidarPoints[i];
            float dx = lp.x - cameraOffset.x;
            float dy = lp.y - cameraOffset.y;

            float lidarAngle = std::atan2(dy, dx);  // radians
            float lidarAngleDeg = lidarAngle * 180.0f / M_PI;

            float angleDiff = std::abs(90.0f - block.angle - lidarAngleDeg);

            // Distance of LiDAR point from origin (0,0)
            float originDistance = std::sqrt(lp.x * lp.x + lp.y * lp.y);

            // Scale tolerance: closer points allow bigger angleDiff, farther ones stricter
            // Example: tolerance shrinks like 1/originDistance
            float dynamicMaxAngleDiff = maxAngleDiff / (1.0f + originDistance * 2.0f);

            // If angle difference within distance-scaled tolerance
            if (angleDiff <= dynamicMaxAngleDiff) {
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
            trafficLightInfos.push_back(TrafficLightInfo{avaliableLidarPoints[bestIndex], block});
            avaliableLidarPoints.erase(avaliableLidarPoints.begin() + bestIndex);  // consume
        }
    }

    return trafficLightInfos;
}

std::vector<ClassifiedTrafficLight> classifyTrafficLights(
    const std::vector<TrafficLightInfo> &trafficLights,
    const lidar_processor::ResolvedWalls &resolvedWalls,
    RotationDirection turnDirection,
    Segment currentSegment

) {
    std::vector<ClassifiedTrafficLight> results;
    results.reserve(trafficLights.size());

    // Pick outer/inner walls
    std::optional<lidar_processor::LineSegment> outerWall, innerWall;
    if (turnDirection == RotationDirection::CLOCKWISE) {
        outerWall = resolvedWalls.leftWall;
        innerWall = resolvedWalls.rightWall;
    } else {
        outerWall = resolvedWalls.rightWall;
        innerWall = resolvedWalls.leftWall;
    }

    for (const auto &tl : trafficLights) {
        const cv::Point2f &p = tl.lidarPosition;

        // --- Distances ---
        float frontDist = 0.0f;
        if (resolvedWalls.frontWall) {
            frontDist = resolvedWalls.frontWall->perpendicularDistance(p.x, p.y);
        } else if (resolvedWalls.backWall) {
            frontDist = 3.0f - resolvedWalls.backWall->perpendicularDistance(p.x, p.y);
        }

        float outerDist = 0.0f;
        if (outerWall) {
            outerDist = outerWall->perpendicularDistance(p.x, p.y);
        } else if (innerWall) {
            outerDist = 1.0f - innerWall->perpendicularDistance(p.x, p.y);
        }

        Segment seg = currentSegment;

        // --- SegmentLocation: A/B/C depending on rotation ---
        SegmentLocation loc;
        if (turnDirection == RotationDirection::CLOCKWISE) {
            if (frontDist > 0.80 && frontDist < 1.15f)
                loc = SegmentLocation::A;  // front
            else if (frontDist > 1.35 && frontDist < 1.65f)
                loc = SegmentLocation::B;  // mid
            else if (frontDist > 1.85 && frontDist < 2.15)
                loc = SegmentLocation::C;  // back
            else
                continue;
        } else {
            if (frontDist > 0.80 && frontDist < 1.15f)
                loc = SegmentLocation::C;  // front (reverse)
            else if (frontDist > 1.35 && frontDist < 1.65f)
                loc = SegmentLocation::B;  // mid
            else if (frontDist > 1.85 && frontDist < 2.15)
                loc = SegmentLocation::A;  // back
            else
                continue;
        }

        // --- WallSide: choose closer wall ---
        WallSide side;
        if (outerDist < 0.480)
            side = WallSide::OUTER;
        else if (outerDist > 0.520)
            side = WallSide::INNER;
        else
            continue;

        // --- Pack result ---
        results.push_back(ClassifiedTrafficLight{tl, TrafficLightLocation{seg, loc, side}});
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
