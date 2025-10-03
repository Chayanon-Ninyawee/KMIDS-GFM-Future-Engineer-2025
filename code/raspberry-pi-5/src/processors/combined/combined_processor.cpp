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
    float trafficLightRadius
) {
    std::vector<TrafficLightInfo> trafficLightInfos;
    std::vector<cv::Point2f> avaliableLidarPoints = lidarPoints;

    // ---- Helper: ray-circle intersection ----
    auto rayCircleIntersect =
        [](const cv::Point2f &rayOrigin, const cv::Point2f &rayDir, const cv::Point2f &circleCenter, float radius, float &tHit) -> bool {
        cv::Point2f oc = rayOrigin - circleCenter;
        float b = 2.0f * (oc.x * rayDir.x + oc.y * rayDir.y);
        float c = oc.x * oc.x + oc.y * oc.y - radius * radius;
        float disc = b * b - 4 * c;
        if (disc < 0) return false;

        float sqrtDisc = std::sqrt(disc);
        float t1 = (-b - sqrtDisc) * 0.5f;
        float t2 = (-b + sqrtDisc) * 0.5f;

        if (t1 >= 0) {
            tHit = t1;
            return true;
        }
        if (t2 >= 0) {
            tHit = t2;
            return true;
        }
        return false;
    };

    // ---- Track which lidar points were hit, and by what colors ----
    std::unordered_map<size_t, std::vector<camera_processor::BlockAngle>> lidarHits;

    for (const auto &block : blockAngles) {
        float rayAngle = (90.0f - block.angle) * static_cast<float>(M_PI) / 180.0f;
        cv::Point2f rayDir{std::cos(rayAngle), std::sin(rayAngle)};

        size_t bestIndex = std::numeric_limits<size_t>::max();
        float bestTHit = std::numeric_limits<float>::max();

        for (size_t i = 0; i < avaliableLidarPoints.size(); ++i) {
            float tHit;
            if (rayCircleIntersect(cameraOffset, rayDir, avaliableLidarPoints[i], trafficLightRadius, tHit)) {
                if (tHit < bestTHit) {  // keep the nearest intersection
                    bestTHit = tHit;
                    bestIndex = i;
                }
            }
        }

        if (bestIndex != std::numeric_limits<size_t>::max()) {
            // only record the closest hit for this ray
            lidarHits[bestIndex].push_back(block);
        }
    }

    // ---- Conflict resolution: discard if multiple colors hit the same point ----
    for (auto &kv : lidarHits) {
        auto &blocks = kv.second;
        bool conflict = false;

        camera_processor::Color firstColor = blocks[0].color;
        for (auto &b : blocks) {
            if (b.color != firstColor) {
                conflict = true;
                break;
            }
        }

        if (!conflict) {
            cv::Point2f rel = lidarPoints[kv.first] - cameraOffset;
            float angle = 90.0f - (std::atan2(rel.y, rel.x) * 180.0f / static_cast<float>(M_PI));

            if (angle >= -54.0f && angle <= 54.0f) {
                for (auto &b : blocks) {
                    trafficLightInfos.push_back(TrafficLightInfo{lidarPoints[kv.first], b});
                }
            }
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

        // --- SegmentLocation: A/B/C depending on rotation ---
        Segment seg;
        SegmentLocation loc;
        WallSide side;
        if (outerDist < 0.900) {
            if (turnDirection == RotationDirection::CLOCKWISE) {
                seg = currentSegment;

                if (frontDist > 0.80 && frontDist < 1.15f)
                    loc = SegmentLocation::C;  // front
                else if (frontDist > 1.35 && frontDist < 1.65f)
                    loc = SegmentLocation::B;  // mid
                else if (frontDist > 1.85 && frontDist < 2.15)
                    loc = SegmentLocation::A;  // back
                else
                    continue;
            } else {
                seg = currentSegment;

                if (frontDist > 0.80 && frontDist < 1.15f)
                    loc = SegmentLocation::A;  // front (reverse)
                else if (frontDist > 1.35 && frontDist < 1.65f)
                    loc = SegmentLocation::B;  // mid
                else if (frontDist > 1.85 && frontDist < 2.15)
                    loc = SegmentLocation::C;  // back
                else
                    continue;
            }

            if (outerDist < 0.480)
                side = WallSide::OUTER;
            else if (outerDist > 0.520)
                side = WallSide::INNER;
            else
                continue;
        } else {
            if (turnDirection == RotationDirection::CLOCKWISE) {
                float nextHeading = currentSegment.toHeading() + 90.0f;
                nextHeading = std::fmod(nextHeading + 360.0f, 360.0f);
                seg = Segment::fromHeading(nextHeading);

                if (outerDist > 0.900 && outerDist < 1.15f)
                    loc = SegmentLocation::A;  // front
                else if (outerDist > 1.35 && outerDist < 1.65f)
                    loc = SegmentLocation::B;  // mid
                else if (outerDist > 1.85 && outerDist < 2.15)
                    loc = SegmentLocation::C;  // back
                else
                    continue;
            } else {
                float nextHeading = currentSegment.toHeading() - 90.0f;
                nextHeading = std::fmod(nextHeading + 360.0f, 360.0f);
                seg = Segment::fromHeading(nextHeading);

                if (outerDist >= 0.900 && outerDist < 1.15f)
                    loc = SegmentLocation::C;  // front
                else if (outerDist > 1.35 && outerDist < 1.65f)
                    loc = SegmentLocation::B;  // mid
                else if (outerDist > 1.85 && outerDist < 2.15)
                    loc = SegmentLocation::A;  // back
                else
                    continue;
            }

            if (frontDist < 0.480)
                side = WallSide::OUTER;
            else if (frontDist > 0.520)
                side = WallSide::INNER;
            else
                continue;
        }

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
    case camera_processor::Color::RED:
        color = cv::Scalar(0, 0, 255);  // BGR
        break;
    case camera_processor::Color::GREEN:
        color = cv::Scalar(0, 255, 0);
        break;
    default:
        color = cv::Scalar(255, 255, 255);  // fallback white
        break;
    }

    cv::circle(img, cv::Point(x, y), radius, color, cv::FILLED);
}

}  // namespace combined_processor
