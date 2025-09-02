#pragma once

#include "camera_processor.h"
#include "camera_struct.h"
#include "direction.h"
#include "lidar_processor.h"
#include "lidar_struct.h"
#include "pico2_struct.h"
#include "ring_buffer.hpp"
#include "robot_pose_struct.h"

#include <optional>

namespace combined_processor
{

struct SyncedLidarCamera {
    TimedFrame frame;
    TimedLidarData lidar;
};

/**
 * @brief Approximate the robot's movement since the LIDAR scan using Pico2 data.
 *
 * Loops through the Pico2 samples from the latest sample before the LIDAR timestamp
 * to the most recent, computing the accumulated change in robot pose.
 *
 * @param timedLidarData The LIDAR scan with timestamp.
 * @param timedPico2Datas Time-ordered vector of Pico2 samples.
 * @return RobotDeltaPose containing deltaX, deltaY, and deltaH.
 */
RobotDeltaPose aproximateRobotPose(const TimedLidarData &timedLidarData, const std::vector<TimedPico2Data> &timedPico2Datas);

/**
 * @brief Synchronize a camera frame with a lidar scan, accounting for delay.
 *
 * @param timedFrames Vector of camera frames (time-sorted).
 * @param timedLidarDatas Vector of lidar scans (time-sorted).
 * @param cameraDelay Camera delay relative to lidar:
 *        - Negative: lidar is slower than camera
 *        - Positive: camera is slower than lidar
 * @return std::optional<SyncedData>
 *         The synchronized result, or nullopt if no match found.
 */
std::optional<SyncedLidarCamera> syncLidarCamera(
    const std::vector<TimedFrame> &timedFrames,
    const std::vector<TimedLidarData> &timedLidarDatas,
    std::chrono::milliseconds cameraDelay
);

struct TrafficLightInfo {
    cv::Point2f lidarPosition;                 ///< Position from LiDAR
    camera_processor::BlockAngle cameraBlock;  ///< Corresponding camera block info
};

/**
 * @brief Combine camera block angles and LiDAR traffic light points.
 *
 * Only returns traffic lights that have a matching camera block based on horizontal angle.
 * The LiDAR points are assumed to be in the LiDAR coordinate frame, and the camera may be
 * offset relative to the LiDAR. The function accounts for this offset when computing angles.
 *
 * @param blockAngles Vector of camera BlockAngle (red/green blocks).
 * @param lidarPoints Vector of 2D points from LiDAR (traffic lights), in LiDAR coordinates.
 * @param cameraOffset Position of the camera relative to the LiDAR (x, y) in LiDAR coordinates.
 * @param maxAngleDiff Maximum allowed difference in angle (radians) to consider a camera block
 *                     as corresponding to a LiDAR point.
 * @return std::vector<TrafficLightInfo> Combined information for traffic lights that have
 *                                      matching camera blocks.
 */
std::vector<TrafficLightInfo> combineTrafficLightInfo(
    const std::vector<camera_processor::BlockAngle> &blockAngles,
    const std::vector<cv::Point2f> &lidarPoints,
    const RobotDeltaPose &robotDeltaPose,
    cv::Point2f cameraOffset = {0.0f, 0.15f},
    float maxAngleDiff = 5.0f
);

/**
 * @brief Represents the classified location of a traffic light relative to the robot's path and walls.
 */
struct TrafficLightLocation {
    Segment segment;           ///< Quadrant segment (A–D) where the traffic light is located.
    SegmentLocation location;  ///< Relative position within the segment (front/mid/back).
    WallSide side;             ///< Relative wall side (inner or outer) the traffic light is near.
};

/**
 * @brief A traffic light along with its classified location in the environment.
 */
struct ClassifiedTrafficLight {
    TrafficLightInfo info;          ///< Original traffic light data (LiDAR and camera block).
    TrafficLightLocation location;  ///< Classified location relative to the robot and walls.
};

/**
 * @brief Classify traffic lights relative to the robot's path and surrounding walls.
 *
 * For each traffic light, this function determines:
 * 1. The segment (A–D) the traffic light is in relative to the robot's current segment.
 * 2. Its position within the segment (SegmentLocation: A/B/C) based on the distance to the front/back wall.
 * 3. Which wall side (WallSide: INNER/OUTER) the traffic light is closest to.
 *
 * Distances are computed using perpendicular distance from walls. If a front or back wall
 * is missing, fallback distances are used (3.0f for back, 1.0f for inner). Traffic lights
 * outside the expected distance ranges are ignored.
 *
 * SegmentLocation mapping depends on rotation direction:
 * - CLOCKWISE:
 *     - Front wall: SegmentLocation::A
 *     - Mid distance: SegmentLocation::B
 *     - Back distance: SegmentLocation::C
 * - COUNTER_CLOCKWISE:
 *     - Front wall: SegmentLocation::C
 *     - Mid distance: SegmentLocation::B
 *     - Back distance: SegmentLocation::A
 *
 * WallSide is chosen based on the distance to the outer wall:
 * - outerDist < 0.48 → OUTER
 * - outerDist > 0.52 → INNER
 * Traffic lights in the ambiguous range (0.48–0.52) are skipped.
 *
 * @param trafficLights Vector of traffic lights to classify.
 * @param resolvedWalls Resolved walls from LiDAR, used to compute distances.
 * @param turnDirection Robot rotation direction (CLOCKWISE or COUNTER_CLOCKWISE).
 * @param currentSegment Robot's current segment (A–D), used as a reference for classification.
 * @return std::vector<ClassifiedTrafficLight> Classified traffic lights with segment, location, and wall side.
 */
std::vector<ClassifiedTrafficLight> classifyTrafficLights(
    const std::vector<TrafficLightInfo> &trafficLights,
    const lidar_processor::ResolvedWalls &resolvedWalls,
    RotationDirection turnDirection,
    Segment currentSegment
);

/**
 * @brief Draws traffic light info on an image.
 *
 * The traffic light position is taken from the LiDAR point and scaled to the image.
 * The color of the circle is based on the camera block color.
 *
 * @param img Image to draw on (CV_8UC3).
 * @param info TrafficLightInfo struct containing LiDAR position and camera block.
 * @param scale Scale factor for converting world coordinates to image pixels.
 * @param radius Circle radius in pixels.
 */
void drawTrafficLightInfo(cv::Mat &img, const TrafficLightInfo &info, float scale = 6.0f, int radius = 4);

}  // namespace combined_processor
