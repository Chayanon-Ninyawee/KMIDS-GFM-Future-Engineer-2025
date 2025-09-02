#pragma once

#include "camera_processor.h"
#include "camera_struct.h"
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
    cv::Point2f cameraOffset = {0.0f, 0.15f},
    float maxAngleDiff = 5.0f
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
