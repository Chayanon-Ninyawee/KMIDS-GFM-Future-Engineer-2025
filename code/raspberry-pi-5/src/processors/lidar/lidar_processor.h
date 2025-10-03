#pragma once

#include <opencv2/opencv.hpp>
#include <optional>

#include "direction.h"
#include "lidar_struct.h"
#include "robot_pose_struct.h"

namespace lidar_processor
{

/**
 * @brief Represents a 2D line segment in Cartesian coordinates.
 */
struct LineSegment {
    float x1, y1, x2, y2;

    /**
     * @brief Compute the Euclidean length of the segment.
     * @return Length of the line segment.
     */
    float length() const {
        float dx = x2 - x1;
        float dy = y2 - y1;
        return std::hypot(dx, dy);
    }

    /**
     * @brief Compute the perpendicular distance from a point to this line.
     *
     * @param x   X-coordinate of the point.
     * @param y   Y-coordinate of the point.
     * @return The perpendicular distance from the point to the infinite line.
     *         Returns 0.0f if the line points are nearly identical.
     */
    float perpendicularDistance(float x, float y) const {
        float num = std::fabs((y2 - y1) * x - (x2 - x1) * y + x2 * y1 - y2 * x1);
        float den = std::hypot(x2 - x1, y2 - y1);
        return (den > 1e-6f) ? num / den : 0.0f;
    }

    /**
     * @brief Compute the perpendicular direction from this line to a point.
     *
     * @param x   X-coordinate of the point.
     * @param y   Y-coordinate of the point.
     * @return The angle of the perpendicular direction in degrees (0 ≤ angle < 360),
     *         measured relative to the positive X-axis.
     * @throws std::invalid_argument if the line segment has zero length.
     */
    float perpendicularDirection(float x, float y) const {
        // Line vector
        float dx = x2 - x1;
        float dy = y2 - y1;

        if (dx == 0.0f && dy == 0.0f) throw std::invalid_argument("Zero-length line segment");

        // Perpendicular vector (rotate CCW)
        float perpX = -dy;
        float perpY = dx;

        // Side test using cross product
        float cross = dx * (y - y1) - dy * (x - x1);
        if (cross < 0) {
            perpX = -perpX;
            perpY = -perpY;
        }

        // Angle in degrees, normalized to [0,360)
        float angle = std::atan2(perpY, perpX) * 180.0f / static_cast<float>(M_PI);
        angle = std::fmod(angle + 180.0f + 360.0f, 360.0f);

        return angle;
    }
};

/**
 * @brief Groups LiDAR-detected walls by relative robot position.
 */
struct RelativeWalls {
    std::vector<LineSegment> frontWalls;  ///< Candidate segments in front
    std::vector<LineSegment> rightWalls;  ///< Candidate segments to the right
    std::vector<LineSegment> backWalls;   ///< Candidate segments behind
    std::vector<LineSegment> leftWalls;   ///< Candidate segments to the left
};

/**
 * @brief Holds resolved single wall segments from candidate walls.
 */
struct ResolvedWalls {
    std::optional<LineSegment> frontWall;     ///< Selected front wall
    std::optional<LineSegment> rightWall;     ///< Selected right wall
    std::optional<LineSegment> backWall;      ///< Selected back wall
    std::optional<LineSegment> leftWall;      ///< Selected left wall
    std::optional<LineSegment> farLeftWall;   ///< Selected far left wall
    std::optional<LineSegment> farRightWall;  ///< Selected far right wall
};

/**
 * @brief Removes lidar points closer than a given distance.
 *
 * Constructs a new TimedLidarData containing only the points from the input
 * whose distance is greater than or equal to the specified threshold. The
 * timestamp from the input data is preserved.
 *
 * @param timedLidarData Source lidar data with timestamp and raw points.
 * @param minDistance Minimum accepted distance in meters. Defaults to 0.05f.
 * @return A TimedLidarData object with filtered lidar points.
 */
TimedLidarData filterLidarData(const TimedLidarData &timedLidarData, float minDistance = 0.05f);

/**
 * @brief Extract line segments from timed LiDAR data by converting points to Cartesian coordinates,
 *        splitting them based on deviation from a best-fit line, and merging approximately collinear segments.
 *
 * The function performs the following steps:
 * 1. Filters out points with invalid distances or outside certain angular/distance ranges.
 * 2. Converts the remaining polar coordinates (angle, distance) to Cartesian coordinates in meters.
 * 3. Splits the points into candidate line segments using `splitSegment`, where a segment is split if any point
 *    deviates from the best-fit line by more than `splitThreshold`.
 * 4. Merges segments that are approximately collinear and close together using `mergeSegments`.
 *
 * @param timedLidarData LiDAR scan data with timestamps.
 * @param robotDeltaPose Robot motion compensation (Δx, Δy, Δh) applied to transform points (default: {0,0,0}).
 * @param splitThreshold Maximum perpendicular distance (meters) from a point to the best-fit line before splitting the segment.
 * @param minPoints Minimum number of points required to form a segment.
 * @param maxPointGap Maximum allowed distance between consecutive points within a segment.
 * @param minLength Minimum length of a segment to keep after splitting.
 * @param mergeAngleThreshold Maximum angle difference (degrees) between segments to merge them.
 * @param mergeGapThreshold Maximum gap (meters) between segments to merge them.
 * @return Vector of LineSegment representing the merged line segments extracted from the LiDAR data.
 */
std::vector<LineSegment> getLines(
    const TimedLidarData &timedLidarData,
    const RobotDeltaPose &robotDeltaPose = {0.0f, 0.0f, 0.0f},
    float splitThreshold = 0.05f,
    int minPoints = 10,
    float maxPointGap = 0.10f,
    float minLength = 0.10f,
    float mergeAngleThreshold = 18.0f,
    float mergeGapThreshold = 0.20f
);

/**
 * @brief Determine relative walls around the robot
 *
 * @param lineSegments Vector of detected line segments.
 * @param targetDirection Robot's target movement direction.
 * @param heading Robot's current heading in degrees.
 * @param minLength Minimum length for a wall to be considered.
 * @param angleThresholdDeg Maximum deviation from two walls to consider as aligned wall.
 * @param collinearThreshold Maximum perpendicular distance to merge collinear segments.
 * @return RelativeWalls struct grouping segments by relative position.
 */
RelativeWalls getRelativeWalls(
    const std::vector<LineSegment> &lineSegments,
    Direction targetDirection,
    float heading,
    float minLength = 0.30f,
    float angleThresholdDeg = 25.0f,
    float collinearThreshold = 0.22f
);

/**
 * @brief Determine the robot's turn direction based on relative walls.
 *
 * @param walls Resolved or candidate walls around the robot.
 * @return Optional RotationDirection; empty if turn direction can't be determined.
 */
std::optional<RotationDirection> getTurnDirection(const RelativeWalls &walls);

/**
 * @brief Selects a single representative wall per side from relative walls.
 *
 * @param relativeWalls Candidate walls grouped by side.
 * @return ResolvedWalls struct containing one wall per side if available.
 */
ResolvedWalls resolveWalls(const RelativeWalls &relativeWalls);

/**
 * @brief Extract walls relevant for parking from relative walls.
 *
 * @param lineSegments Vector of detected line segments.
 * @param targetDirection Robot's target movement direction.
 * @param heading Robot's current heading in degrees.
 * @param maxLength Maximum length of a wall to consider it a parking wall.
 * @return Vector of LineSegment for parking walls.
 */
std::vector<LineSegment> getParkingWalls(
    const std::vector<LineSegment> &lineSegments,
    Direction targetDirection,
    float heading,
    float maxLength = 0.25f
);

/**
 * @brief Detect traffic light points from LiDAR data and resolved walls.
 *
 * @param timedLidarData LiDAR scan data with timestamps.
 * @param resolveWalls Resolved walls around the robot.
 * @param turnDirection Optional turn direction of the robot (if has no value assume CLOCKWISE).
 * @param distanceThreshold Maximum distance between points to cluster into a single traffic light point.
 * @param minClusterSize Minimum cluster size
 * @return Vector of 2D points representing detected traffic light locations.
 */
std::vector<cv::Point2f> getTrafficLightPoints(
    const TimedLidarData &timedLidarData,
    const ResolvedWalls &resolveWalls,
    const RobotDeltaPose &robotDeltaPose,
    std::optional<RotationDirection> turnDirection,
    float distanceThreshold = 0.05f,
    size_t minClusterSize = 10
);

/**
 * @brief Draw LiDAR scan points onto an existing image.
 *
 * @param img The cv::Mat to draw on. Must be already allocated with correct size and type.
 * @param timedLidarDatas The LiDAR scan data to visualize.
 * @param scale Meters-to-pixels scaling. For example, scale=4 means 4 meters = img.rows pixels.
 */
void drawLidarData(cv::Mat &img, const TimedLidarData &timedLidarDatas, float scale = 4.0f);

/**
 * @brief Draw a single line segment onto an existing image.
 *
 * @param img The cv::Mat to draw on. Must be already allocated with correct size and type.
 * @param segment The LineSegment to draw.
 * @param scale Meters-to-pixels scaling. For example, scale=4 means 4 meters = img.rows pixels.
 * @param color Line color (default green).
 * @param thickness Line thickness (default 2).
 */
void drawLineSegment(
    cv::Mat &img,
    const LineSegment &segment,
    float scale = 4.0f,
    cv::Scalar color = cv::Scalar(0, 255, 0),
    int thickness = 2
);

/**
 * @brief Draw a traffic light point onto an image.
 *
 * @param img The cv::Mat to draw on. Must be preallocated.
 * @param point 2D point representing the traffic light location.
 * @param scale Meters-to-pixels scaling. For example, scale=4 means 4 meters = img.rows pixels.
 * @param color Point color.
 * @param radius Radius of the point in pixels.
 */
void drawTrafficLightPoint(
    cv::Mat &img,
    const cv::Point2f &point,
    float scale = 4.0f,
    cv::Scalar color = cv::Scalar(255, 0, 0),
    int radius = 6
);

}  // namespace lidar_processor
