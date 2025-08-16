#include <opencv2/opencv.hpp>
#include <optional>

#include "direction.h"
#include "lidar_struct.h"

namespace lidar_processor
{

struct LineSegment {
    float x1, y1, x2, y2;
};

// Container for all walls
struct Walls {
    // 4 directions × 2 WallSide = 8 possible walls
    std::array<std::optional<LineSegment>, 4 * 2> walls;

    // Set or get wall by Direction + RelativeSide (rotation-agnostic)
    std::optional<LineSegment> &get(Direction dir, RelativeSide side) {
        // Store Left = index 0, Right = index 1
        return walls[static_cast<int>(dir) * 2 + static_cast<int>(side)];
    }

    const std::optional<LineSegment> &get(Direction dir, RelativeSide side) const {
        return const_cast<Walls *>(this)->get(dir, side);
    }

    // Convert Left/Right to Inner/Outer once rotationDirection is known
    std::optional<LineSegment> &get(Direction dir, WallSide side, RotationDirection rotation) {
        // Map WallSide -> RelativeSide based on rotation
        RelativeSide relSide;
        switch (rotation) {
        case RotationDirection::Clockwise:
            relSide = (side == WallSide::Inner) ? RelativeSide::Right : RelativeSide::Left;
            break;
        case RotationDirection::CounterClockwise:
            relSide = (side == WallSide::Inner) ? RelativeSide::Left : RelativeSide::Right;
            break;
        }
        return get(dir, relSide);
    }

    const std::optional<LineSegment> &get(Direction dir, WallSide side, RotationDirection rotation) const {
        return const_cast<Walls *>(this)->get(dir, side, rotation);
    }
};

std::vector<LineSegment> getLines(
    const TimedLidarData &timedLidarData,
    float splitThreshold = 0.05f,
    int minPoints = 10,
    float maxPointGap = 0.10f,
    float minLength = 0.10f,
    float mergeAngleThreshold = 18.0f,
    float mergeGapThreshold = 0.20f
);

std::vector<LineSegment> getWalls(
    const std::vector<LineSegment> &lineSegments,
    float heading,
    float minLength = 0.30f,
    float angleThresholdDeg = 25.0f,
    float collinearThreshold = 0.22f
);

std::vector<LineSegment> getParkingWalls(const std::vector<LineSegment> &lineSegments, float maxLength = 0.25);

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

}  // namespace lidar_processor
