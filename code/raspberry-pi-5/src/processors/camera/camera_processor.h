#pragma once

#include <opencv2/opencv.hpp>

#include "camera_struct.h"

namespace camera_processor
{

// ------------------------ Color Ranges (HSV) ------------------------
const cv::Scalar lowerRed1Light(0, 124, 110);
const cv::Scalar upperRed1Light(4, 242, 245);
// If a second red range is required
const cv::Scalar lowerRed2Light(175, 124, 110);
const cv::Scalar upperRed2Light(180, 242, 245);

const cv::Scalar lowerGreen1Light(58, 122, 91);
const cv::Scalar upperGreen1Light(61, 142, 184);
// If a second green range is required
const cv::Scalar lowerGreen2Light(61, 165, 91);
const cv::Scalar upperGreen2Light(79, 229, 184);

const cv::Scalar lowerPink1Light(165, 244, 200);
const cv::Scalar upperPink1Light(171, 255, 255);
// Optional second pink range
const cv::Scalar lowerPink2Light(171, 255, 255);
const cv::Scalar upperPink2Light(171, 255, 255);

/**
 * @brief Stores information about a single contour.
 *
 * Contains the geometric centroid and area of a contour.
 * The area is represented as a double to preserve the
 * subpixel precision provided by OpenCV’s cv::contourArea().
 */
struct ContourInfo {
    cv::Point2f centroid;  ///< Centroid of the contour (x, y) in image coordinates.
    double area;           ///< Area of the contour in pixels (double precision).
};

/**
 * @brief Stores information about all valid contours of a single color.
 *
 * Contains:
 * - The binary mask for the color.
 * - A list of contour information (centroid and area) for
 *   contours above the specified area threshold.
 */
struct ColorInfo {
    std::vector<ContourInfo> contours;  ///< All contours that passed the area threshold.
    cv::Mat mask;                       ///< Binary mask representing the color.
};

/**
 * @brief Stores filtered results for multiple colors.
 *
 * Holds the masks and contour information for:
 * - Red
 * - Green
 */
struct ColorMasks {
    ColorInfo red;    ///< Results for red color range.
    ColorInfo green;  ///< Results for green color range.
};

/**
 * @brief Filters an input frame for red, green, and pink colors and extracts contours.
 *
 * The input frame is converted to HSV color space, thresholded into binary masks
 * for each target color (red, green, pink), and processed to extract contour
 * centroids and areas. Contours smaller than the given area threshold are discarded.
 *
 * @param timedFrame Input frame with timestamp and image data.
 * @param areaThreshold Minimum area threshold (in pixels) to filter out small/noisy contours.
 * @return ColorMasks A struct containing masks and contour info for red, green, and pink.
 */
ColorMasks filterColors(const TimedFrame &timedFrame, double areaThreshold = 500.0);

/**
 * @brief Draws ColorMasks on an image.
 *
 * - Overlays the mask for each color with semi-transparent color.
 * - Draws the centroids as circles.
 * - Annotates each centroid with its area and position.
 *
 * @param img The image to draw on (CV_8UC3).
 * @param colors The ColorMasks containing masks and contours.
 */
void drawColorMasks(cv::Mat &img, const ColorMasks &colors);

/**
 * @brief Convert a camera pixel x-coordinate into a horizontal angle.
 *
 * Angle is measured relative to the camera optical axis:
 *   - Negative = left of center
 *   - Zero     = image center
 *   - Positive = right of center
 *
 * @param pixelX      Pixel x-coordinate (0 = left, width-1 = right).
 * @param imageWidth  Width of the image in pixels.
 * @param hfov        Horizontal field of view of the camera in radians.
 * @return float      Angle in radians relative to camera center.
 */
float pixelToAngle(int pixelX, int imageWidth, float hfov);

/**
 * @brief Enum for colors we care about.
 */
enum class Color
{
    Red,
    Green
};

/**
 * @brief Stores angle information of detected blocks.
 */
struct BlockAngle {
    float angle;           ///< Horizontal angle of the block (radians).
    double area;           ///< Area of the block (pixels).
    cv::Point2f centroid;  ///< Centroid of the block in image coordinates.
    Color color;           ///< Color of the block.
};

/**
 * @brief Compute the horizontal angles of all detected red and green blocks.
 *
 * This function processes the contours stored in a ColorMasks structure,
 * calculating the horizontal angle of each block relative to the camera's
 * optical axis using the x-coordinate of the contour's centroid.
 *
 * @param masks       The ColorMasks containing red and green contour data.
 * @param imageWidth  The width of the camera image in pixels. Default is 1296.
 * @param hfov        The horizontal field of view of the camera in degrees. Default is 100.0f.
 *
 * @return std::vector<BlockAngle> A vector of BlockAngle structs, each containing:
 *         - angle: Horizontal angle of the block in radians.
 *         - area: Area of the block in pixels.
 *         - centroid: Pixel coordinates of the block's centroid.
 *         - color: Color enum (Red or Green) of the block.
 *
 * @note The function assumes that the centroid x-coordinate is sufficient
 *       to determine the block's horizontal angle and does not compute
 *       vertical angles.
 */
std::vector<BlockAngle> computeBlockAngles(const ColorMasks &masks, int imageWidth = 1296, float hfov = 100.0f);

}  // namespace camera_processor
