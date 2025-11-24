## `lidar_processor.h` Reference: LIDAR Data Processing and Segmentation

This header provides geometric utilities and core algorithms for processing raw LIDAR scan points. It focuses on filtering, converting polar data to Cartesian coordinates, extracting line segments (walls), and classifying these segments relative to the robot's pose.

______________________________________________________________________

### Namespace: `lidar_processor`

#### Data Structures

| Struct Name | Description | Members |
| :--- | :--- | :--- |
| **`LineSegment`** | Represents a 2D line segment in the Cartesian plane. Includes methods for geometric calculations. | `float x1, y1, x2, y2`: Coordinates of the segment endpoints. |
| **`RelativeWalls`** | Groups candidate wall segments extracted from the scan based on their cardinal direction relative to the robot's current heading. | `std::vector<LineSegment> frontWalls`: Segments directly ahead. <br> `std::vector<LineSegment> rightWalls`: Segments to the right side. <br> `std::vector<LineSegment> backWalls`: Segments behind. <br> `std::vector<LineSegment> leftWalls`: Segments to the left side. |
| **`ResolvedWalls`** | Stores the final, single best-fit line segment selected for each major wall side, potentially including far sides for context. | `std::optional<LineSegment> frontWall`, `rightWall`, `backWall`, `leftWall`, `farLeftWall`, `farRightWall`: The selected wall segment for each direction. |

#### `LineSegment` Utility Methods

These methods are embedded within the `LineSegment` struct for geometric analysis:

| Method | Description | Formula / Behavior |
| :--- | :--- | :--- |
| **`float length() const`** | Calculates the Euclidean distance between the two endpoints of the segment. | $\\text{Length} = \\sqrt{(x_2 - x_1)^2 + (y_2 - y_1)^2}$ |
| **`float perpendicularDistance(float x, float y) const`** | Calculates the shortest distance from a given point $(x, y)$ to the infinite line defined by the segment. | $\\text{Distance} = \\frac{|(y_2 - y_1)x - (x_2 - x_1)y + x_2 y_1 - y_2 x_1|}{\\sqrt{(x_2 - x_1)^2 + (y_2 - y_1)^2}}$ |
| **`float perpendicularDirection(float x, float y) const`** | Calculates the direction (angle in degrees) of the normal vector from the line toward the point $(x, y)$, normalized to $\[0, 360)$. | Calculates the angle of the line's perpendicular vector, adjusted based on which side of the line the point lies. |

#### Core Processing Functions

| Function Signature | Description |
| :--- | :--- |
| **`TimedLidarData filterLidarData(const TimedLidarData &timedLidarData, float minDistance = 0.05f)`** | **Radial Filter.** Filters out LIDAR points that are too close to the sensor (less than `minDistance`), typically used to ignore the robot's own structure or mounting artifacts. |
| **`std::vector<LineSegment> getLines(...)`** | **Line Extraction.** The primary algorithm for converting raw polar LIDAR points into simplified line segments. It involves: <br> 1. Cartesian conversion with optional `robotDeltaPose` motion compensation. <br> 2. Segmentation based on the RANSAC/split-and-merge principle, using `splitThreshold` for best-fit line deviation. <br> 3. Merging of collinear segments based on `mergeAngleThreshold` and `mergeGapThreshold`. |
| **`RelativeWalls getRelativeWalls(...)`** | **Wall Grouping.** Takes the extracted `LineSegment`s and groups them into `frontWalls`, `rightWalls`, etc., based on the robot's `heading` and the desired `targetDirection`. |
| **`std::optional<RotationDirection> getTurnDirection(const RelativeWalls &walls)`** | **Path Analysis.** Determines the robot's next logical turn direction (CLOCKWISE or COUNTER_CLOCKWISE) based on the presence, absence, and configuration of the relative walls. |
| **`ResolvedWalls resolveWalls(const RelativeWalls &relativeWalls)`** | **Wall Selection.** From the candidate groups in `RelativeWalls`, selects the single most representative `LineSegment` for each cardinal direction (front, back, left, right). |
| **`std::vector<LineSegment> getParkingWalls(...)`** | **Parking Feature Extraction.** Identifies short, distinctive line segments (less than `maxLength`) in specific areas that are indicative of parking spots or obstacles. |
| **`std::vector<cv::Point2f> getTrafficLightPoints(...)`** | **Traffic Light Detection.** Identifies small, distinct clusters of LIDAR points that are likely traffic lights. The process uses proximity clustering and filters points near known walls (`resolveWalls`) to distinguish objects from boundary lines. |

#### Visualization Functions

These functions provide utilities for rendering the processed LIDAR data onto an OpenCV image matrix (`cv::Mat`).

| Function Signature | Description |
| :--- | :--- |
| **`void drawLidarData(cv::Mat &img, const TimedLidarData &timedLidarDatas, float scale = 4.0f)`** | Draws the raw LIDAR scan points onto the image, scaling the world coordinates (meters) to image pixels. |
| **`void drawLineSegment(cv::Mat &img, const LineSegment &segment, ...)`** | Draws a single line segment. The `scale` parameter translates the segment's world coordinates into pixel positions for visualization. |
| **`void drawTrafficLightPoint(cv::Mat &img, const cv::Point2f &point, ...)`** | Draws a detected traffic light point as a colored circle, using the same scaling system. |
