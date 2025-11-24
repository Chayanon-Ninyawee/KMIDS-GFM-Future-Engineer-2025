## `camera_processor.h` Reference: Camera Frame Processing

This header defines utility functions and data structures for processing raw camera frames (`TimedFrame`) captured by the `CameraModule`. It primarily handles color filtering in the HSV space, contour detection, and calculating the horizontal angle of detected objects (blocks).

______________________________________________________________________

### Namespace: `camera_processor`

#### Color Range Constants (HSV)

These constants define the target color ranges in the Hue-Saturation-Value (HSV) color space, used for initial thresholding. The ranges are optimized for typical lighting conditions.

| Color | Range Name | H (Hue) | S (Saturation) | V (Value) | Description |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Red** | `lowerRed1Light` | 0 | 110 | 170 | Primary red range. |
| **Red** | `upperRed1Light` | 1 | 230 | 255 | |
| **Red** | `lowerRed2Light` | 166 | 70 | 170 | Secondary red range (for wrapping around H=180). |
| **Red** | `upperRed2Light` | 180 | 230 | 255 | |
| **Green** | `lowerGreen1Light` | 45 | 110 | 100 | Primary green range. |
| **Green** | `upperGreen1Light` | 85 | 230 | 200 | |
| **Pink** | `lowerPink1Light` | 165 | 244 | 200 | Primary pink/magenta range. |
| **Pink** | `upperPink1Light` | 171 | 255 | 255 | |

#### Data Structures

| Struct Name | Description | Members |
| :--- | :--- | :--- |
| **`ContourInfo`** | Stores geometric properties of a single detected object contour. | `cv::Point2f centroid`: Center position (x, y) in pixels. <br> `double area`: Area in pixels (double precision). |
| **`ColorInfo`** | Aggregates all results for a single color channel after processing. | `std::vector<ContourInfo> contours`: List of contours that exceeded the area threshold. <br> `cv::Mat mask`: The resulting binary mask (monochrome image) for this color. |
| **`ColorMasks`** | Holds the complete processing results for all tracked colors. | `ColorInfo red`: Results for the red color channel. <br> `ColorInfo green`: Results for the green color channel. |
| **`BlockAngle`** | Final, processed angular data for a detected object (block). | `float angle`: Horizontal angle in radians relative to the camera center. <br> `double area`: Area in pixels. <br> `cv::Point2f centroid`: Centroid pixel coordinates. <br> `Color color`: The color enum of the block. |

#### Enumeration

| Enum Name | Description | Values |
| :--- | :--- | :--- |
| **`Color`** | Defines the colors relevant for block detection. | `RED`, `GREEN` |

#### Core Functions

| Function Signature | Description |
| :--- | :--- |
| **`ColorMasks filterColors(const TimedFrame &timedFrame, double areaThreshold = 600.0)`** | **Color Filtering & Contour Extraction.** Converts the input frame to HSV, thresholds for red, green, and pink, computes contours, and filters them based on the `areaThreshold`. |
| **`void drawColorMasks(cv::Mat &img, const ColorMasks &colors)`** | **Visualization.** Draws the extracted information onto the original image: overlays semi-transparent masks and annotates the centroids with their area and position. |
| **`float pixelToAngle(int pixelX, int imageWidth, float hfov)`** | **Geometric Conversion.** Calculates the horizontal angle (in radians) of a point relative to the camera's optical center, given its pixel x-coordinate, image width, and the camera's horizontal field of view. |
| **`std::vector<BlockAngle> computeBlockAngles(const ColorMasks &masks, int imageWidth = 1296, float hfov = 110.0f)`** | **Angle Calculation.** Processes the detected contours in `ColorMasks` (red and green) and uses `pixelToAngle` to convert the centroid's x-coordinate into a horizontal angle for each block. |

##### `pixelToAngle` Geometry

The `pixelToAngle` function is crucial for translating pixel coordinates into real-world directional data.

$$\\text{Angle} (\\text{rad}) = \\arctan \\left( \\left( \\frac{\\text{pixelX} - \\text{center} \\cdot \\text{offset}}{\\text{imageWidth}} \\right) \\cdot 2 \\cdot \\tan \\left( \\frac{\\text{hfov}}{2} \\right) \\right)$$

Where:

- $\\text{Center Offset} = \\text{pixelX} - (\\text{imageWidth} / 2)$
- $\\text{Angle}$ is negative to the left and positive to the right of the center.
