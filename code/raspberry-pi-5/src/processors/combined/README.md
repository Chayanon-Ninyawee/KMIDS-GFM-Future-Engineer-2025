## `combined_processor.h` Reference: Fusing Sensor Data

This header defines utilities for synchronizing and fusing data from the robot's primary sensors: the Camera (`TimedFrame`), the LIDAR (`TimedLidarData`), and the motion/pose data from the Pico 2 module (`TimedPico2Data`). The primary goal is to combine the visual classification of objects (blocks) with their accurate location from the LIDAR.

______________________________________________________________________

### Namespace: `combined_processor`

#### Data Structures

| Struct Name | Description | Members |
| :--- | :--- | :--- |
| **`SyncedLidarCamera`** | A pair of data samples (frame and scan) determined to be temporally synchronized. | `TimedFrame frame`: The camera frame. <br> `TimedLidarData lidar`: The corresponding LIDAR scan. |
| **`TrafficLightInfo`** | The fused result for a detected traffic light, combining 3D position and visual classification. | `cv::Point2f lidarPosition`: Position $(x, y)$ of the traffic light cluster in the LIDAR coordinate frame. <br> `camera_processor::BlockAngle cameraBlock`: Corresponding block information (color, angle, area) from the camera. |
| **`TrafficLightLocation`** | Classified location of a traffic light relative to the global path and surrounding walls. | `Segment segment`: Quadrant segment (A–D). <br> `SegmentLocation location`: Position within the segment (front/mid/back). <br> `WallSide side`: Proximity to the inner or outer wall. |
| **`ClassifiedTrafficLight`** | A traffic light object paired with its path-relative classification. | `TrafficLightInfo info`: The original fused data. <br> `TrafficLightLocation location`: The calculated path classification. |

#### Core Fusion and Synchronization Functions

| Function Signature | Description |
| :--- | :--- |
| **`RobotDeltaPose aproximateRobotPose(const TimedLidarData &timedLidarData, const std::vector<TimedPico2Data> &timedPico2Datas)`** | **Motion Compensation.** Estimates the accumulated change in the robot's position ($\\Delta x, \\Delta y$) and heading ($\\Delta H$) between the time the LIDAR scan was captured and the most recent time step. This is done by integrating motion data from the Pico 2 samples within that time window. |
| **`std::optional<SyncedLidarCamera> syncLidarCamera(...)`** | **Temporal Synchronization.** Attempts to pair a camera frame and a LIDAR scan based on their timestamps and a predefined `cameraDelay`. Returns the matched pair, or $\\text{nullopt}$ if no temporally corresponding data is found in the provided buffers. |
| **`std::vector<TrafficLightInfo> combineTrafficLightInfo(...)`** | **Spatial Fusion (Traffic Lights).** Matches the angular position of a detected visual block (from camera) with the angular position of a classified point cluster (from LIDAR) to determine which LIDAR point corresponds to which traffic light color. It accounts for the `cameraOffset` relative to the LIDAR. |

#### Classification Functions

| Function Signature | Description |
| :--- | :--- |
| **`std::vector<ClassifiedTrafficLight> classifyTrafficLights(...)`** | **Path Classification.** Determines the location of a traffic light relative to the robot's current path segment (A–D) and surrounding walls (inner/outer). This process involves complex geometric checks: <br> 1. **Segment Determination:** Maps the light's world coordinate position to the path segment. <br> 2. **Segment Location:** Calculates position (A/B/C) based on distance to the front/back of the segment, factoring in the `turnDirection`. <br> 3. **Wall Side:** Determines `INNER` or `OUTER` side based on the light's perpendicular distance from the path walls. |

#### Visualization Functions

| Function Signature | Description |
| :--- | :--- |
| **`void drawTrafficLightInfo(cv::Mat &img, const TrafficLightInfo &info, float scale = 6.0f, int radius = 4)`** | **Visualization.** Draws the fused traffic light information on an image. The $x, y$ position from the LiDAR is scaled and overlaid on the image, and the circle's color is determined by the camera's color classification (Red or Green). |
