#pragma once

/**
 * Structure to hold data for each LIDAR scan node.
 * Contains the angle and distance of the detected point.
 */
struct RawLidarNode {
    float angle;
    float distance;
};
