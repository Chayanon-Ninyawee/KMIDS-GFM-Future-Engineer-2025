#include "lidar_processor.h"

namespace lidar_processor
{

namespace
{

    /**
     * @brief Convert LiDAR coordinates to standard Cartesian coordinates.
     *
     * The LiDAR is mounted rotated 90° clockwise relative to the normal coordinate system.
     * This function rotates the LiDAR coordinates counterclockwise by 90° to align them
     * with the standard (X, Y) axes. It also applies any necessary sign flips.
     *
     * @param lidarX X-coordinate from the LiDAR (distance * sin(angle)).
     * @param lidarY Y-coordinate from the LiDAR (distance * cos(angle)).
     * @param x Reference to store the converted X-coordinate.
     * @param y Reference to store the converted Y-coordinate.
     */
    inline void lidarToCartesian(float lidarX, float lidarY, float &x, float &y) {
        x = lidarY;   // LiDAR Y becomes standard X
        y = -lidarX;  // LiDAR X becomes negative standard Y
    }

    /**
     * @brief Compute the perpendicular distance from a point to a line.
     *
     * This function calculates the shortest Euclidean distance from a point (x, y)
     * to the infinite line defined by two points (x1, y1) and (x2, y2).
     *
     * @param x   X-coordinate of the point.
     * @param y   Y-coordinate of the point.
     * @param x1  X-coordinate of the first point on the line.
     * @param y1  Y-coordinate of the first point on the line.
     * @param x2  X-coordinate of the second point on the line.
     * @param y2  Y-coordinate of the second point on the line.
     *
     * @return The perpendicular distance from the point to the line.
     *         Returns 0.0f if the line points are nearly identical.
     */
    float perpendicularDistance(float x, float y, float x1, float y1, float x2, float y2) {
        float num = std::fabs((y2 - y1) * x - (x2 - x1) * y + x2 * y1 - y2 * x1);
        float den = std::hypot(x2 - x1, y2 - y1);
        return (den > 1e-6f) ? num / den : 0.0f;
    }

    /**
     * @brief Compute the perpendicular direction from a line to a point.
     *
     * This function calculates the angle (in degrees) of the perpendicular vector
     * from a line (defined by (x1, y1) → (x2, y2)) to a point (x, y).
     * The perpendicular is taken as a 90° counterclockwise rotation of the line’s
     * direction vector, and its orientation is flipped if the point lies on the
     * opposite side of the line. The angle is normalized to [0, 360).
     *
     * @param x   X-coordinate of the point.
     * @param y   Y-coordinate of the point.
     * @param x1  X-coordinate of the first point on the line.
     * @param y1  Y-coordinate of the first point on the line.
     * @param x2  X-coordinate of the second point on the line.
     * @param y2  Y-coordinate of the second point on the line.
     *
     * @return The angle of the perpendicular direction in degrees (0 ≤ angle < 360),
     *         measured relative to the positive X-axis.
     */
    float perpendicularDirection(float x, float y, float x1, float y1, float x2, float y2) {
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

    // --- Recursive Split Step ---
    void splitSegment(
        const std::vector<cv::Point2f> &points,
        int start,
        int end,
        std::vector<LineSegment> &segments,
        float threshold,
        int minPointsPerSegment,
        float maxPointGap,
        float minLength
    ) {
        if (end <= start + 1) return;

        float x1 = points[start].x;
        float y1 = points[start].y;
        float x2 = points[end].x;
        float y2 = points[end].y;

        // Find farthest point from the segment
        float maxDist = 0.0f;
        int index = -1;
        for (int i = start + 1; i < end; i++) {
            float dist = perpendicularDistance(points[i].x, points[i].y, x1, y1, x2, y2);
            if (dist > maxDist) {
                maxDist = dist;
                index = i;
            }
        }

        if (maxDist > threshold && index != -1) {
            splitSegment(points, start, index, segments, threshold, minPointsPerSegment, maxPointGap, minLength);
            splitSegment(points, index, end, segments, threshold, minPointsPerSegment, maxPointGap, minLength);
            return;
        }

        if (end - start < minPointsPerSegment) return;

        for (int i = start; i < end; i++) {
            float gap = cv::norm(points[i + 1] - points[i]);
            if (gap > maxPointGap) {
                splitSegment(points, start, i, segments, threshold, minPointsPerSegment, maxPointGap, minLength);
                splitSegment(points, i + 1, end, segments, threshold, minPointsPerSegment, maxPointGap, minLength);
                return;
            }
        }

        float length = std::hypot(x2 - x1, y2 - y1);
        if (length < minLength) return;

        segments.push_back({x1, y1, x2, y2});
    }

    // --- Merge Step: Merge collinear & close segments ---
    std::vector<LineSegment> mergeSegments(const std::vector<LineSegment> &segments, float angleThresholdDeg, float gapThreshold) {
        std::vector<LineSegment> mergedSegments;
        if (segments.empty()) return mergedSegments;

        mergedSegments.push_back(segments.front());
        for (size_t i = 1; i < segments.size(); i++) {
            auto &last = mergedSegments.back();
            const auto &curr = segments[i];

            // Compute angles
            float angleLast = std::atan2(last.y2 - last.y1, last.x2 - last.x1);
            float angleCurr = std::atan2(curr.y2 - curr.y1, curr.x2 - curr.x1);
            float angleDiff = std::fabs(angleLast - angleCurr) * 180.0f / static_cast<float>(M_PI);

            // If nearly collinear and endpoints are close -> merge
            float endGap = std::hypot(curr.x1 - last.x2, curr.y1 - last.y2);
            if (angleDiff < angleThresholdDeg && endGap < gapThreshold) {
                last.x2 = curr.x2;
                last.y2 = curr.y2;
            } else {
                mergedSegments.push_back(curr);
            }
        }

        // --- Circular merge: check first and last segments ---
        if (mergedSegments.size() > 1) {
            auto &first = mergedSegments.front();
            auto &last = mergedSegments.back();

            float angleFirst = std::atan2(first.y2 - first.y1, first.x2 - first.x1);
            float angleLast = std::atan2(last.y2 - last.y1, last.x2 - last.x1);
            float angleDiff = std::fabs(angleFirst - angleLast);
            if (angleDiff > M_PI) angleDiff = 2 * M_PI - angleDiff;
            angleDiff = angleDiff * 180.0f / static_cast<float>(M_PI);

            float gap1 = std::hypot(first.x1 - last.x2, first.y1 - last.y2);
            float gap2 = std::hypot(first.x2 - last.x1, first.y2 - last.y1);
            float endGap = std::min(gap1, gap2);

            if (angleDiff < angleThresholdDeg && endGap < gapThreshold) {
                // Merge last into first
                first.x1 = last.x1;
                first.y1 = last.y1;
                mergedSegments.pop_back();
            }
        }

        return mergedSegments;
    }

    std::vector<LineSegment> mergeAlignedSegments(
        const std::vector<LineSegment> &segments,
        float angleThresholdDeg,
        float collinearThreshold
    ) {
        std::vector<LineSegment> mergedSegments = segments;
        bool merged;

        do {
            merged = false;
            std::vector<LineSegment> newSegments;
            std::vector<bool> used(mergedSegments.size(), false);

            for (size_t i = 0; i < mergedSegments.size(); ++i) {
                if (used[i]) continue;

                LineSegment current = mergedSegments[i];
                cv::Point2f start(current.x1, current.y1);
                cv::Point2f end(current.x2, current.y2);
                cv::Point2f dir = end - start;
                float mag = cv::norm(dir);
                if (mag > 1e-6f) dir /= mag;

                for (size_t j = i + 1; j < mergedSegments.size(); ++j) {
                    if (used[j]) continue;
                    LineSegment other = mergedSegments[j];

                    // Check if lines are aligned & collinear
                    float angle1 = std::atan2(current.y2 - current.y1, current.x2 - current.x1);
                    float angle2 = std::atan2(other.y2 - other.y1, other.x2 - other.x1);
                    float angleDiff = std::fabs(angle1 - angle2);
                    if (angleDiff > M_PI) angleDiff = 2 * M_PI - angleDiff;
                    angleDiff = angleDiff * 180.0f / static_cast<float>(M_PI);

                    if (angleDiff > angleThresholdDeg) continue;

                    cv::Point2f otherStart(other.x1, other.y1);
                    cv::Point2f otherEnd(other.x2, other.y2);

                    if (current.perpendicularDistance(otherStart.x, otherStart.y) > collinearThreshold &&
                        current.perpendicularDistance(otherEnd.x, otherEnd.y) > collinearThreshold)
                    {
                        // Check intermediate points
                        bool aligned = false;
                        int numIntermediatePoints = 10;
                        for (int k = 1; k < numIntermediatePoints; ++k) {
                            float t = float(k) / float(numIntermediatePoints - 1);
                            cv::Point2f pt = otherStart + t * (otherEnd - otherStart);
                            if (current.perpendicularDistance(pt.x, pt.y) <= collinearThreshold) {
                                aligned = true;
                                break;
                            }
                        }
                        if (!aligned) continue;
                    }

                    // Project endpoints onto current line direction to extend the segment
                    std::vector<cv::Point2f> pts = {start, end, otherStart, otherEnd};
                    auto proj = [&](const cv::Point2f &pt) { return (pt - start).dot(dir); };

                    double minProj = proj(pts[0]);
                    double maxProj = minProj;
                    cv::Point2f minPt = pts[0], maxPt = pts[0];

                    for (const auto &pt : pts) {
                        double p = proj(pt);
                        if (p < minProj) {
                            minProj = p;
                            minPt = pt;
                        }
                        if (p > maxProj) {
                            maxProj = p;
                            maxPt = pt;
                        }
                    }

                    start = minPt;
                    end = maxPt;

                    used[j] = true;
                    merged = true;
                }

                newSegments.push_back({start.x, start.y, end.x, end.y});
            }

            mergedSegments = std::move(newSegments);

        } while (merged);

        return mergedSegments;
    }

}  // namespace

TimedLidarData filterLidarData(const TimedLidarData &timedLidarData, float minDistance) {
    TimedLidarData filteredLidarData;
    filteredLidarData.timestamp = timedLidarData.timestamp;

    for (const auto &node : timedLidarData.lidarData) {
        if (node.distance >= minDistance) {
            filteredLidarData.lidarData.push_back(node);
        }
    }

    return filteredLidarData;
}

std::vector<LineSegment> getLines(
    const TimedLidarData &timedLidarData,
    const RobotDeltaPose &robotDeltaPose,
    float splitThreshold,
    int minPoints,
    float maxPointGap,
    float minLength,
    float mergeAngleThreshold,
    float mergeGapThreshold
) {
    // Convert polar to Cartesian (in meters)
    std::vector<cv::Point2f> points;
    points.reserve(timedLidarData.lidarData.size());

    for (const auto &node : timedLidarData.lidarData) {
        // TODO: Change this value to fit the actual robot
        if (node.distance < 0.005) continue;
        if (node.distance > 3.200) continue;
        if (node.angle > 5 && node.angle < 175 && node.distance > 0.700) continue;

        float rad = node.angle * static_cast<float>(M_PI) / 180.0f;

        float lidarX = node.distance * std::sin(rad);
        float lidarY = node.distance * std::cos(rad);
        float x, y;
        lidarToCartesian(lidarX, lidarY, x, y);

        points.emplace_back(x, y);
    }

    std::vector<LineSegment> rawSegments;
    if (!points.empty()) {
        splitSegment(points, 0, points.size() - 1, rawSegments, splitThreshold, minPoints, maxPointGap, minLength);
    }

    auto mergedSegments = mergeSegments(rawSegments, mergeAngleThreshold, mergeGapThreshold);

    // Apply delta transform: translate (-deltaX, -deltaY) and rotate (-deltaH)
    float radH = robotDeltaPose.deltaH * static_cast<float>(M_PI) / 180.0f;
    float cosH = std::cos(radH);
    float sinH = std::sin(radH);

    for (auto &seg : mergedSegments) {
        // Translate
        float x1t = seg.x1 - robotDeltaPose.deltaX;
        float y1t = seg.y1 - robotDeltaPose.deltaY;
        float x2t = seg.x2 - robotDeltaPose.deltaX;
        float y2t = seg.y2 - robotDeltaPose.deltaY;

        // Rotate around (0,0)
        seg.x1 = x1t * cosH - y1t * sinH;
        seg.y1 = x1t * sinH + y1t * cosH;
        seg.x2 = x2t * cosH - y2t * sinH;
        seg.y2 = x2t * sinH + y2t * cosH;
    }

    return mergedSegments;
}

RelativeWalls getRelativeWalls(
    const std::vector<LineSegment> &lineSegments,
    Direction targetDirection,
    float heading,
    float minLength,
    float angleThresholdDeg,
    float collinearThreshold
) {
    std::vector<LineSegment> filteredSegments;

    for (const auto &segment : lineSegments) {
        float length = std::hypot(segment.x2 - segment.x1, segment.y2 - segment.y1);
        if (length >= minLength) {
            filteredSegments.push_back(segment);
        }
    }

    std::vector<LineSegment> mergedSegments = mergeAlignedSegments(filteredSegments, angleThresholdDeg, collinearThreshold);

    RelativeWalls relativeWalls;

    for (const auto &segment : mergedSegments) {
        // Angle of the segment’s perpendicular relative to the robot’s forward direction
        float perpAngleRobotFrame = segment.perpendicularDirection(0.0f, 0.0f);

        // Angle of the segment’s perpendicular relative to the target direction frame
        float perpAngleTargetFrame = std::fmod(perpAngleRobotFrame - (heading - targetDirection.toHeading()) + 360.0f, 360.0f);

        float perpDistance = segment.perpendicularDistance(0.0f, 0.0f);

        if (perpAngleTargetFrame >= 315.0f || perpAngleTargetFrame < 45.0f) {
            relativeWalls.rightWalls.push_back(segment);
        } else if (perpAngleTargetFrame >= 45.0f && perpAngleTargetFrame < 135.0f) {
            relativeWalls.frontWalls.push_back(segment);
        } else if (perpAngleTargetFrame >= 135.0f && perpAngleTargetFrame < 225.0f) {
            relativeWalls.leftWalls.push_back(segment);
        } else {
            relativeWalls.backWalls.push_back(segment);
        }
    }

    return relativeWalls;
}

std::optional<RotationDirection> getTurnDirection(const RelativeWalls &walls) {
    if (walls.frontWalls.empty()) return std::nullopt;
    if (walls.leftWalls.empty() && walls.rightWalls.empty()) return std::nullopt;

    // Pick the highest front line
    const LineSegment *frontLine = &walls.frontWalls[0];
    float frontMidY = (frontLine->y1 + frontLine->y2) / 2.0f;
    for (const auto &line : walls.frontWalls) {
        float midY = (line.y1 + line.y2) / 2.0f;
        if (midY > frontMidY) {
            frontLine = &line;
            frontMidY = midY;
        }
    }

    // Determine left and right points of the front line
    float frontLeftX, frontLeftY, frontRightX, frontRightY;
    if (frontLine->x1 < frontLine->x2) {
        frontLeftX = frontLine->x1;
        frontLeftY = frontLine->y1;
        frontRightX = frontLine->x2;
        frontRightY = frontLine->y2;
    } else {
        frontLeftX = frontLine->x2;
        frontLeftY = frontLine->y2;
        frontRightX = frontLine->x1;
        frontRightY = frontLine->y1;
    }

    // Check left walls
    for (const auto &leftLine : walls.leftWalls) {
        float leftHigherX, leftHigherY;
        if (leftLine.y1 < leftLine.y2) {
            leftHigherX = leftLine.x1;
            leftHigherY = leftLine.y1;
        } else {
            leftHigherX = leftLine.x2;
            leftHigherY = leftLine.y2;
        }

        // Check for left wall that is far away in x direction from front wall
        float dir = leftLine.perpendicularDirection(frontLeftX, frontLeftY);
        if (dir > 90.0f && dir < 270.0f) {
            if (leftLine.perpendicularDistance(0.0f, 0.0f) > 1.70f) return RotationDirection::COUNTER_CLOCKWISE;

            continue;
        }

        if (frontLine->perpendicularDistance(leftHigherX, leftHigherY) < 0.30) return RotationDirection::CLOCKWISE;

        if (leftLine.perpendicularDistance(frontLeftX, frontLeftY) > 0.30f) {
            float dir = leftLine.perpendicularDirection(frontLeftX, frontLeftY);
            if (dir > 270.0f || dir < 90.0f) return RotationDirection::COUNTER_CLOCKWISE;
        }
    }

    // Check right walls
    for (const auto &rightLine : walls.rightWalls) {
        float rightHigherX, rightHigherY;
        if (rightLine.y1 < rightLine.y2) {
            rightHigherX = rightLine.x1;
            rightHigherY = rightLine.y1;
        } else {
            rightHigherX = rightLine.x2;
            rightHigherY = rightLine.y2;
        }

        // Check for right wall that is far away in x direction from front wall
        float dir = rightLine.perpendicularDirection(frontRightX, frontRightY);
        if (dir > 270.0f || dir < 90.0f) {
            if (rightLine.perpendicularDistance(0.0f, 0.0f) > 1.70f) return RotationDirection::CLOCKWISE;

            continue;
        }

        if (frontLine->perpendicularDistance(rightHigherX, rightHigherY) < 0.30) return RotationDirection::COUNTER_CLOCKWISE;

        if (rightLine.perpendicularDistance(frontRightX, frontRightY) > 0.30f) {
            float dir = rightLine.perpendicularDirection(frontRightX, frontRightY);
            if (dir > 90.0f && dir < 270.0f) return RotationDirection::CLOCKWISE;
        }
    }

    return std::nullopt;  // unknown if no rule matched
}

ResolvedWalls resolveWalls(const RelativeWalls &relativeWalls) {
    ResolvedWalls resolveWalls;

    for (auto &newWall : relativeWalls.leftWalls) {
        float newDist = newWall.perpendicularDistance(0.0f, 0.0f);
        if (newDist > 1.20f) continue;
        if (resolveWalls.leftWall.has_value()) {
            LineSegment curWall = resolveWalls.leftWall.value();
            float curDist = curWall.perpendicularDistance(0.0f, 0.0f);
            if (curDist <= newDist) continue;
        }
        resolveWalls.leftWall = newWall;
    }
    for (auto &newWall : relativeWalls.rightWalls) {
        float newDist = newWall.perpendicularDistance(0.0f, 0.0f);
        if (newDist > 1.20f) continue;
        if (resolveWalls.rightWall.has_value()) {
            LineSegment curWall = resolveWalls.rightWall.value();
            float curDist = curWall.perpendicularDistance(0.0f, 0.0f);
            if (curDist <= newDist) continue;
        }
        resolveWalls.rightWall = newWall;
    }
    for (auto &newWall : relativeWalls.frontWalls) {
        float newDist = newWall.perpendicularDistance(0.0f, 0.0f);
        if (resolveWalls.frontWall.has_value()) {
            LineSegment curWall = resolveWalls.frontWall.value();
            float curDist = curWall.perpendicularDistance(0.0f, 0.0f);
            if (curDist >= newDist) continue;
        }
        resolveWalls.frontWall = newWall;
    }
    for (auto &newWall : relativeWalls.backWalls) {
        float newDist = newWall.perpendicularDistance(0.0f, 0.0f);
        if (resolveWalls.backWall.has_value()) {
            LineSegment curWall = resolveWalls.backWall.value();
            float curDist = curWall.perpendicularDistance(0.0f, 0.0f);
            if (curDist >= newDist) continue;
        }
        resolveWalls.backWall = newWall;
    }

    for (auto &newWall : relativeWalls.leftWalls) {
        float newDist = newWall.perpendicularDistance(0.0f, 0.0f);
        if (newDist <= 1.20f) continue;
        if (newDist >= 3.20f) continue;
        if (resolveWalls.farLeftWall.has_value()) {
            LineSegment curWall = resolveWalls.farLeftWall.value();
            float curDist = curWall.perpendicularDistance(0.0f, 0.0f);
            if (curDist >= newDist) continue;
        }
        resolveWalls.farLeftWall = newWall;
    }
    for (auto &newWall : relativeWalls.rightWalls) {
        float newDist = newWall.perpendicularDistance(0.0f, 0.0f);
        if (newDist <= 1.20f) continue;
        if (newDist >= 3.20f) continue;
        if (resolveWalls.farRightWall.has_value()) {
            LineSegment curWall = resolveWalls.farRightWall.value();
            float curDist = curWall.perpendicularDistance(0.0f, 0.0f);
            if (curDist >= newDist) continue;
        }
        resolveWalls.farRightWall = newWall;
    }

    return resolveWalls;
}

std::vector<LineSegment> getParkingWalls(
    const std::vector<LineSegment> &lineSegments,
    Direction targetDirection,
    float heading,
    float maxLength
) {
    std::vector<LineSegment> filteredSegments;

    for (const auto &segment : lineSegments) {
        // Angle of the segment’s perpendicular relative to the robot’s forward direction
        float perpAngleRobotFrame = segment.perpendicularDirection(0.0f, 0.0f);

        // Angle of the segment’s perpendicular relative to the target direction frame
        float perpAngleTargetFrame = std::fmod(perpAngleRobotFrame - (heading - targetDirection.toHeading()) + 360.0f, 360.0f);

        if (not((perpAngleTargetFrame > 85.0f && perpAngleTargetFrame < 95.0f) ||
                (perpAngleTargetFrame > 265.0f && perpAngleTargetFrame < 285.0f)))
            continue;

        if (segment.length() <= maxLength) {
            filteredSegments.push_back(segment);
        }
    }

    return filteredSegments;
}

std::vector<cv::Point2f> getTrafficLightPoints(
    const TimedLidarData &timedLidarData,
    const ResolvedWalls &resolveWalls,
    std::optional<RotationDirection> turnDirection,
    float distanceThreshold,
    size_t minClusterSize
) {
    // Convert polar to Cartesian (in meters)
    std::vector<cv::Point2f> points;
    points.reserve(timedLidarData.lidarData.size());

    for (const auto &node : timedLidarData.lidarData) {
        // TODO: Change this value to fit the actual robot
        if (node.distance < 0.005) continue;
        if (node.distance > 3.200) continue;
        if (node.angle > 5 && node.angle < 175 && node.distance > 0.700) continue;

        float rad = node.angle * static_cast<float>(M_PI) / 180.0f;

        float lidarX = node.distance * std::sin(rad);
        float lidarY = node.distance * std::cos(rad);
        float x, y;
        lidarToCartesian(lidarX, lidarY, x, y);

        points.emplace_back(x, y);
    }

    std::vector<cv::Point2f> filteredPoints;
    for (auto &point : points) {
        if (not resolveWalls.frontWall) return {};
        float frontDistance = resolveWalls.frontWall->perpendicularDistance(point.x, point.y);

        std::optional<LineSegment> outerWall, innerWall, farOuterWall;
        if (turnDirection.value_or(RotationDirection::CLOCKWISE) == RotationDirection::CLOCKWISE) {
            outerWall = resolveWalls.leftWall;
            innerWall = resolveWalls.rightWall;

            farOuterWall = resolveWalls.farRightWall;
        } else {
            outerWall = resolveWalls.rightWall;
            innerWall = resolveWalls.leftWall;

            farOuterWall = resolveWalls.farLeftWall;
        }

        float outerDistance;
        if (outerWall) {
            outerDistance = outerWall->perpendicularDistance(point.x, point.y);
        } else if (innerWall) {
            outerDistance = 1.00f - innerWall->perpendicularDistance(point.x, point.y);
        } else {
            return {};
        }

        // TODO: Clean up this magic number
        const float outerEdge = 0.30f;
        const float innerEdge = 0.70f;

        if (farOuterWall) {
            float outerFarDistance = farOuterWall->perpendicularDistance(point.x, point.y);

            if (frontDistance < outerEdge or frontDistance > 3.00f - outerEdge or outerDistance < outerEdge or outerFarDistance < outerEdge)
                continue;
            if (frontDistance > innerEdge and outerDistance > innerEdge and outerFarDistance > innerEdge) continue;
        } else {
            if (frontDistance < outerEdge or frontDistance > 3.00f - outerEdge or outerDistance < outerEdge or
                outerDistance > 3.00f - outerEdge)
                continue;
            if (frontDistance > innerEdge and outerDistance > innerEdge) continue;
        }

        filteredPoints.push_back(point);
    }

    std::vector<cv::Point2f> averages;
    if (filteredPoints.empty()) return averages;

    std::vector<bool> visited(filteredPoints.size(), false);

    for (size_t i = 0; i < filteredPoints.size(); ++i) {
        if (visited[i]) continue;

        std::vector<cv::Point2f> currentCluster;
        currentCluster.push_back(filteredPoints[i]);
        visited[i] = true;

        size_t idx = 0;
        while (idx < currentCluster.size()) {
            cv::Point2f p = currentCluster[idx];

            for (size_t j = 0; j < filteredPoints.size(); ++j) {
                if (visited[j]) continue;

                float dist = std::hypot(p.x - filteredPoints[j].x, p.y - filteredPoints[j].y);
                if (dist < distanceThreshold) {
                    currentCluster.push_back(filteredPoints[j]);
                    visited[j] = true;
                }
            }
            ++idx;
        }

        if (currentCluster.size() >= minClusterSize) {
            // compute average
            float sumX = 0, sumY = 0;
            for (auto &pt : currentCluster) {
                sumX += pt.x;
                sumY += pt.y;
            }
            averages.emplace_back(sumX / currentCluster.size(), sumY / currentCluster.size());
        }
    }

    return averages;
}

void drawLidarData(cv::Mat &img, const TimedLidarData &timedLidarDatas, float scale) {
    CV_Assert(!img.empty());
    CV_Assert(img.type() == CV_8UC3);  // make sure it's a 3-channel color image

    cv::Point center(img.cols / 2, img.rows / 2);

    for (const auto &node : timedLidarDatas.lidarData) {
        if (node.distance <= 0) continue;

        float rad = node.angle * static_cast<float>(CV_PI) / 180.0f;

        float lidarX = node.distance * std::sin(rad);
        float lidarY = node.distance * std::cos(rad);
        float x, y;
        lidarToCartesian(lidarX, lidarY, x, y);

        int cvX = static_cast<int>(center.x + x * (img.rows / scale));
        int cvY = static_cast<int>(center.y - y * (img.rows / scale));

        if (cvX >= 0 && cvX < img.cols && cvY >= 0 && cvY < img.rows) {
            cv::circle(img, cv::Point(cvX, cvY), 1, cv::Scalar(255, 255, 255), -1);
        }
    }

    // Draw LiDAR origin as an isosceles triangle
    std::vector<cv::Point> triangle;

    // size of the triangle in pixels
    int size = 12;

    // pointing upwards
    triangle.push_back(cv::Point(center.x, center.y - size));                 // top vertex
    triangle.push_back(cv::Point(center.x - size / 2, center.y + size / 2));  // bottom-left
    triangle.push_back(cv::Point(center.x + size / 2, center.y + size / 2));  // bottom-right

    cv::fillConvexPoly(img, triangle, cv::Scalar(173, 12, 168));
}

void drawLineSegment(cv::Mat &img, const LineSegment &segment, float scale, cv::Scalar color, int thickness) {
    CV_Assert(!img.empty());
    CV_Assert(img.type() == CV_8UC3);  // ensure 3-channel image

    cv::Point center(img.cols / 2, img.rows / 2);

    int x1 = static_cast<int>(center.x + segment.x1 * (img.cols / scale));
    int y1 = static_cast<int>(center.y - segment.y1 * (img.rows / scale));
    int x2 = static_cast<int>(center.x + segment.x2 * (img.cols / scale));
    int y2 = static_cast<int>(center.y - segment.y2 * (img.rows / scale));

    cv::line(img, cv::Point(x1, y1), cv::Point(x2, y2), color, thickness);
}

void drawTrafficLightPoint(cv::Mat &img, const cv::Point2f &point, float scale, cv::Scalar color, int radius) {
    CV_Assert(!img.empty());
    CV_Assert(img.type() == CV_8UC3);  // ensure 3-channel image

    cv::Point center(img.cols / 2, img.rows / 2);

    int x = static_cast<int>(center.x + point.x * (img.cols / scale));
    int y = static_cast<int>(center.y - point.y * (img.rows / scale));

    cv::circle(img, cv::Point(x, y), radius, color, cv::FILLED);
}

}  // namespace lidar_processor
