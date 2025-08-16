#include "lidar_processor.h"

namespace lidar_processor
{

namespace
{

    // --- Helper: Perpendicular distance from point to line ---
    float perpendicularDistance(float x, float y, float x1, float y1, float x2, float y2) {
        float num = std::fabs((y2 - y1) * x - (x2 - x1) * y + x2 * y1 - y2 * x1);
        float den = std::hypot(x2 - x1, y2 - y1);
        return (den > 1e-6f) ? num / den : 0.0f;
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

                    if (perpendicularDistance(otherStart.x, otherStart.y, current.x1, current.y1, current.x2, current.y2) >
                            collinearThreshold &&
                        perpendicularDistance(otherEnd.x, otherEnd.y, current.x1, current.y1, current.x2, current.y2) > collinearThreshold)
                    {
                        // Check intermediate points
                        bool aligned = false;
                        int numIntermediatePoints = 10;
                        for (int k = 1; k < numIntermediatePoints; ++k) {
                            float t = float(k) / float(numIntermediatePoints - 1);
                            cv::Point2f pt = otherStart + t * (otherEnd - otherStart);
                            if (perpendicularDistance(pt.x, pt.y, current.x1, current.y1, current.x2, current.y2) <= collinearThreshold) {
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

std::vector<LineSegment> getLines(
    const TimedLidarData &timedLidarData,
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
        if (node.distance <= 0) continue;
        float rad = node.angle * static_cast<float>(M_PI) / 180.0f;
        points.emplace_back(node.distance * std::cos(rad), -node.distance * std::sin(rad));
    }

    std::vector<LineSegment> rawSegments;
    if (!points.empty()) {
        splitSegment(points, 0, points.size() - 1, rawSegments, splitThreshold, minPoints, maxPointGap, minLength);
    }

    return mergeSegments(rawSegments, mergeAngleThreshold, mergeGapThreshold);
}

std::vector<LineSegment> getWalls(
    const std::vector<LineSegment> &lineSegments,
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

    std::vector<LineSegment> verticalSegments;
    std::vector<LineSegment> horizontalSegments;

    for (const auto &segment : mergedSegments) {
        float dx = segment.x2 - segment.x1;
        float dy = segment.y2 - segment.y1;
        float segmentAngle = std::atan2(dy, dx) * 180.0f / M_PI;  // angle in degrees

        // Normalize angles to [0, 360)
        if (segmentAngle < 0) segmentAngle += 360.0f;
        float headingNorm = fmod(heading, 360.0f);

        float angleDiff = std::fabs(segmentAngle - headingNorm);
        if (angleDiff > 180.0f) angleDiff = 360.0f - angleDiff;

        if (angleDiff <= 45.0f) {
            // close to heading -> horizontal
            horizontalSegments.push_back(segment);
        } else if (angleDiff >= 135.0f) {
            // perpendicular -> vertical
            verticalSegments.push_back(segment);
        }
        // else ignore (diagonal)
    }
}

std::vector<LineSegment> getParkingWalls(const std::vector<LineSegment> &lineSegments, float maxLength) {
    // TODO: Filter the wall out
    std::vector<LineSegment> filteredSegments;

    for (const auto &segment : lineSegments) {
        float length = std::hypot(segment.x2 - segment.x1, segment.y2 - segment.y1);
        if (length <= maxLength) {
            filteredSegments.push_back(segment);
        }
    }

    return filteredSegments;
}

void drawLidarData(cv::Mat &img, const TimedLidarData &timedLidarDatas, float scale) {
    CV_Assert(!img.empty());
    CV_Assert(img.type() == CV_8UC3);  // make sure it's a 3-channel color image

    cv::Point center(img.cols / 2, img.rows / 2);

    for (const auto &node : timedLidarDatas.lidarData) {
        if (node.distance <= 0) continue;

        float rad = node.angle * static_cast<float>(CV_PI) / 180.0f;
        int x = static_cast<int>(center.x + node.distance * (img.rows / scale) * std::cos(rad));
        int y = static_cast<int>(center.y + node.distance * (img.rows / scale) * std::sin(rad));

        if (x >= 0 && x < img.cols && y >= 0 && y < img.rows) {
            cv::circle(img, cv::Point(x, y), 1, cv::Scalar(255, 255, 255), -1);
        }
    }

    // Draw LiDAR origin
    cv::circle(img, center, 5, cv::Scalar(168, 12, 173), -1);
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

}  // namespace lidar_processor
