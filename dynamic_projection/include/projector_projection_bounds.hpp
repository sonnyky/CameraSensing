#pragma once

#include <opencv2/core.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace Tinker {
    struct ProjectorProjectionBounds {
        size_t finiteCenters = 0;
        size_t fullyVisibleCircles = 0;
        size_t intersectingCircles = 0;
        double minX = std::numeric_limits<double>::infinity();
        double minY = std::numeric_limits<double>::infinity();
        double maxX = -std::numeric_limits<double>::infinity();
        double maxY = -std::numeric_limits<double>::infinity();
    };

    inline ProjectorProjectionBounds projector_projection_bounds(
        const std::vector<cv::Point2f>& points, cv::Size imageSize, double radius)
    {
        ProjectorProjectionBounds result;
        for (const auto& point : points) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y)) continue;
            ++result.finiteCenters;
            result.minX = std::min(result.minX, static_cast<double>(point.x));
            result.minY = std::min(result.minY, static_cast<double>(point.y));
            result.maxX = std::max(result.maxX, static_cast<double>(point.x));
            result.maxY = std::max(result.maxY, static_cast<double>(point.y));
            if (point.x - radius >= 0 && point.y - radius >= 0 &&
                point.x + radius < imageSize.width && point.y + radius < imageSize.height) {
                ++result.fullyVisibleCircles;
            }
            // Bounding-box intersection, not a guarantee of disk visibility at corners.
            if (point.x + radius >= 0 && point.y + radius >= 0 &&
                point.x - radius < imageSize.width && point.y - radius < imageSize.height) {
                ++result.intersectingCircles;
            }
        }
        return result;
    }
}
