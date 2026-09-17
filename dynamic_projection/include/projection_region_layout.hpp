#pragma once
#include <opencv2/core.hpp>
#include <cmath>
#include <stdexcept>

namespace Tinker {
    inline cv::Point3f centered_projection_grid_origin(cv::Point3f origin, cv::Point3f axisX, cv::Point3f axisY,
        cv::Size gridSize, double centerXmm, double topMm, double widthMm, double heightMm)
    {
        const double xSpacing = cv::norm(axisX), ySpacing = cv::norm(axisY);
        const double gridWidth = (2 * gridSize.width - 1) * xSpacing;
        const double gridHeight = (gridSize.height - 1) * ySpacing;
        if (xSpacing <= 0 || ySpacing <= 0 || gridWidth >= widthMm || gridHeight >= heightMm) {
            throw std::invalid_argument("dynamic grid is too large for white region, or spacing is invalid");
        }
        return origin + axisX * static_cast<float>((centerXmm - gridWidth / 2) / xSpacing) +
            axisY * static_cast<float>((topMm + (heightMm - gridHeight) / 2) / ySpacing);
    }
}
