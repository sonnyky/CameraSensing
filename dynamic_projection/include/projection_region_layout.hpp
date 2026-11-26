#pragma once
#include <opencv2/core.hpp>
#include <cmath>
#include <stdexcept>

namespace Tinker {
    inline cv::Point3f centered_checkerboard_grid_origin(cv::Point3f origin, cv::Point3f axisX, cv::Point3f axisY,
        cv::Size boardSize, cv::Size gridSize)
    {
        const double xSpacing = cv::norm(axisX), ySpacing = cv::norm(axisY);
        if (xSpacing <= 0 || ySpacing <= 0 || boardSize.width < 2 || boardSize.height < 2 ||
            gridSize.width < 1 || gridSize.height < 1) {
            throw std::invalid_argument("checkerboard/grid dimensions or spacing are invalid");
        }

        // Checkerboard square centers range from -0.5 to width-0.5 (and
        // height-0.5) relative to the first inner corner. Keep every projected
        // center on a square center; the asymmetric row stagger then preserves
        // square color across all grid points.
        const double centeredX = ((boardSize.width - 1) - (2 * gridSize.width - 1)) / 2.0;
        const double centeredY = ((boardSize.height - 1) - (gridSize.height - 1)) / 2.0;
        const auto nearestHalf = [](double value) { return std::floor(value) + 0.5; };
        const double x0 = nearestHalf(centeredX);
        const double y0 = nearestHalf(centeredY);
        const double maxX = x0 + 2 * (gridSize.width - 1) + ((gridSize.height > 1) ? 1 : 0);
        const double maxY = y0 + gridSize.height - 1;
        if (x0 < -0.5 || y0 < -0.5 || maxX > boardSize.width - 0.5 || maxY > boardSize.height - 0.5) {
            throw std::invalid_argument("dynamic grid cannot fit on checkerboard square centers");
        }
        return origin + axisX * static_cast<float>(x0) + axisY * static_cast<float>(y0);
    }

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
