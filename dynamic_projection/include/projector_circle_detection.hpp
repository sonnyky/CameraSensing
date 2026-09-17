#pragma once

#include <opencv2/features2d.hpp>
#include <vector>

namespace Tinker {
    cv::Mat threshold_projector_circles(const cv::Mat& image);
    bool detect_projector_circles(const cv::Mat& thresholdedImage, cv::Size patternSize,
        std::vector<cv::Point2f>& centers, std::vector<cv::KeyPoint>& blobs);
}
