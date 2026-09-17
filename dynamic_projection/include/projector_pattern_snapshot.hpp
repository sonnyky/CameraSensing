#pragma once
#include <opencv2/core.hpp>
#include <vector>

namespace Tinker {
    class ProjectorPatternSnapshot {
    public:
        void record(const std::vector<cv::Point2f>& queuedPoints, bool hasPattern) {
            displayed = hasPattern ? queuedPoints : std::vector<cv::Point2f>{};
        }
        const std::vector<cv::Point2f>& points() const { return displayed; }
    private:
        std::vector<cv::Point2f> displayed;
    };
}
