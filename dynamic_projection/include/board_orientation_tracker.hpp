#pragma once
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace Tinker {
    struct BoardOrientationMarker { int id; cv::Point2f center; };
    class BoardOrientationTracker {
    public:
        bool orient(std::vector<cv::Point2f>& corners, cv::Size boardSize,
            const std::vector<BoardOrientationMarker>& markers, std::string& status);
        void reset() { previous.clear(); }
    private:
        std::vector<cv::Point2f> previous;
    };
}
