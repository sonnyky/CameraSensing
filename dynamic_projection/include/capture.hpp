#pragma once
#include <opencv2/opencv.hpp>
#include <string>

namespace Tinker {
    class capture {
    public:
        capture(const std::string& type, int index);
        ~capture();
        cv::Mat read();

    private:
        std::string camera_type;
        int camera_index = 0;
        cv::VideoCapture cap;
        cv::Mat frame_image;
    };
}