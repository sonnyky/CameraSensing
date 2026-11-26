#include "projector_circle_detection.hpp"
#include "flags.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

cv::Mat Tinker::threshold_projector_circles(const cv::Mat& image)
{
    cv::Mat gray, thresholded;
    if (image.channels() == 1) gray = image;
    else cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    const bool automatic = FLAGS_projector_detection_threshold < 0;
    cv::threshold(gray, thresholded, automatic ? 0 : FLAGS_projector_detection_threshold, 255,
        cv::THRESH_BINARY_INV | (automatic ? cv::THRESH_OTSU : 0));
    return thresholded;
}

bool Tinker::detect_projector_circles(const cv::Mat& thresholdedImage, cv::Size patternSize,
    std::vector<cv::Point2f>& centers, std::vector<cv::KeyPoint>& blobs)
{
    cv::SimpleBlobDetector::Params params;
    params.filterByColor = true;
    params.blobColor = 0; // Projected bright circles become dark after inversion.
    params.filterByArea = true;
    params.minArea = static_cast<float>(FLAGS_projector_blob_min_area);
    params.maxArea = static_cast<float>(FLAGS_projector_blob_max_area);
    params.filterByCircularity = FLAGS_projector_blob_min_circularity > 0;
    if (params.filterByCircularity) params.minCircularity = static_cast<float>(FLAGS_projector_blob_min_circularity);
    params.filterByInertia = FLAGS_projector_blob_min_inertia > 0;
    if (params.filterByInertia) params.minInertiaRatio = static_cast<float>(FLAGS_projector_blob_min_inertia);
    params.filterByConvexity = FLAGS_projector_blob_min_convexity > 0;
    if (params.filterByConvexity) params.minConvexity = static_cast<float>(FLAGS_projector_blob_min_convexity);
    const auto detector = cv::SimpleBlobDetector::create(params);
    detector->detect(thresholdedImage, blobs);
    centers.clear();
    return cv::findCirclesGrid(thresholdedImage, patternSize, centers,
        cv::CALIB_CB_ASYMMETRIC_GRID, detector);
}
