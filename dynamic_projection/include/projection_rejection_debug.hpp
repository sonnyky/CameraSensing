#pragma once
#include <opencv2/core.hpp>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace Tinker {
struct ProjectionCircleDebug {
    cv::Point3d projectorMm{NAN,NAN,NAN};
    double normalizedRadius=std::numeric_limits<double>::quiet_NaN();
};
struct ProjectionResult {
    std::vector<cv::Point2f> points;
    std::vector<ProjectionCircleDebug> circles;
    std::string rejectionReason;
    size_t offendingCircle=0;
    double allowedRadius=std::numeric_limits<double>::quiet_NaN();
};
inline std::string projection_circle_debug(const ProjectionResult& result, size_t index,
                                         cv::Size pattern, const std::string& reason) {
    std::ostringstream text;
    text << std::fixed << std::setprecision(4) << "reason=" << reason;
    if (result.circles.empty()) text << "; circle=N/A";
    else {
        text << "; circle=" << index+1 << '/' << result.circles.size();
        if (pattern.width>0) text << "; row=" << index/pattern.width+1 << "; column=" << index%pattern.width+1;
    }
    if (index<result.circles.size()) {
        const auto& circle=result.circles[index];
        text << "; projector_xyz_mm=(" << circle.projectorMm.x << ',' << circle.projectorMm.y << ',' << circle.projectorMm.z
             << "); depth_mm=" << circle.projectorMm.z << "; normalized_radius=";
        if (std::isfinite(circle.normalizedRadius)) text << circle.normalizedRadius;
        else text << "N/A";
    }
    text << "; allowed_radius=" << result.allowedRadius;
    if (index<result.points.size()) text << "; pixel_center=(" << result.points[index].x << ',' << result.points[index].y << ')';
    return text.str();
}
inline std::string clipped_circle_edges(cv::Point2f point, cv::Size image, double radius) {
    std::string edges;
    auto add=[&](const char* edge) { if (!edges.empty()) edges+=','; edges+=edge; };
    if (point.x-radius<0) add("left");
    if (point.x+radius>=image.width) add("right");
    if (point.y-radius<0) add("top");
    if (point.y+radius>=image.height) add("bottom");
    return edges;
}
}
