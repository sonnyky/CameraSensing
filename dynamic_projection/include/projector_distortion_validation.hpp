#pragma once
#include <opencv2/core.hpp>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace Tinker {
inline double minimum_radial_polynomial(double a, double b, double c, double end) {
    auto value = [=](double t) { return 1 + t * (a + t * (b + t*c)); };
    double result = std::min(value(0), value(end));
    auto examine = [&](double t) { if (t > 0 && t < end) result = std::min(result, value(t)); };
    if (std::abs(c) < 1e-15) { if (std::abs(b) > 1e-15) examine(-a/(2*b)); }
    else {
        const double discriminant = 4*b*b - 12*c*a;
        if (discriminant >= 0) {
            examine((-2*b + std::sqrt(discriminant))/(6*c));
            examine((-2*b - std::sqrt(discriminant))/(6*c));
        }
    }
    return result;
}

// Validate the 4/5-coefficient pinhole model over a conservative disk enclosing
// the projector image in normalized coordinates, plus 25% radial margin.
inline bool valid_projector_distortion(const cv::Mat& matrix, const cv::Mat& distortion,
                                      cv::Size imageSize, std::string& reason) {
    if (matrix.rows != 3 || matrix.cols != 3 || matrix.channels() != 1 ||
        distortion.channels() != 1 || (distortion.total() != 4 && distortion.total() != 5) ||
        imageSize.width <= 0 || imageSize.height <= 0 ||
        !cv::checkRange(matrix) || !cv::checkRange(distortion)) {
        reason = "invalid intrinsics or unsupported distortion model"; return false;
    }
    cv::Mat k, d;
    matrix.convertTo(k, CV_64F); distortion.reshape(1, 1).convertTo(d, CV_64F);
    const double fx=k.at<double>(0,0), fy=k.at<double>(1,1), cx=k.at<double>(0,2), cy=k.at<double>(1,2);
    if (fx <= 0 || fy <= 0) { reason="non-positive focal length"; return false; }
    double radius=0;
    for (double x : {0.0, double(imageSize.width-1)}) for (double y : {0.0, double(imageSize.height-1)})
        radius=std::max(radius, std::hypot((x-cx)/fx, (y-cy)/fy));
    radius *= 1.25;
    const double k1=d.at<double>(0,0), k2=d.at<double>(0,1), p1=d.at<double>(0,2), p2=d.at<double>(0,3);
    const double k3=d.total()==5 ? d.at<double>(0,4) : 0;
    if (!std::isfinite(radius) || minimum_radial_polynomial(k1,k2,k3,radius*radius) <= 1e-6 ||
        minimum_radial_polynomial(3*k1,5*k2,7*k3,radius*radius) <= 1e-6) {
        reason="radial mapping folds or reverses within the projector validation domain"; return false;
    }
    // Sample the full radial+tangential Jacobian as an additional fold check.
    for (int r=0;r<=16;++r) for (int angle=0;angle<64;++angle) {
        const double x=radius*r/16*std::cos(angle*2*CV_PI/64), y=radius*r/16*std::sin(angle*2*CV_PI/64);
        const double t=x*x+y*y, scale=1+t*(k1+t*(k2+t*k3)), slope=k1+t*(2*k2+3*k3*t);
        const double xx=scale+2*x*x*slope+2*p1*y+6*p2*x;
        const double yy=scale+2*y*y*slope+6*p1*y+2*p2*x;
        const double xy=2*x*y*slope+2*p1*x+2*p2*y;
        if (!std::isfinite(xx*yy-xy*xy) || xx <= 1e-6 || xx*yy-xy*xy <= 1e-6) {
            reason="radial/tangential mapping folds at a sampled projector coordinate"; return false;
        }
    }
    reason.clear(); return true;
}
}
