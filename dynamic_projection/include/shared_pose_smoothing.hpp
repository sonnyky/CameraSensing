#pragma once
#include <opencv2/calib3d.hpp>
#include <algorithm>

namespace Tinker {
inline void smooth_shared_board_pose(const cv::Mat& rotation, const cv::Mat& translation,
                                    double alpha, cv::Mat& previousRotation, cv::Mat& previousTranslation) {
    alpha=std::clamp(alpha,0.0,1.0);
    if (previousRotation.empty() || previousTranslation.empty() || alpha==1) {
        previousRotation=rotation.clone(); previousTranslation=translation.clone(); return;
    }
    cv::Mat oldMatrix, newMatrix, relativeVector, increment, updated;
    cv::Rodrigues(previousRotation,oldMatrix); cv::Rodrigues(rotation,newMatrix);
    cv::Rodrigues(newMatrix*oldMatrix.t(),relativeVector);
    cv::Rodrigues(relativeVector*alpha,increment);
    updated=increment*oldMatrix;
    cv::Rodrigues(updated,previousRotation);
    previousTranslation=previousTranslation*(1-alpha)+translation*alpha;
}
}
