#pragma once
#include <opencv2/calib3d.hpp>
#include <cmath>

namespace Tinker {
    inline bool commit_valid_stereo_fit(double rms, double maximumRms,
        const cv::Mat& rotationMatrix, const cv::Mat& translation,
        cv::Mat& workingRotation, cv::Mat& workingTranslation, double& workingRms)
    {
        if (!std::isfinite(rms) || rms > maximumRms || rotationMatrix.empty() || translation.empty() ||
            !cv::checkRange(rotationMatrix) || !cv::checkRange(translation)) return false;
        cv::Mat rotationVector;
        cv::Rodrigues(rotationMatrix, rotationVector);
        if (!cv::checkRange(rotationVector)) return false;
        workingRotation = rotationVector;
        workingTranslation = translation.clone();
        workingRms = rms;
        return true;
    }
}
