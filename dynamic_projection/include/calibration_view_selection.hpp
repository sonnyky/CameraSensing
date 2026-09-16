#pragma once

#include <opencv2/core.hpp>
#include <cstddef>
#include <vector>

namespace Tinker {

	struct CalibrationViewSelection {
		std::vector<size_t> indices;
		double robustRmsThreshold = 0.0;
		double positionSpan = 0.0;
		double distanceRatio = 1.0;
		double orientationSpanDegrees = 0.0;
		bool hasEnoughQualityViews = false;
		bool hasRequiredCoverage = false;
	};

	CalibrationViewSelection select_calibration_views(
		const std::vector<float>& perViewRms,
		const std::vector<cv::Mat>& boardRotations,
		const std::vector<cv::Mat>& boardTranslations,
		const std::vector<std::vector<cv::Point2f>>& measuredImagePoints,
		cv::Size imageSize,
		size_t requiredViews,
		double maximumPerViewRms,
		double minimumPositionSpan,
		double minimumDistanceRatio,
		double minimumOrientationSpanDegrees);
}
