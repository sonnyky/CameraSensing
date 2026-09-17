#include "calibration_view_selection.hpp"

#include <opencv2/calib3d.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>

namespace {
	constexpr double radiansToDegrees = 180.0 / CV_PI;

	double median(std::vector<double> values)
	{
		if (values.empty()) {
			return 0.0;
		}
		const size_t middle = values.size() / 2;
		std::nth_element(values.begin(), values.begin() + middle, values.end());
		const double upper = values[middle];
		if ((values.size() % 2) != 0) {
			return upper;
		}
		std::nth_element(values.begin(), values.begin() + middle - 1, values.end());
		return (values[middle - 1] + upper) * 0.5;
	}

	struct ViewDescriptor {
		double centerX;
		double centerY;
		double logDistance;
		cv::Vec3d normal;
		double rollSin;
		double rollCos;
	};

	ViewDescriptor describe_view(
		const cv::Mat& rotationVector,
		const cv::Mat& translationVector,
		const std::vector<cv::Point2f>& imagePoints,
		cv::Size imageSize)
	{
		cv::Point2d center(0.0, 0.0);
		for (const auto& point : imagePoints) {
			center.x += point.x;
			center.y += point.y;
		}
		if (!imagePoints.empty()) {
			center.x /= imagePoints.size();
			center.y /= imagePoints.size();
		}

		cv::Mat rotationMatrix;
		cv::Rodrigues(rotationVector, rotationMatrix);
		rotationMatrix.convertTo(rotationMatrix, CV_64F);
		cv::Vec3d normal(
			rotationMatrix.at<double>(0, 2),
			rotationMatrix.at<double>(1, 2),
			rotationMatrix.at<double>(2, 2));
		normal /= std::max(cv::norm(normal), 1e-12);

		cv::Mat translation64;
		translationVector.convertTo(translation64, CV_64F);
		const double distance = std::max(std::abs(translation64.at<double>(2, 0)), 1e-6);
		const double roll = std::atan2(
			rotationMatrix.at<double>(1, 0),
			rotationMatrix.at<double>(0, 0));

		return {
			center.x / std::max(imageSize.width, 1),
			center.y / std::max(imageSize.height, 1),
			std::log(distance),
			normal,
			std::sin(roll),
			std::cos(roll)
		};
	}

	double descriptor_distance_squared(const ViewDescriptor& a, const ViewDescriptor& b)
	{
		const double dx = a.centerX - b.centerX;
		const double dy = a.centerY - b.centerY;
		const double dz = a.logDistance - b.logDistance;
		const cv::Vec3d dn = a.normal - b.normal;
		const double ds = a.rollSin - b.rollSin;
		const double dc = a.rollCos - b.rollCos;
		return 4.0 * (dx * dx + dy * dy) + dz * dz + dn.dot(dn) + 0.25 * (ds * ds + dc * dc);
	}
}

Tinker::CalibrationViewSelection Tinker::select_calibration_views(
	const std::vector<float>& perViewRms,
	const std::vector<cv::Mat>& boardRotations,
	const std::vector<cv::Mat>& boardTranslations,
	const std::vector<std::vector<cv::Point2f>>& measuredImagePoints,
	cv::Size imageSize,
	size_t requiredViews,
	double maximumPerViewRms,
	double minimumPositionSpan,
	double minimumDistanceRatio,
	double minimumOrientationSpanDegrees)
{
	CalibrationViewSelection result;
	const size_t viewCount = perViewRms.size();
	if (requiredViews == 0 || viewCount != boardRotations.size() ||
		viewCount != boardTranslations.size() || viewCount != measuredImagePoints.size()) {
		return result;
	}

	std::vector<double> errors(perViewRms.begin(), perViewRms.end());
	const double medianError = median(errors);
	std::vector<double> deviations;
	deviations.reserve(errors.size());
	for (double error : errors) {
		deviations.push_back(std::abs(error - medianError));
	}
	const double robustLimit = medianError + 2.5 * median(deviations);
	result.robustRmsThreshold = std::min(maximumPerViewRms, robustLimit);

	std::vector<size_t> eligible;
	for (size_t i = 0; i < viewCount; ++i) {
		if (std::isfinite(perViewRms[i]) && perViewRms[i] <= result.robustRmsThreshold) {
			eligible.push_back(i);
		}
	}
	result.hasEnoughQualityViews = eligible.size() >= requiredViews;
	result.eligibleViewCount = eligible.size();
	if (!result.hasEnoughQualityViews) {
		return result;
	}

	std::vector<ViewDescriptor> descriptors(viewCount);
	for (size_t index : eligible) {
		descriptors[index] = describe_view(
			boardRotations[index], boardTranslations[index], measuredImagePoints[index], imageSize);
	}

	const auto bestError = std::min_element(eligible.begin(), eligible.end(),
		[&perViewRms](size_t a, size_t b) { return perViewRms[a] < perViewRms[b]; });
	result.indices.push_back(*bestError);

	while (result.indices.size() < requiredViews) {
		size_t bestIndex = std::numeric_limits<size_t>::max();
		double bestDistance = -1.0;
		for (size_t candidate : eligible) {
			if (std::find(result.indices.begin(), result.indices.end(), candidate) != result.indices.end()) {
				continue;
			}
			double nearestSelected = std::numeric_limits<double>::infinity();
			for (size_t selected : result.indices) {
				nearestSelected = std::min(nearestSelected,
					descriptor_distance_squared(descriptors[candidate], descriptors[selected]));
			}
			if (nearestSelected > bestDistance + 1e-12 ||
				(std::abs(nearestSelected - bestDistance) <= 1e-12 &&
					(bestIndex == std::numeric_limits<size_t>::max() || perViewRms[candidate] < perViewRms[bestIndex]))) {
				bestDistance = nearestSelected;
				bestIndex = candidate;
			}
		}
		if (bestIndex == std::numeric_limits<size_t>::max()) {
			result.indices.clear();
			return result;
		}
		result.indices.push_back(bestIndex);
	}

	double minimumDistance = std::numeric_limits<double>::infinity();
	double maximumDistance = 0.0;
	for (size_t i = 0; i < result.indices.size(); ++i) {
		const auto& a = descriptors[result.indices[i]];
		const double distance = std::exp(a.logDistance);
		minimumDistance = std::min(minimumDistance, distance);
		maximumDistance = std::max(maximumDistance, distance);
		for (size_t j = i + 1; j < result.indices.size(); ++j) {
			const auto& b = descriptors[result.indices[j]];
			const double dx = a.centerX - b.centerX;
			const double dy = a.centerY - b.centerY;
			result.positionSpan = std::max(result.positionSpan, std::sqrt(dx * dx + dy * dy));
			const double cosine = std::clamp(a.normal.dot(b.normal), -1.0, 1.0);
			result.orientationSpanDegrees = std::max(
				result.orientationSpanDegrees, std::acos(cosine) * radiansToDegrees);
		}
	}
	result.distanceRatio = maximumDistance / std::max(minimumDistance, 1e-6);
	result.hasRequiredCoverage =
		result.positionSpan >= minimumPositionSpan &&
		result.distanceRatio >= minimumDistanceRatio &&
		result.orientationSpanDegrees >= minimumOrientationSpanDegrees;
	return result;
}
