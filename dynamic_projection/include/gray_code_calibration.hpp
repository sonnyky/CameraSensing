#pragma once

#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace Tinker {
	struct GrayCodeDecodeDiagnostics {
		size_t total=0;
		size_t coarseGrayValid=0;
		size_t phaseValid=0;
		size_t grayPhaseConsistent=0;
		size_t accepted=0;
	};

	int gray_code_bit_count(int extent);
	std::vector<cv::Mat> make_gray_code_patterns(cv::Size projectorSize,int projectorBlockSize=1,int phasePeriod=0);
	bool decode_gray_code_samples(const std::vector<cv::Mat>& capturedGrayFrames,
		cv::Size projectorSize,
		int projectorBlockSize,
		int phasePeriod,
		const std::vector<cv::Point2f>& cameraSamples,
		const std::vector<cv::Point3f>& boardSamples,
		double minimumDirectContrast,
		double minimumBitContrast,
		int sampleRadius,
		std::vector<cv::Point2f>& decodedProjectorPoints,
		std::vector<cv::Point2f>& acceptedCameraSamples,
		std::vector<cv::Point3f>& acceptedBoardSamples,
		cv::Mat* debugImage,
		std::string& reason,
		GrayCodeDecodeDiagnostics* diagnostics=nullptr);
}
