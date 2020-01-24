#ifndef __POSE_DETECTOR__
#define __POSE_DETECTOR__

#pragma once

#include <iostream>
#include <sstream>
#include <librealsense2/rs.hpp> 
#include <Windows.h>
#include <comutil.h>
#include <fstream>

#include <string>
#include <string.h>
#include <tchar.h>
#include <stdio.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <inference_engine.hpp>

#include "peak.hpp"

#ifndef UNUSED
#ifdef WIN32
#define UNUSED
#else
#define UNUSED  __attribute__((unused))
#endif
#endif

namespace human_pose_estimation {
	class pose_detector {

	public:

		static const size_t keypointsNumber;

		std::vector<HumanPose> estimate(const cv::Mat& image);

		// Constructor
		pose_detector(const std::string& modelPath,
			const std::string& targetDeviceName_,
			bool enablePerformanceReport);

		// Destructor
		~pose_detector();

	private:
		void initialize();
		void finalize();

		void preprocess(const cv::Mat& image, uint8_t* buffer) const;
		std::vector<HumanPose> postprocess(
			const float* heatMapsData, const int heatMapOffset, const int nHeatMaps,
			const float* pafsData, const int pafOffset, const int nPafs,
			const int featureMapWidth, const int featureMapHeight,
			const cv::Size& imageSize) const;
		std::vector<HumanPose> extractPoses(const std::vector<cv::Mat>& heatMaps,
			const std::vector<cv::Mat>& pafs) const;
		void resizeFeatureMaps(std::vector<cv::Mat>& featureMaps) const;
		void correctCoordinates(std::vector<HumanPose>& poses,
			const cv::Size& featureMapsSize,
			const cv::Size& imageSize) const;
		bool inputWidthIsChanged(const cv::Size& imageSize);

		int minJointsNumber;
		int stride;
		cv::Vec4i pad;
		cv::Vec3f meanPixel;
		float minPeaksDistance;
		float midPointsScoreThreshold;
		float foundMidPointsRatioThreshold;
		float minSubsetScore;
		cv::Size inputLayerSize;
		int upsampleRatio;
		InferenceEngine::Core ie;
		std::string targetDeviceName;
		InferenceEngine::CNNNetwork network;
		InferenceEngine::ExecutableNetwork executableNetwork;
		InferenceEngine::InferRequest request;
		InferenceEngine::CNNNetReader netReader;
		std::string pafsBlobName;
		std::string heatmapsBlobName;
		bool enablePerformanceReport;
		std::string modelPath;
	};
}
#endif 