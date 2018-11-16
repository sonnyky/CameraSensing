#include "detection.h"
#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms

#pragma once

/*
Calibration methods
*/
const float * _calc_homography(UPoint * src, UPoint * dst, int length) {

	vector<Point2f> proj_points;
	vector<Point2f> img_points;
	vector<float> homographyValues;
	Point2f point;

	for (int i = 0; i < length; i++) {
		Point2f tempSrc, tempDst;
		tempSrc.x = src->x;
		tempSrc.y = src->y;
		proj_points.push_back(tempSrc);

		tempDst.x = dst->x;
		tempDst.y = dst->y;
		img_points.push_back(tempDst);

		src++;
		dst++;
	}
	
	Mat homography = findHomography(proj_points, img_points);
	for (int i = 0; i < homography.rows; i++) {
		for (int j = 0; j < homography.cols; j++) {
			homographyValues.push_back((float) homography.at<double>(i, j));
		}
	}
	return &homographyValues[0];
}