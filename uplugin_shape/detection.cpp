#include "detection.h"
#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms

#pragma once
int _detect_shape(Mat res, vector<Point> contour) {
	int result = -1;
	vector<Point> approx;

	double contourLength = arcLength(contour, true);
	approxPolyDP(contour, approx, contourLength * 0.04, true);
	_draw_polygon(res, approx);
	Rect bRect = boundingRect(contour);

	if (approx.size() == 3) {
		putText(res, "triangle", (bRect.tl() + bRect.br()) * 0.5, FONT_HERSHEY_PLAIN, 2, Scalar::all(255), 2, 8);
		result = 0;
	}
	else if (approx.size() == 4) {
		putText(res, "Rectangular", (bRect.tl() + bRect.br()) * 0.5, FONT_HERSHEY_PLAIN, 2, Scalar::all(255), 2, 8);
		result = 1;
	}
	else if (approx.size() == 5) {
		putText(res, "Pentagon", (bRect.tl() + bRect.br()) * 0.5, FONT_HERSHEY_PLAIN, 2, Scalar::all(255), 2, 8);
		result = 2;
	}
	else {
		// Do nothing, result is still -1
	}
	return result;
}

void _draw_polygon(Mat res, vector<Point> approx) {
	int num_of_sides = approx.size();
	for (int i = 0; i < num_of_sides; i++) {
		if (i < num_of_sides - 1) {
			line(res, approx.at(i), approx.at(i + 1), cvScalar(0, 0, 255), 4);
		}
		else if (i == num_of_sides - 1) {
			line(res, approx.at(i), approx.at(0), cvScalar(0, 0, 255), 4);
		}
	}
}