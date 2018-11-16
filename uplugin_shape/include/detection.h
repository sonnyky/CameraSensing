#include "recognition.h"

#pragma once

using namespace std;
using namespace cv;

int _detect_shape(Mat res, vector<Point> contour);
void _draw_polygon(Mat res, vector<Point> approx);