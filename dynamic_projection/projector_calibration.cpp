#include "include\projector_calibration.hpp"

Tinker::projector_calibration::projector_calibration()
{
}

Tinker::projector_calibration::~projector_calibration()
{
}

void Tinker::projector_calibration::load(string projector_config)
{
}

void Tinker::projector_calibration::set_static_candidate_image_points()
{
	candidateImagePoints.clear();
	Point2f p;
	for (int i = 0; i < boardSize.height; i++) {
		for (int j = 0; j < boardSize.width; j++) {
			p.x = patternPosition.x + float(((2 * j) + (i % 2)) * squareSize);
			p.y = patternPosition.y + float(i * squareSize);
			candidateImagePoints.push_back(p);
		}
	}
}

void Tinker::projector_calibration::setPatternPosition(float px, float py)
{
	patternPosition = Point2f(px, py);
}
