#include "plane_detection.h"
#include "pose_detector.hpp"

class PositionDetector {

public:
	// Constructor
	PositionDetector();

	// Destructor
	~PositionDetector();
	
private :
	void initialize();
	void finalize();
	pcl_ptr points;
};