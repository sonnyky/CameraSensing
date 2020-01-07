#include "plane_detection.h"

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