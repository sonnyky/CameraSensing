#ifndef __PLANE_DETECTION__
#define __PLANE_DETECTION__

#include <Windows.h>
#include <comutil.h>
#include <iostream>
#include <cstdio>
#include <ctime>

#include <wtypes.h>
#include <comdef.h> 
#include <string>
#include <string.h>
#include <tchar.h>
#include <stdio.h>
#include "atlbase.h"
#include "atlwin.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <librealsense2/rs.hpp> 

#include <wrl/client.h>
using namespace Microsoft::WRL;
using namespace cv;
using namespace std;
using namespace rs2;

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;


class PlaneSolver {

public:

	// The plane is assumed to take the format ax + by + cz + d = 0
	struct plane_coefficient {
		float a;
		float b;
		float c;
		float d;
	};

	struct position {
		float x;
		float y;
		float z;
	};

	// Constructor
	PlaneSolver();

	// Destructor
	~PlaneSolver();

	plane_coefficient solve(pcl_ptr points);
	plane_coefficient solve();
	pcl_ptr points_to_pcl(const rs2::points& points);
	pcl_ptr points_to_pcl(vector<position> positions);

	vector<position> project_positions_on_plane(plane_coefficient plane, vector<position> before_projection);

	// The parameters are adjusted to data obtained from realsense deprojection
	void add_to_positions_list(float x, float y, float z);

	// Obtain data from devices such as depth cameras
	void obtain_positions_data(float x, float y, float z);

	void clear_positions_list();

	void allow_data_gathering();

	bool is_data_gathering_allowed();

private :
	void initialize();
	void finalize();

	vector<position> positions;

	std::chrono::time_point<std::chrono::steady_clock> last_data_time_point;

	int max_number_of_points;

	bool gathering_data;
};

#endif // __APP__