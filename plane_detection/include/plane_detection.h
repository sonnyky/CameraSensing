#ifndef __PLANE_DETECTION__
#define __PLANE_DETECTION__

#define BOOST_CONFIG_SUPPRESS_OUTDATED_MESSAGE
#define _HAS_AUTO_PTR_ETC 1

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
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;


class PlaneSolver {

public:
	// Constructor
	PlaneSolver();

	// Destructor
	~PlaneSolver();
	void solve(pcl_ptr points);
	pcl_ptr points_to_pcl(const rs2::points& points);

private :
	void initialize();
	void finalize();
};

#endif // __APP__