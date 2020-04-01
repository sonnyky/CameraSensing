#pragma once

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>


using namespace std;

namespace Tinker {

	class pcl_to_mesh {
	public:
		pcl_to_mesh();
		~pcl_to_mesh();

		void estimate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	private:
	};

}