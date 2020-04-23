#pragma once

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/filters/passthrough.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include "pugixml.hpp";

using namespace std;
using namespace pcl;

namespace Tinker {

	//convenient typedefs
	typedef pcl::PointXYZ PointT;
	typedef pcl::PointCloud<PointT> PointCloud;
	typedef pcl::PointNormal PointNormalT;
	typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

	class pcl_to_mesh {
	public:
		pcl_to_mesh();
		~pcl_to_mesh();

		void estimate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, string fileName, bool useFile = false);
		void pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false);

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2;
		pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud;

		void add_to_cloud1(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
		void add_to_cloud2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
		void align_and_save_clouds();
		void align_clouds();
		void save_and_generate_mesh();
		void generate_mesh_from_file();

		void continuous_scan_store_aligned_as_cloud1();

		Eigen::Matrix4f GlobalTransform;

		void setup_reconstruction_parameters();
	private:
		int k_search_param = 20;
		double search_radius = 0.025;
		double mu = 2.5;
		int max_nearest_neighbour = 100;
		int cloud_size_limit = 60000;
		double max_correspondence_distance = 0.1;
	};


	// Define a new point representation for < x, y, z, curvature >
	class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
	{
		using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
	public:
		MyPointRepresentation()
		{
			// Define the number of dimensions
			nr_dimensions_ = 4;
		}

		// Override the copyToFloatArray method to define our feature vector
		virtual void copyToFloatArray(const PointNormalT &p, float * out) const
		{
			// < x, y, z, curvature >
			out[0] = p.x;
			out[1] = p.y;
			out[2] = p.z;
			out[3] = p.curvature;
		}
	};


}