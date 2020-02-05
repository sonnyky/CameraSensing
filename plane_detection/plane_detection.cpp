#include "plane_detection.h"
#include "util.h"
#include <thread>
#include <chrono>

#include <numeric>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

using namespace std;


// Constructor
PlaneSolver::PlaneSolver()
{
	// Initialize
	initialize();
}

// Destructor
PlaneSolver::~PlaneSolver()
{
	// Finalize
	finalize();
}

void PlaneSolver::initialize() {
	last_data_time_point = chrono::steady_clock::now();
	gathering_data = false;
}

void PlaneSolver::finalize() {

}

pcl_ptr PlaneSolver::points_to_pcl(const rs2::points& points)
{
	pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;
	cloud->points.resize(points.size());
	auto ptr = points.get_vertices();
	for (auto& p : cloud->points)
	{
		p.x = ptr->x;
		p.y = ptr->y;
		p.z = ptr->z;
		ptr++;
	}

	return cloud;
}

pcl_ptr PlaneSolver::points_to_pcl(vector<position> positions)
{
	pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	cloud->width = max_number_of_points;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	for (std::size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = positions[i].x;
		cloud->points[i].y = positions[i].y;
		cloud->points[i].z = positions[i].y;
	}
	return cloud;
}

vector<PlaneSolver::position> PlaneSolver::project_positions_on_plane(plane_coefficient plane, vector<position> before_projection)
{
	vector<position> projected_positions;

	pcl_ptr input_points = points_to_pcl(before_projection);
	pcl_ptr projected(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[0] = plane.a;
	coefficients->values[1] = plane.b;
	coefficients->values[2] = plane.c;
	coefficients->values[3] = plane.d;

	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(input_points);
	proj.setModelCoefficients(coefficients);
	proj.filter(*projected);

	for (int i = 0; i < projected->points.size(); i++) {
		position proj = {projected->points[i].x, projected->points[i].y, projected->points[i].z};
		projected_positions.push_back(proj);
	}

	return projected_positions;
}

void PlaneSolver::add_to_positions_list(float x, float y, float z)
{
	position new_position = { x, y, z };
	positions.push_back(new_position);
}

void PlaneSolver::obtain_positions_data(float x, float y, float z)
{
	auto current = chrono::steady_clock::now();
	if (chrono::duration_cast<chrono::seconds> (current - last_data_time_point).count() > 5.0) {
		last_data_time_point = current;
		add_to_positions_list(x, y, z);
		if (positions.size() == max_number_of_points) {
			gathering_data = false;
		}
	}
}

void PlaneSolver::clear_positions_list()
{
	positions.clear();
}

void PlaneSolver::allow_data_gathering() {
	clear_positions_list();
	gathering_data = true;
}

bool PlaneSolver::is_data_gathering_allowed()
{
	return gathering_data;
}

PlaneSolver::plane_coefficient PlaneSolver::solve()
{
	plane_coefficient result = { 0.0, 0.0, 0.0, 0.0 };
	if (gathering_data || positions.size() == 0) {
		return result;
	}

	pcl_ptr points_to_solve = points_to_pcl(positions);

	result = solve(points_to_solve);
	return result;
}

PlaneSolver::plane_coefficient PlaneSolver::solve(pcl_ptr points) {
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	plane_coefficient result_plane = { 0.0, 0.0, 0.0, 0.0 };

	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);

	seg.setInputCloud(points);
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0)
	{
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
		return result_plane;
	}

	std::cout << "Model coefficients: " << coefficients->values[0] << " "
		<< coefficients->values[1] << " "
		<< coefficients->values[2] << " "
		<< coefficients->values[3] << std::endl;

		result_plane.a = coefficients->values[0];
		result_plane.b = coefficients->values[1];
		result_plane.c = coefficients->values[2];
		result_plane.d = coefficients->values[3];


		return result_plane;
	/*for (std::size_t i = 0; i < points->points.size(); ++i)
	{
		float result = (coefficients->values[0] * points->points[i].x) + (coefficients->values[1] * points->points[i].y) + (coefficients->values[2] * points->points[i].z) + coefficients->values[3];
	}*/
}