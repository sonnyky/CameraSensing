#include <iostream>
#include <sstream>
#include <librealsense2/rs.hpp> 
#include "app.h"

bool fexists(const std::string& filename) {
	std::ifstream ifile(filename.c_str());
	return (bool)ifile;
}

int main(int argc, char* argv[])
{
	
	try {
		PlaneSolver plane_solver;

		// Create test data
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

		// Fill in the cloud data
		cloud->width = 15;
		cloud->height = 1;
		cloud->points.resize(cloud->width * cloud->height);

		if (!fexists("test_points.txt")) {
			ofstream myfile;
			myfile.open("test_points.txt");

			// Generate the data, no outliers
			// formula is 2x + 3y -5z + 7 = 0
			for (std::size_t i = 0; i < cloud->points.size(); ++i)
			{
				// Have to have a randomness to make sure the points are spread out and not in a line
				cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
				cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
				cloud->points[i].z = ((2.0 * cloud->points[i].x) + (3.0 * cloud->points[i].y) + 7.0) / 5.0;
				myfile << cloud->points[i].x << "," << cloud->points[i].y << "," << cloud->points[i].z << "\n";

			}
			
			myfile.close();
		}
		else {
			ifstream myfile("test_points.txt");
			string line;
			std::string delimiter = ",";
			if (myfile.is_open())
			{
				int counter = 0;
				while (std::getline(myfile, line) && counter < 15)
				{
					size_t pos = 0;
					std::string token;
					int axes_id = 0;
					while ((pos = line.find(delimiter)) != std::string::npos) {
						token = line.substr(0, pos);
						std::cout << token << std::endl;
						if (axes_id == 0) {
							cloud->points[counter].x = stof(token);
						}
						else if(axes_id == 1){
							cloud->points[counter].y = stof(token);
						}
						axes_id++;
						line.erase(0, pos + delimiter.length());
					}
					std::cout << line << std::endl;
					cloud->points[counter].z = stof(line);
					counter++;
				}
				myfile.close();
			}

		}
		plane_solver.solve(cloud);
	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}

	return 0;
}