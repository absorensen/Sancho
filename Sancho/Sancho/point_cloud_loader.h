#pragma once

#include <fstream>
#include <string>
#include <iostream>

#include "PointCloud.h"

void load_point_cloud(const std::string path, const unsigned int no_of_points, const unsigned int no_of_coords, PointCloud &point_cloud) 
{
	std::cout << "Loading Point Cloud....              " << std::endl;
	std::cout << "File: " << path.c_str() << std::endl;

	std::ifstream infile(path.c_str());
	float min_x, min_y, min_z, max_x, max_y, max_z, sum_x, sum_y, sum_z;
	min_x = min_y = min_z = sum_x = sum_y = sum_z = 0.0f;
	//PointCloud point_cloud;
	point_cloud.points = new float[no_of_points*no_of_coords];
	point_cloud.no_of_coords = no_of_coords;
	point_cloud.no_of_points = no_of_points;
	point_cloud.size = no_of_coords * no_of_points;
	const int size = point_cloud.size;
	//int coord = 0;
	//bool max_x_init, min_x_init, max_y_init, min_y_init, max_z_init, min_z_init;
	//max_x_init = min_x_init = max_y_init = min_y_init = max_z_init = min_z_init = false;

	for (int i = 0; i < size; ++i) {
		infile >> point_cloud.points[i];

		//// min
		//if (coord == 0) {
		//	if(min_x_init) min_x = std::min(min_x, point_cloud.points[i]);
		//	else min_x = point_cloud.points[i];
		//}
		//if (coord == 1) {
		//	if(min_y_init) min_y = std::min(min_y, point_cloud.points[i]);
		//	else min_y = point_cloud.points[i];
		//}
		//if (coord == 2) {
		//	if(min_z_init) min_z = std::min(min_z, point_cloud.points[i]);
		//	else min_z = point_cloud.points[i];
		//}
		//
		//// max
		//if (coord == 0) {
		//	if(max_x_init) max_x = std::max(max_x, point_cloud.points[i]);
		//	else max_x = point_cloud.points[i];
		//}
		//if (coord == 1) {
		//	if(max_y_init) max_y = std::max(max_y, point_cloud.points[i]);
		//	else max_y = point_cloud.points[i];
		//}
		//if (coord == 2) {
		//	if(max_z_init) max_z = std::max(max_z, point_cloud.points[i]);
		//	else max_z = point_cloud.points[i];
		//}

		//// sum
		//if (coord == 0) sum_x += point_cloud.points[i];
		//if (coord == 1) sum_y += point_cloud.points[i];
		//if (coord == 2) sum_z += point_cloud.points[i];
		//++coord;
		//if (coord == 3) coord = 0;
		//std::cout << point_cloud.points[i] << ' ';
	}
	infile.close();

	//sum_x = sum_x / size;
	//sum_y = sum_y / size;
	//sum_z = sum_z / size;

	//float center_x, center_y, center_z;
	//center_x = center_y = center_z = 0.0f;
	//
	//center_x = (max_x + min_x) * 0.5f;
	//center_y = (max_y + min_y) * 0.5f;
	//center_z = (max_z + min_z) * 0.5f;

	//for (int i = 0; i < size; ++i) {
	//	// put origo into the corner - nobody puts origo in a corner!
	//	/*if (coord == 0) point_cloud.points[i] -= min_x;
	//	if (coord == 1) point_cloud.points[i] -= min_y;
	//	if (coord == 2) point_cloud.points[i] -= min_z;*/
	//	
	//	// put origo in the centroid
	//	//if (coord == 0) point_cloud.points[i] -= sum_x;
	//	//if (coord == 1) point_cloud.points[i] -= sum_y;
	//	//if (coord == 2) point_cloud.points[i] -= sum_z;

	//	// put origo into the middle
	//	if (coord == 0) point_cloud.points[i] -= center_x;
	//	if (coord == 1) point_cloud.points[i] -= center_y;
	//	if (coord == 2) point_cloud.points[i] -= center_z;


	//	++coord;
	//	if (coord == 3) coord = 0;
	//	//std::cout << point_cloud.points[i] << ' ';
	//}

	return;
}
