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
	float min_x, min_y, min_z;
	min_x = min_y = min_z = 0.0f;
	//PointCloud point_cloud;
	point_cloud.points = new float[no_of_points*no_of_coords];
	point_cloud.no_of_coords = no_of_coords;
	point_cloud.no_of_points = no_of_points;
	point_cloud.size = no_of_coords * no_of_points;
	const int size = point_cloud.size;
	int coord = 0;
	for (int i = 0; i < size; ++i) {
		infile >> point_cloud.points[i];
		if (coord == 0) min_x = std::min(min_x, point_cloud.points[i]);
		if (coord == 1) min_y = std::min(min_y, point_cloud.points[i]);
		if (coord == 2) min_z = std::min(min_z, point_cloud.points[i]);
		++coord;
		if (coord == 3) coord = 0;
		//std::cout << point_cloud.points[i] << ' ';
	}
	infile.close();

	for (int i = 0; i < size; ++i) {
		if (coord == 0) point_cloud.points[i] -= min_x;
		if (coord == 1) point_cloud.points[i] -= min_y;
		if (coord == 2) point_cloud.points[i] -= min_z;
		++coord;
		if (coord == 3) coord = 0;
		//std::cout << point_cloud.points[i] << ' ';
	}

	return;
}
