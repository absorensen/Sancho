#pragma once
#ifndef POINT_CLOUD_LOADER_H
#include <fstream>
#include <string>
#include "point_cloud.h"

PointCloud* load_point_cloud(const std::string path, const unsigned int no_of_points, const unsigned int no_of_coords) 
{
	std::ifstream infile(path.c_str());
	PointCloud point_cloud;
	point_cloud.points = new float[no_of_points*no_of_coords];
	point_cloud.no_of_coords = no_of_coords;
	point_cloud.no_of_points = no_of_points;
	point_cloud.size = no_of_coords * no_of_points;
	for (int i = 0; i < point_cloud.size; ++i) {
		infile >> point_cloud.points[i];
		std::cout << point_cloud.points[i] << ' ';
	}
	infile.close();
	return &point_cloud;
}
#endif