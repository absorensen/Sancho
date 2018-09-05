#pragma once
#ifndef POINT_CLOUD_LOADER_H
#include <fstream>
#include <string>
#include "point_cloud.h"

PointCloud* load_point_cloud(const std::string path, const unsigned int no_of_points, const unsigned int no_of_coords) 
{
	std::ifstream infile(path.c_str());
	PointCloud point_cloud;
	point_cloud.points = new float*[no_of_points];
	point_cloud.no_of_coords = no_of_coords;
	point_cloud.no_of_points = no_of_points;
	for (int i = 0; i < no_of_points; ++i) {
		point_cloud.points[i] = new float[no_of_coords];
		for (int j = 0; j < no_of_coords; ++j) {
			infile >> point_cloud.points[i][j];
			std::cout << point_cloud.points[i][j] << ' ';
		}
		std::cout << '\n';

	}
	infile.close();
	return &point_cloud;
}
#endif