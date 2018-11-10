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

	for (int i = 0; i < size; ++i) {
		infile >> point_cloud.points[i];
	}
	infile.close();

	return;
}

void load_compressed_point_cloud(const std::string path, PointCloud &point_cloud, const bool easily_decodeable)
{
	std::cout << "Loading Compressed Point Cloud....              " << std::endl;
	std::cout << "File: " << path.c_str() << std::endl;

	if (easily_decodeable) load_easily_decodeable(); else load_compressed();
}

void load_easily_decodeable() {

}

void load_compressed(){

}