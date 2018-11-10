#pragma once

#include <fstream>
#include <string>
#include <iostream>

#include "PointCloud.h"
#include "Patch.h"

void load_point_cloud(const std::string path, const unsigned int no_of_points, const unsigned int no_of_coords, PointCloud &point_cloud) 
{
	std::cout << "Loading Point Cloud....              " << std::endl;
	std::cout << "File: " << path.c_str() << std::endl;

	std::ifstream infile(path.c_str());
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
	std::cout << "Loading Compressed Point Cloud...." << std::endl;
	std::cout << "File: " << path.c_str() << std::endl;

	if (easily_decodeable) load_easily_decodeable(path, point_cloud); else load_compressed(path, point_cloud);
}

void load_patch(std::ifstream &file, std::vector<float> &points) {
	Patch patch;
	read_patch(file, points, patch);
	decode_patch(points, patch);
}

void read_patch(std::ifstream &file, std::vector<float> &points, Patch &patch) {
	// origin
	file >> patch.origin[0];
	file >> patch.origin[1];
	file >> patch.origin[2];

	// plane dir1
	file >> patch.plane_dir1[0];
	file >> patch.plane_dir1[1];
	file >> patch.plane_dir1[2];

	// plane dir2
	file >> patch.plane_dir2[0];
	file >> patch.plane_dir2[1];
	file >> patch.plane_dir2[2];

	// plane norm
	file >> patch.plane_norm[0];
	file >> patch.plane_norm[1];
	file >> patch.plane_norm[2];

	// quants
	file >> patch.quant_x;
	file >> patch.quant_y;
	file >> patch.quant_z;

	// number of points
	file >> patch.num_points;

	patch.points = new int8_t[patch.num_points * 3];
	const int num_coords = 3 * patch.num_points;
	for (int i = 0; i < num_coords; ++i) {
		file >> patch.points[i];
	}
}

void decode_patch(std::vector<float> &points, Patch &patch) {
	const unsigned int no_of_coords = patch.num_points * 3;
	Eigen::Vector4d decoded_point;
	Eigen::Vector4d origin;
	origin(0) = patch.origin[0]; origin(1) = patch.origin[1]; 
	origin(2) = patch.origin[2]; origin(3) = 1.0;
	
	Eigen::Matrix4d patch_space_to_world_space;
	patch_space_to_world_space = Eigen::Matrix4d::Identity();

	patch_space_to_world_space(0, 0) = patch.plane_dir1[0];
	patch_space_to_world_space(0, 1) = patch.plane_dir1[1];
	patch_space_to_world_space(0, 2) = patch.plane_dir1[2];
	patch_space_to_world_space(0, 3) = 0.0;
	
	patch_space_to_world_space(1, 0) = patch.plane_dir2[0];
	patch_space_to_world_space(1, 1) = patch.plane_dir2[1];
	patch_space_to_world_space(1, 2) = patch.plane_dir2[2];
	patch_space_to_world_space(1, 3) = 0.0;
	
	patch_space_to_world_space(2, 0) = patch.plane_norm[0];
	patch_space_to_world_space(2, 1) = patch.plane_norm[1];
	patch_space_to_world_space(2, 2) = patch.plane_norm[2];
	patch_space_to_world_space(2, 3) = 0.0;

	patch_space_to_world_space = patch_space_to_world_space.inverse();
	for (unsigned int i = 0, j = 0; i < no_of_coords; i += 3, ++j) {
		decoded_point.x() = patch.points[i] / patch.quant_x;
		decoded_point.y() = patch.points[i + 1] / patch.quant_y;
		decoded_point.z() = patch.points[i + 2] / patch.quant_z;
		decoded_point.w() = 1.0;

		decoded_point = origin + patch_space_to_world_space * decoded_point;
		decoded_point.w() = 1.0;
		points.push_back(decoded_point(0));
		points.push_back(decoded_point(1));
		points.push_back(decoded_point(2));
	}
}

void load_easily_decodeable(const std::string path, PointCloud &point_cloud) {
	std::ifstream file(path.c_str());
	std::vector<float> points;
	std::string token;
	do {
		load_patch(file, points);
		file >> token;
	} while (token.c_str == "E");
}

void load_compressed(const std::string path, PointCloud &point_cloud){

}