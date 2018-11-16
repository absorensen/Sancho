#pragma once

#include <fstream>
#include <string>
#include <iostream>
#include <queue>

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
	point_cloud.max[0] = 0.0f;
	point_cloud.max[1] = 0.0f;
	point_cloud.max[2] = 0.0f;
	point_cloud.min[0] = 0.0f;
	point_cloud.min[1] = 0.0f;
	point_cloud.min[2] = 0.0f;

	for (int i = 0, j = 0; i < size; ++i, ++j) {
		infile >> point_cloud.points[i];

		// ISSUES HERE
		if (point_cloud.points[i] > point_cloud.max[j]) point_cloud.max[j] = point_cloud.points[i];
		if (point_cloud.points[i] < point_cloud.min[j]) point_cloud.min[j] = point_cloud.points[i];
		j = j % 3;
	}

	infile.close();

	return;
}


void read_patch(std::ifstream &file, std::queue<float> &points, Patch &patch) {
	//// origin
	//file.read((char*)&patch.origin[0], sizeof(patch.origin[0]));
	//file.read((char*)&patch.origin[1], sizeof(patch.origin[1]));
	//file.read((char*)&patch.origin[2], sizeof(patch.origin[2]));

	//// plane dir1
	//file.read((char*)&patch.plane_dir1[0], sizeof(patch.plane_dir1[0]));
	//file.read((char*)&patch.plane_dir1[1], sizeof(patch.plane_dir1[1]));
	//file.read((char*)&patch.plane_dir1[2], sizeof(patch.plane_dir1[2]));

	//// plane dir2
	//file.read((char*)&patch.plane_dir2[0], sizeof(patch.plane_dir2[0]));
	//file.read((char*)&patch.plane_dir2[1], sizeof(patch.plane_dir2[1]));
	//file.read((char*)&patch.plane_dir2[2], sizeof(patch.plane_dir2[2]));

	//// plane norm
	//file.read((char*)&patch.plane_norm[0], sizeof(patch.plane_norm[0]));
	//file.read((char*)&patch.plane_norm[1], sizeof(patch.plane_norm[1]));
	//file.read((char*)&patch.plane_norm[2], sizeof(patch.plane_norm[2]));

	//// quants
	//file.read((char*)&patch.quant_x, sizeof(patch.quant_x));
	//file.read((char*)&patch.quant_y, sizeof(patch.quant_y));
	//file.read((char*)&patch.quant_z, sizeof(patch.quant_z));
	//
	//// number of points
	//file.read((char*)&patch.num_points, sizeof(patch.num_points));

	//patch.points = new int8_t[patch.num_points * 3];
	//const int num_coords = 3 * patch.num_points;
	//std::streamsize entry_size = sizeof(patch.num_points);
	//for (int i = 0; i < num_coords; ++i) {
	//	file.read((char*)&patch.points[i], entry_size);
	//}


		// origin
	file.read((char*)&patch.origin[0], sizeof(float));
	file.read((char*)&patch.origin[1], sizeof(float));
	file.read((char*)&patch.origin[2], sizeof(float));

	// plane dir1
	file.read((char*)&patch.plane_dir1[0], sizeof(float));
	file.read((char*)&patch.plane_dir1[1], sizeof(float));
	file.read((char*)&patch.plane_dir1[2], sizeof(float));

	// plane dir2
	file.read((char*)&patch.plane_dir2[0], sizeof(float));
	file.read((char*)&patch.plane_dir2[1], sizeof(float));
	file.read((char*)&patch.plane_dir2[2], sizeof(float));

	// plane norm
	file.read((char*)&patch.plane_norm[0], sizeof(float));
	file.read((char*)&patch.plane_norm[1], sizeof(float));
	file.read((char*)&patch.plane_norm[2], sizeof(float));

	// quants
	file.read((char*)&patch.quant_x, sizeof(float));
	file.read((char*)&patch.quant_y, sizeof(float));
	file.read((char*)&patch.quant_z, sizeof(float));

	// number of points
	file.read((char*)&patch.num_points, sizeof(uint8_t));

	patch.points = new int8_t[patch.num_points * 3];
	const int num_coords = 3 * patch.num_points;
	std::streamsize entry_size = sizeof(int8_t);
	for (int i = 0; i < num_coords; ++i) {
		file.read((char*)&patch.points[i], entry_size);
	}
}

void decode_patch(std::queue<float> &points, Patch &patch) {
	const unsigned int no_of_coords = patch.num_points * 3;
	Eigen::Vector4d decoded_point;
	Eigen::Vector4d origin;
	origin(0) = patch.origin[0]; origin(1) = patch.origin[1];
	origin(2) = patch.origin[2]; origin(3) = 1.0;
	
	Eigen::Matrix4d patch_coord_system;
	patch_coord_system = Eigen::Matrix4d::Identity();

	patch_coord_system(0, 0) = patch.plane_dir1[0];
	patch_coord_system(0, 1) = patch.plane_dir1[1];
	patch_coord_system(0, 2) = patch.plane_dir1[2];
	patch_coord_system(0, 3) = 0.0;

	patch_coord_system(1, 0) = patch.plane_dir2[0];
	patch_coord_system(1, 1) = patch.plane_dir2[1];
	patch_coord_system(1, 2) = patch.plane_dir2[2];
	patch_coord_system(1, 3) = 0.0;

	patch_coord_system(2, 0) = patch.plane_norm[0];
	patch_coord_system(2, 1) = patch.plane_norm[1];
	patch_coord_system(2, 2) = patch.plane_norm[2];
	patch_coord_system(2, 3) = 0.0;

	Eigen::Matrix4d patch_space_to_world_space;
	patch_space_to_world_space = patch_coord_system.inverse();
	for (unsigned int i = 0; i < no_of_coords; i += 3) {
		decoded_point.x() = patch.points[i] / patch.quant_x;
		decoded_point.y() = patch.points[i + 1] / patch.quant_y;
		decoded_point.z() = patch.points[i + 2] / patch.quant_z;
		decoded_point.w() = 1.0;

		decoded_point = origin + patch_space_to_world_space * decoded_point;
		points.push(decoded_point(0));
		points.push(decoded_point(1));
		points.push(decoded_point(2));
	}
}

void load_patch(std::ifstream &file, std::queue<float> &points) {
	Patch patch;
	read_patch(file, points, patch);
	decode_patch(points, patch);
}



void load_easily_decodeable(const std::string path, PointCloud &point_cloud) {
	std::ifstream file(path.c_str());
	std::queue<float> points;
	do {
		load_patch(file, points);
	} while (!file.eof());
	point_cloud.no_of_points = points.size();
	point_cloud.no_of_coords = 3;
	point_cloud.size = point_cloud.no_of_points * point_cloud.no_of_coords;
	point_cloud.points = new float[point_cloud.size];
	for (int i = 0; i < point_cloud.size; ++i) {
		point_cloud.points[i] = points.front();
		points.pop();
	}

}

void load_compressed(const std::string path, PointCloud &point_cloud) {
	std::ifstream file(path.c_str(), std::ios::in | std::ios::binary);
	std::queue<float> points;
	do {
		load_patch(file, points);
	} while (!file.eof());
	file.close();
	point_cloud.no_of_coords = 3;
	point_cloud.size = points.size();
	point_cloud.no_of_points = point_cloud.size / point_cloud.no_of_coords;
	point_cloud.points = new float[point_cloud.size];
	point_cloud.max[0] = 0.0f;
	point_cloud.max[1] = 0.0f;
	point_cloud.max[2] = 0.0f;
	point_cloud.min[0] = 0.0f;
	point_cloud.min[1] = 0.0f;
	point_cloud.min[2] = 0.0f;
	for (int i = 0, j = 0; i < point_cloud.size; ++i, ++j) {
		point_cloud.points[i] = points.front();
		points.pop();

		// ISSUES HERE
		if (point_cloud.points[i] > point_cloud.max[j]) point_cloud.max[j] = point_cloud.points[i];
		if (point_cloud.points[i] < point_cloud.min[j]) point_cloud.min[j] = point_cloud.points[i];
		j = j % 3;
	}
}

void load_compressed_point_cloud(const std::string path, PointCloud &point_cloud, const bool easily_decodeable)
{
	std::cout << "Loading Compressed Point Cloud...." << std::endl;
	std::cout << "File: " << path.c_str() << std::endl;

	if (easily_decodeable) load_easily_decodeable(path, point_cloud); else load_compressed(path, point_cloud);
}

void write_point_cloud_to_binary(const std::string file_name, PointCloud &point_cloud) {
	std::ofstream file(file_name.c_str(), std::ios::out | std::ios::trunc | std::ios::binary);
	const int size = point_cloud.size;
	file.write((char*)&size, sizeof(size));
	auto float_size = sizeof(point_cloud.points[0]);
	for (int i = 0; i < size; ++i) {
		file.write((char*)&point_cloud.points[i], float_size);
	}
}

void load_point_cloud_from_binary(const std::string file_name, PointCloud &point_cloud) {
	std::ifstream file(file_name.c_str(), std::ios::in | std::ios::binary);
	file.read((char*)&point_cloud.size, sizeof(int));
	point_cloud.no_of_coords = 3;
	point_cloud.no_of_points = point_cloud.size / point_cloud.no_of_coords;
	auto float_size = sizeof(float);
	for (int i = 0; i < point_cloud.size; ++i) {
		file.read((char*)&point_cloud.points[i], float_size);
	}
}