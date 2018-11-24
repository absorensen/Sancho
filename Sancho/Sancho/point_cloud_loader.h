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
	point_cloud.max[0] = point_cloud.max[1] = point_cloud.max[2] = 
	point_cloud.min[0] = point_cloud.min[1] = point_cloud.min[2] = 0.0f;

	for (int i = 0, j = 0; i < size; ++i, ++j) {
		infile >> point_cloud.points[i];

		if (point_cloud.points[i] > point_cloud.max[j] || point_cloud.max[j] == 0.0f) point_cloud.max[j] = point_cloud.points[i];
		if (point_cloud.points[i] < point_cloud.min[j] || point_cloud.min[j] == 0.0f) point_cloud.min[j] = point_cloud.points[i];
		j = j % 3;
	}

	infile.close();

	return;
}


void read_patch_a(std::ifstream &file, std::queue<float> &points, Patch &patch) {
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

void decode_patch_a(std::queue<float> &points, Patch &patch) {
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

void load_patch_a(std::ifstream &file, std::queue<float> &points) {
	Patch patch;
	read_patch_a(file, points, patch);
	decode_patch_a(points, patch);
}


void load_compressed_a(const std::string path, PointCloud &point_cloud) {
	std::ifstream file(path.c_str(), std::ios::in | std::ios::binary);
	std::queue<float> points;
	do {
		load_patch_a(file, points);
	} while (!file.eof());
	file.close();
	point_cloud.no_of_coords = 3;
	point_cloud.size = points.size();
	point_cloud.no_of_points = point_cloud.size / point_cloud.no_of_coords;
	point_cloud.points = new float[point_cloud.size];
	point_cloud.max[0] = point_cloud.max[1] = point_cloud.max[2] =
	point_cloud.min[0] = point_cloud.min[1] = point_cloud.min[2] = 0.0f;
	
	for (int i = 0, j = 0; i < point_cloud.size; ++i, ++j) {
		point_cloud.points[i] = points.front();
		points.pop();

		if (point_cloud.points[i] > point_cloud.max[j] || point_cloud.max[j] == 0.0f) point_cloud.max[j] = point_cloud.points[i];
		if (point_cloud.points[i] < point_cloud.min[j] || point_cloud.min[j] == 0.0f) point_cloud.min[j] = point_cloud.points[i];
		j = j % 3;
	}
}

void read_patch_b(std::ifstream &file, std::queue<float> &points, Patch &patch) {
	// origin
	file.read((char*)&patch.origin[0], sizeof(float));
	file.read((char*)&patch.origin[1], sizeof(float));
	file.read((char*)&patch.origin[2], sizeof(float));

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

void decode_patch_b(std::queue<float> &points, Patch &patch) {
	const unsigned int no_of_coords = patch.num_points * 3;
	Eigen::Vector4d decoded_point;
	Eigen::Vector4d origin;
	origin(0) = patch.origin[0]; origin(1) = patch.origin[1];
	origin(2) = patch.origin[2]; origin(3) = 1.0;

	for (unsigned int i = 0; i < no_of_coords; i += 3) {
		decoded_point.x() = static_cast<double>(patch.points[i]) / patch.quant_x;
		decoded_point.y() = static_cast<double>(patch.points[i + 1]) / patch.quant_y;
		decoded_point.z() = static_cast<double>(patch.points[i + 2]) / patch.quant_z;
		decoded_point.w() = 1.0;

		decoded_point = decoded_point + origin;
		points.push(decoded_point(0));
		points.push(decoded_point(1));
		points.push(decoded_point(2));
	}
}

void load_patch_b(std::ifstream &file, std::queue<float> &points) {
	Patch patch;
	read_patch_b(file, points, patch);
	decode_patch_b(points, patch);
}


void load_compressed_b(const std::string path, PointCloud &point_cloud) {
	std::ifstream file(path.c_str(), std::ios::in | std::ios::binary);
	std::queue<float> points;
	do {
		load_patch_b(file, points);
	} while (!file.eof());
	file.close();
	point_cloud.no_of_coords = 3;
	point_cloud.size = points.size();
	point_cloud.no_of_points = point_cloud.size / point_cloud.no_of_coords;
	point_cloud.points = new float[point_cloud.size];
	point_cloud.max[0] = point_cloud.max[1] = point_cloud.max[2] =
		point_cloud.min[0] = point_cloud.min[1] = point_cloud.min[2] = 0.0f;

	for (int i = 0, j = 0; i < point_cloud.size; ++i, ++j) {
		point_cloud.points[i] = points.front();
		points.pop();

		if (point_cloud.points[i] > point_cloud.max[j] || point_cloud.max[j] == 0.0f) point_cloud.max[j] = point_cloud.points[i];
		if (point_cloud.points[i] < point_cloud.min[j] || point_cloud.min[j] == 0.0f) point_cloud.min[j] = point_cloud.points[i];
		j = j % 3;
	}
}

void read_patch_c(std::ifstream &file, std::queue<float> &points, Patch &patch) {
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

void decode_patch_c(std::queue<float> &points, Patch &patch) {
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

void load_patch_c(std::ifstream &file, std::queue<float> &points) {
	Patch patch;
	read_patch_c(file, points, patch);
	decode_patch_c(points, patch);
}

void load_uncompressed_c(std::ifstream &file, std::queue<float> &points, const unsigned int coords) {
	float value = 0.0f;
	std::streamsize entry_size = sizeof(float);
	for (unsigned int i = 0; i < coords; ++i) {
		file.read((char*)&value, entry_size);
		points.push(value);
	}
}

void load_compressed_c(const std::string path, const std::string path_b, PointCloud &point_cloud, const unsigned int coords) {
	std::ifstream file(path.c_str(), std::ios::in | std::ios::binary);
	std::queue<float> points;
	do {
		load_patch_c(file, points);
	} while (!file.eof());
	file.close();

	std::ifstream file_b(path_b.c_str(), std::ios::in | std::ios::binary);
	load_uncompressed_c(file_b, points, coords-points.size());
	file_b.close();

	point_cloud.no_of_coords = 3;
	point_cloud.size = points.size();
	point_cloud.no_of_points = point_cloud.size / point_cloud.no_of_coords;
	point_cloud.points = new float[point_cloud.size];
	point_cloud.max[0] = point_cloud.max[1] = point_cloud.max[2] =
		point_cloud.min[0] = point_cloud.min[1] = point_cloud.min[2] = 0.0f;

	for (int i = 0, j = 0; i < point_cloud.size; ++i, ++j) {
		point_cloud.points[i] = points.front();
		points.pop();

		if (point_cloud.points[i] > point_cloud.max[j] || point_cloud.max[j] == 0.0f) point_cloud.max[j] = point_cloud.points[i];
		if (point_cloud.points[i] < point_cloud.min[j] || point_cloud.min[j] == 0.0f) point_cloud.min[j] = point_cloud.points[i];
		j = j % 3;
	}
}

void read_patch_d(std::ifstream &file, std::queue<float> &points, Patch &patch) {
	// origin
	file.read((char*)&patch.origin[0], sizeof(float));
	file.read((char*)&patch.origin[1], sizeof(float));
	file.read((char*)&patch.origin[2], sizeof(float));

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

void decode_patch_d(std::queue<float> &points, Patch &patch) {
	const unsigned int no_of_coords = patch.num_points * 3;
	Eigen::Vector4d decoded_point;
	Eigen::Vector4d origin;
	origin(0) = patch.origin[0]; origin(1) = patch.origin[1];
	origin(2) = patch.origin[2]; origin(3) = 1.0;

	for (unsigned int i = 0; i < no_of_coords; i += 3) {
		decoded_point.x() = patch.points[i] / patch.quant_x;
		decoded_point.y() = patch.points[i + 1] / patch.quant_y;
		decoded_point.z() = patch.points[i + 2] / patch.quant_z;
		decoded_point.w() = 1.0;

		decoded_point = origin + decoded_point;
		points.push(decoded_point(0));
		points.push(decoded_point(1));
		points.push(decoded_point(2));
	}
	delete patch.points;
}

void load_patch_d(std::ifstream &file, std::queue<float> &points) {
	Patch patch;
	read_patch_d(file, points, patch);
	decode_patch_d(points, patch);
}

void load_uncompressed_d(std::ifstream &file, std::queue<float> &points, const unsigned int coords) {
	std::streamsize entry_size = sizeof(float);
	float value = 0.0f;

	for (unsigned int i = 0; i < coords; ++i) {
		file.read((char*)&value, entry_size);
		points.push(value);
	}
}

void load_compressed_d(const std::string path, const std::string path_b, PointCloud &point_cloud, const unsigned int coords) {
	std::ifstream file(path.c_str(), std::ios::in | std::ios::binary);
	std::queue<float> points;
	do {
		load_patch_d(file, points);
	} while (!file.eof());
	file.close();

	std::ifstream file_b(path_b.c_str(), std::ios::in | std::ios::binary);
	load_uncompressed_d(file_b, points, coords-points.size());
	file_b.close();

	point_cloud.no_of_coords = 3;
	point_cloud.size = points.size();
	point_cloud.no_of_points = point_cloud.size / point_cloud.no_of_coords;
	point_cloud.points = new float[point_cloud.size];
	point_cloud.max[0] = point_cloud.max[1] = point_cloud.max[2] =
		point_cloud.min[0] = point_cloud.min[1] = point_cloud.min[2] = 0.0f;

	for (int i = 0, j = 0; i < point_cloud.size; ++i, ++j) {
		point_cloud.points[i] = points.front();
		points.pop();

		if (point_cloud.points[i] > point_cloud.max[j] || point_cloud.max[j] == 0.0f) point_cloud.max[j] = point_cloud.points[i];
		if (point_cloud.points[i] < point_cloud.min[j] || point_cloud.min[j] == 0.0f) point_cloud.min[j] = point_cloud.points[i];
		j = j % 3;
	}
}

void load_compressed_point_cloud(const std::string path, const std::string path_b, PointCloud &point_cloud, const COMP_MODE comp_mode, const unsigned int coords = 0)
{
	std::cout << "Loading Compressed Point Cloud...." << std::endl;
	std::cout << "File: " << path.c_str() << std::endl;

	if (comp_mode == A) { load_compressed_a(path, point_cloud); }
	else if (comp_mode == B) { load_compressed_b(path, point_cloud); }
	else if (comp_mode == C) { load_compressed_c(path, path_b, point_cloud, coords); }
	else if(comp_mode == D) { load_compressed_d(path, path_b, point_cloud, coords); }
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
	point_cloud.points = new float[point_cloud.size];
	auto float_size = sizeof(float);
	for (int i = 0; i < point_cloud.size; ++i) {
		file.read((char*)&point_cloud.points[i], float_size);
	}
	file.close();
}