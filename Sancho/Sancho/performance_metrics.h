#pragma once

// implementation based on description found in:
// Real-time Point Cloud Compression by Golla and Klein
// http://cg.cs.uni-bonn.de/aigaion2root/attachments/GollaIROS2015_authorsversion.pdf

#include "PointCloud.h"
#include "common_includes.h"

double rmse(PointCloud &a, PointCloud &b) {
	const int size = a.size;
	double sum = 0.0;
	float diff;
	float nn_dist = 0.0f;
	float p[3];
	float q[3];
	unsigned nn_index = 0;
	for (unsigned i = 0; i < size; i += 3) {
		p[0] = a.points[i];
		p[1] = a.points[i + 1];
		p[2] = a.points[i + 2];
		nn_dist = 9999999.0f;
		for (unsigned j = 0; j < size; j += 3) {
			if (i == j) continue;
			q[0] = b.points[j];
			q[1] = b.points[j + 1];
			q[2] = b.points[j + 2];

			diff = std::sqrtf(
				(p[0] - q[0]) * (p[0] - q[0]) +
				(p[1] - q[1]) * (p[1] - q[1]) +
				(p[2] - q[2]) * (p[2] - q[2])
			);
			if (diff < nn_dist) { 
				nn_dist = diff; 
				nn_index = j;
			}
		}
		diff = a.points[i] - b.points[nn_index];
		diff += a.points[i+1] - b.points[nn_index+1];
		diff += a.points[i+2] - b.points[nn_index+2];
		sum += diff * diff;
	}
	sum /= static_cast<double>(size);
	sum = std::sqrt(sum);
	return sum;
}

void psnr(PointCloud &a, PointCloud &b) {
	const double psnr_factor = 20.0;
	const double rmse1 = rmse(a, b);
	const double rmse2 = rmse(b, a);
	const double max_rmse = std::max(rmse1, rmse2);
	const double peak = 
		std::sqrt(
			std::pow(a.max[0] - a.min[0], 2.0) +
			std::pow(a.max[1] - a.min[1], 2.0) +
			std::pow(a.max[2] - a.min[2], 2.0));
	std::cout << "Peak equals: " << peak << std::endl;
	std::cout << "RMSE1 equals: " << rmse1 << std::endl;
	std::cout << "RMSE2 equals: " << rmse2 << std::endl;
	std::cout << "Max RMSE equals: " << max_rmse << std::endl;
	std::cout << "rmse over peak: " << max_rmse / peak << std::endl;
	const double psnr = psnr_factor * std::log10(max_rmse/peak);
	std::cout << "PSNR equals: " << psnr << std::endl;
}
