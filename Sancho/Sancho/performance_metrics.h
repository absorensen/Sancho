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
	std::cout << "RMSE 1: " << rmse(a, b) << std::endl;
	std::cout << "RMSE 2: " << rmse(b, a) << std::endl;
}
