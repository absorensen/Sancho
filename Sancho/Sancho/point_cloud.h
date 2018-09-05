#pragma once
#ifndef POINT_CLOUD_H
struct PointCloud {
	float** points = 0;
	unsigned int no_of_points;
	unsigned int no_of_coords;
};
#endif