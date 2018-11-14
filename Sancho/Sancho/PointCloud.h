#pragma once

#include "common_includes.h"

struct PointCloud {
	GLfloat* points = 0;
	unsigned int no_of_points;
	unsigned int no_of_coords;
	unsigned int size;
	float max[3];
	float min[3];
};
