#pragma once

#include "settings.h"
#include "common_includes.h"
#include "shader.h"

class Patch {
public:
	Patch() {};
	Patch(Patch* to_copy) {
		for (short i = 0; i < 3; ++i) {
			plane_dir1[i] = to_copy->plane_dir1[i];
			plane_dir2[i] = to_copy->plane_dir2[i];
			plane_norm[i] = to_copy->plane_norm[i];
			origin[i] = to_copy->origin[i];
		}
		quant_x = to_copy->quant_x;
		quant_y = to_copy->quant_y;
		quant_z = to_copy->quant_z;

		num_points = to_copy->num_points;

		points = to_copy->points;
	};
	~Patch() {
		if (points != nullptr) {
			delete points;
		}
	};

	// article specifies position, orientation and size
	float origin[3];
	float plane_dir1[3];
	float plane_dir2[3];
	float plane_norm[3];
	float quant_x, quant_y, quant_z; 
	uint8_t num_points;
	int8_t* points;
	float* non_compressible_points;
};