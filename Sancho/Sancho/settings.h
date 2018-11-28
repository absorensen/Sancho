#pragma once

#define NONZERO 0.00001
#define EPS 1.E-3

#include <string>
#include "camera.h"
#include "shader.h"

enum STATE {COMPRESS, DECOMPRESS, TEST};
enum COMP_MODE { A, B, C, D };

class Settings {
public:

	Settings()
	{
		// Percentage of the number of points from the point cloud to stop subdividing
		s_ps = 0.002f;

	
	}

	int s_ms;
	int max_points_leaf;
	float s_ps;

	double max_point_distance;

	float Z_NEAR;
	float Z_FAR;
	float SCR_WIDTH;
	float SCR_HEIGHT;
	float ASPECT_RATIO;
	float point_size;
	float height_of_near_plane;

	bool reorient_patches;
	COMP_MODE comp_mode;
	STATE state;

	int bits_reserved_axes;
	int min_points;

	bool* draw_patch_normals;
	bool* draw_patch_planes;

	Shader* cube_shader;
	Shader* point_shader;
	Shader* normals_shader;
	Shader* patch_planes_shader;
	
	Camera* camera;

	std::string file;
	std::string extension;
};
