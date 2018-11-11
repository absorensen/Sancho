#pragma once
#define debugplanes
//#define realtime

#define ACCUM_BIN_TYPE float
#define NONZERO 0.00001
#define EPS 1.E-3

#include <string>
#include "camera.h"
#include "shader.h"

enum MODE {COMPRESS, DECOMPRESS, TEST};

class Settings {
public:

	Settings()
	{
		max_points_leaf = 32;

		// Accumulator discretization
		phi_num = 30;
		rho_num = 300;

		// Percentage of the number of points from the point cloud to stop subdividing
		s_ps = 0.002f;

		// relative tolerances associated with plane thickness (s_alpha) and plane isotropy (s_beta)
		max_thickness = 1.0f / 25.0f;
		min_isotropy = 1.0f / 6.0f;

		// Point cloud examples (let one block uncommented) =======================================
		// Max distance is only used for coloring the point cloud after the plane detection

		// 9 planes
		/**
		s_level = 1;
		file = "../pointclouds/Computer.txt";
		max_distance2plane = 0.025;
		/**/

		// 9 planes
		/**
		s_level = 4;
		file = "../pointclouds/Room.txt";
		max_distance2plane = 0.2;
		/**/

		// 11 planes
		/**
		s_level = 5;
		file = "../pointclouds/Utrecht.txt";
		max_distance2plane = 0.2;
		/**/

		// 14 planes
		/**/
		s_level = 6;
		file = "../pointclouds/Museum.txt";
		max_distance2plane = 0.3f;
		/**/

		// 6 planes
		/**
		s_level = 2;
		file = "../pointclouds/Box.txt";
		max_distance2plane = 10.0;
		/**/

		// 13 planes
		/**
		s_level = 7;
		file = "../pointclouds/Bremen.txt";
		max_distance2plane = 0.5;
		/**/

		// ========================================================================================

	}

	int phi_num;
	int rho_num;
	int s_level;
	int s_ms;
	int max_points_leaf;
	float s_ps;

	double max_point_distance;
	float max_distance2plane;

	float max_thickness;
	float min_isotropy;

	float Z_NEAR;
	float Z_FAR;
	float SCR_WIDTH;
	float SCR_HEIGHT;
	float ASPECT_RATIO;
	float point_size;
	float height_of_near_plane;

	bool reorient_patches;
	bool easily_decodeable;
	MODE state;

	int bits_reserved_axes;

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
