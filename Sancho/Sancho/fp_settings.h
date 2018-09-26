#pragma once

#ifndef FP_SETTINGS_H
#define FP_SETTINGS_H

#define debugplanes
//#define realtime

#define ACCUM_BIN_TYPE float
#define NONZERO 0.00001
#define EPS 1.E-3

#include <string>

class fp_settings {
public:

	fp_settings()
	{
		// Accumulator discretization
		phi_num = 30;
		rho_num = 300;

		// Percentage of the number of points from the point cloud to stop subdividing
		s_ps = 0.002;

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
		max_distance2plane = 0.3;
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
	float s_ps;

	float max_point_distance;
	float max_distance2plane;

	float max_thickness;
	float min_isotropy;

	std::string file;
	std::string extension;
};

#endif