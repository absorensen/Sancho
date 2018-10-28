#pragma once

#include "settings.h"
#include "common_includes.h"
#include "shader.h"

struct Patch {
	// article specifies position, orientation and size
	double plane_dir1[4];
	double plane_dir2[4];
	double plane_norm[4];
	double plane_orientation[4];
	double origin[4];
	int8_t height_map[16][16];
	bool occupancy_map[16][16];

	int8_t size;
	double min_value, max_value;
};

class Octree {
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Octree();

	void subdivide(Settings &settings);
	void show(int height);
	void show_level(int height);
	void show_tree();
	void show_normals(const float length);
	void show_patch_planes(const float size);
	void show();
	void clear(void);
	void get_nodes(std::vector< Octree*> &nodes);
	void least_variance_direction();
	double plane_distance(Eigen::Vector4d &point);
	double signed_plane_distance(Eigen::Vector4d &point);
	void draw_wire_cube(Eigen::Vector4d &middle, float size);
	void draw_points();
	void draw_normal(const float length);
	void draw_patch_plane(const float size);
	Eigen::Matrix3d fast_covariance_matrix();


	std::vector<Eigen::Vector4d> m_points, m_colors;
	std::vector<int> m_indexes;

	Eigen::Matrix3d m_covariance;
	bool m_is_leaf;
	Octree *m_children, *m_root;
	Settings* _settings;
	Eigen::Vector4d normal1, normal2, normal3, m_middle;
	Eigen::Vector4d m_centroid, color;
	double m_size, variance1, variance2, variance3;
	short m_level;

	// wire cube
	unsigned int wireCubeVAO = 0;
	unsigned int wireCubeVBO = 0;

	// points
	unsigned int pointsVAO = 0;
	unsigned int pointsVBO = 0;

	// normals
	unsigned int normalVAO = 0;
	unsigned int normalVBO = 0;

	// planes
	unsigned int planeVAO = 0;
	unsigned int planeVBO = 0;
};
