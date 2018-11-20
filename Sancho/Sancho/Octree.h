#pragma once

#include <fstream>

#include "settings.h"
#include "common_includes.h"
#include "shader.h"

#include "Patch.h"

//class Patch {
//public:
//	Patch();
//	~Patch();
//
//	// article specifies position, orientation and size
//	double plane_dir1[3];
//	double plane_dir2[3];
//	double plane_norm[3];
//	double origin[3];
//	double quant_x, quant_y, quant_z, num_points;
//	int8_t* points;
//};

class Octree {
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Octree();
	//~Octree();

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
	void write_patches_to_file(const std::string file, const std::string file_b);
	void add_node_patches_to_vector(std::vector<Patch*> &patches);
	void print_patch_a(std::ofstream &file, Patch &patch);
	void print_patch_b(std::ofstream &file, Patch &patch);
	void print_patch_c(std::ofstream &file, std::ofstream &file_b, Patch &patch);
	void print_patch_d(std::ofstream &file, std::ofstream &file_b, Patch &patch);
	void calculate_patch_a();
	void calculate_patch_b();
	void calculate_patch_c();
	void calculate_patch_d();
	void leaf_distribution();
	void get_leaf_counts(std::map<int, float> &dist);
	Eigen::Matrix3d fast_covariance_matrix();

	std::vector<Eigen::Vector4d> m_points, m_colors;
	std::vector<int> m_indexes;
	std::vector<float> m_seperate_points;
	//std::vector<Patch> m_patches;

	Eigen::Matrix3d m_covariance;
	bool m_is_leaf;
	Octree *m_children, *m_root;
	Patch m_patch;
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
