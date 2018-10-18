#pragma once

#include "settings.h"
#include "common_includes.h"
#include "shader.h"

class Octree {
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Octree();

	void subdivide(Settings &settings);
	void show(int height);
	void show();
	void clear(void);
	void get_nodes(std::vector< Octree*> &nodes);
	void draw_wire_cube(Eigen::Vector4d &middle, float size);

	std::vector<Eigen::Vector4d> m_points, m_colors;
	std::vector<int> m_indexes;

	Octree *m_children, *m_root;
	Settings* _settings;
	Eigen::Vector4d normal1, normal2, normal3, m_middle;
	Eigen::Vector4d m_centroid, color;
	double m_size;
	short m_level;

	// wire cube
	unsigned int wireCubeVAO = 0;
	unsigned int wireCubeVBO = 0;
};
