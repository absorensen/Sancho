#pragma once

#define FREEGLUT_STATIC
#define _LIB
#define FREEGLUT_LIB_PRAGMAS 0

#include "kht3d_settings.h"
#include "common_includes.h"
#include "GL/glut.h"

class kht3d_octree {
public:

	kht3d_octree();

	void subdivide(kht3d_settings &settings);
	void least_variance_direction(void);
	void remove_outliers(void);
	void print_points(void);
	void show(bool type, int height);
	void show(bool type);
	void clear(void);
	void get_nodes(std::vector< kht3d_octree*> &nodes);
	double distance2plane(Eigen::VectorXd &point);
	Eigen::Matrix<double, 3, 3> fast_covariance_matrix(void);

	Eigen::Matrix<double, 3, 3> m_covariance;
	std::vector<Eigen::VectorXd> m_points, m_colors;
	std::vector<int> m_indexes;

	kht3d_octree *m_children, *m_root;
	Eigen::Vector3d normal1, normal2, normal3, m_middle;
	Eigen::Vector4d m_centroid, color;
	double variance1, variance2, variance3, m_size, representativeness;
	short m_level;
	bool coplanar;
	int votes;
};
