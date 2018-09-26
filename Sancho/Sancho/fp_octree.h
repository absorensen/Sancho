#pragma once
#ifndef FP_OCTREE_H
#define FP_OCTREE_H

#include <Eigen/Dense>
#include "fp_settings.h"
#include "GL\glut.h"
#include <omp.h>
#include <vector>


class fp_octree {
public:

	fp_octree();

	void subdivide(fp_settings &settings);
	void least_variance_direction(void);
	void remove_outliers(void);
	void print_points(void);
	void show(bool type, int height);
	void show(bool type);
	void clear(void);
	void get_nodes(std::vector< fp_octree*> &nodes);
	double distance2plane(Eigen::Vector4d &point);
	Eigen::Matrix<double, 3, 3> fast_covariance_matrix(void);

	Eigen::Matrix<double, 3, 3> m_covariance;
	std::vector<Eigen::Vector4d> m_points, m_colors;
	std::vector<int> m_indexes;

	fp_octree * m_children, *m_root;
	Eigen::Vector4d normal1, normal2, normal3, m_middle, m_centroid, color;
	double variance1, variance2, variance3, m_size, representativeness;
	short m_level;
	bool coplanar;
	int votes;
};

#endif