#pragma once
// based on the 3DKHT octree implementation
// http://www.inf.ufrgs.br/~oliveira/pubs_files/HT3D/HT3D_page.html

//#define FREEGLUT_STATIC
//#define _LIB
//#define FREEGLUT_LIB_PRAGMAS 0

#include "settings.h"
#include "common_includes.h"
//#include "GL/glut.h"

class Octree {
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Octree();

	void subdivide(Settings &settings);
	void least_variance_direction(void);
	void remove_outliers(void);
	void print_points(void);
	void show(bool type, int height);
	void show(bool type);
	void clear(void);
	void get_nodes(std::vector< Octree*> &nodes);
	double distance2plane(Eigen::Vector4d &point);
	Eigen::Matrix<double, 3, 3> fast_covariance_matrix(void);
	void draw_cube(Eigen::Vector4d &middle, double size, bool solid = true);

	Eigen::Matrix<double, 3, 3> m_covariance;
	std::vector<Eigen::Vector4d> m_points, m_colors;
	std::vector<int> m_indexes;

	Octree *m_children, *m_root;
	Eigen::Vector4d normal1, normal2, normal3, m_middle;
	Eigen::Vector4d m_centroid, color;
	double variance1, variance2, variance3, m_size, representativeness;
	short m_level;
	bool coplanar;
	int votes;
	unsigned int cubeVAO = 0;
	unsigned int cubeVBO = 0;

};
