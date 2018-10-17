#pragma once
// based on the 3DKHT octree implementation
// http://www.inf.ufrgs.br/~oliveira/pubs_files/HT3D/HT3D_page.html

//#define FREEGLUT_STATIC
//#define _LIB
//#define FREEGLUT_LIB_PRAGMAS 0

#include "settings.h"
#include "common_includes.h"
#include "shader.h"
//#include "GL/glut.h"

class Octree {
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Octree();
	
	void subdivide(Settings &settings);
	void least_variance_direction(void);
	void remove_outliers(void);
	void print_points(void);
	void show(int height);
	void show();
	void clear(void);
	void get_nodes(std::vector< Octree*> &nodes);
	double distance2plane(Eigen::Vector4d &point);
	Eigen::Matrix<double, 3, 3> fast_covariance_matrix(void);
	void draw_wire_cube(Eigen::Vector4d &middle, float size);
	void draw_solid_cube(Eigen::Vector4d &middle, float size);

	Eigen::Matrix<double, 3, 3> m_covariance;
	std::vector<Eigen::Vector4d> m_points, m_colors;
	std::vector<int> m_indexes;

	Octree *m_children, *m_root;
	Settings* _settings;
	Eigen::Vector4d normal1, normal2, normal3, m_middle;
	Eigen::Vector4d m_centroid, color;
	double variance1, variance2, variance3, m_size, representativeness;
	short m_level;
	bool coplanar;
	int votes;
	
	// solid cube
	unsigned int solidCubeVAO = 0;
	unsigned int solidCubeVBO = 0;

	// wire cube
	unsigned int wireCubeVAO = 0;
	unsigned int wireCubeVBO = 0;
};
