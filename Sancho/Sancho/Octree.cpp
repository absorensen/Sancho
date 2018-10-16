#pragma once

// based on the 3DKHT octree implementation
// http://www.inf.ufrgs.br/~oliveira/pubs_files/HT3D/HT3D_page.html

#include "Octree.h"

Octree::Octree()
{
	m_centroid = Eigen::Vector4d(0.0, 0.0, 0.0, 0.0);
	color = Eigen::Vector4d(0.5, 0.5, 0.5, 1.0);
	m_children = NULL;
	coplanar = false;
	variance1 = variance2 = variance3 = 0.0;
	votes = 0;
}


void Octree::clear()
{
	if (m_children != NULL)
	{
		for (short i = 0; i < 8; i++)
		{
			m_children[i].m_indexes.clear();
			m_children[i].clear();
		}
		delete[] m_children;
		m_children = NULL;
	}
	m_children = NULL;
}

void Octree::subdivide(Settings &settings)
{

	// s_ms verification
	if (m_indexes.size() < (unsigned int)settings.s_ms) return;

	// s_level verification
	if (m_level >= settings.s_level)
	{
		// principal component analysis
		least_variance_direction();

		// Planarity verification
		double thickness = variance1 / variance2;
		double isotropy = variance2 / variance3;
		if (thickness < settings.max_thickness && isotropy > settings.min_isotropy) {

			// Refitting step
			remove_outliers();
			least_variance_direction();

			coplanar = true;
			return;
		}
	}

	m_children = new Octree[8];
	double newsize = m_size / 2.0;
	for (int i = 0; i < 8; i++) {
		m_children[i].m_size = newsize;
		m_children[i].m_level = m_level + 1;
		m_children[i].m_root = m_root;
		m_children[i].m_indexes.reserve(m_indexes.size() / 4);
	}

	double size4 = m_size / 4.0;

	// Calculation of son nodes
	m_children[0].m_middle(0) = m_middle(0) - size4;
	m_children[1].m_middle(0) = m_middle(0) - size4;
	m_children[2].m_middle(0) = m_middle(0) - size4;
	m_children[3].m_middle(0) = m_middle(0) - size4;
	m_children[4].m_middle(0) = m_middle(0) + size4;
	m_children[5].m_middle(0) = m_middle(0) + size4;
	m_children[6].m_middle(0) = m_middle(0) + size4;
	m_children[7].m_middle(0) = m_middle(0) + size4;

	m_children[0].m_middle(1) = m_middle(1) - size4;
	m_children[1].m_middle(1) = m_middle(1) - size4;
	m_children[2].m_middle(1) = m_middle(1) + size4;
	m_children[3].m_middle(1) = m_middle(1) + size4;
	m_children[4].m_middle(1) = m_middle(1) - size4;
	m_children[5].m_middle(1) = m_middle(1) - size4;
	m_children[6].m_middle(1) = m_middle(1) + size4;
	m_children[7].m_middle(1) = m_middle(1) + size4;
	
	m_children[0].m_middle(2) = m_middle(2) - size4;
	m_children[1].m_middle(2) = m_middle(2) + size4;
	m_children[2].m_middle(2) = m_middle(2) - size4;
	m_children[3].m_middle(2) = m_middle(2) + size4;
	m_children[4].m_middle(2) = m_middle(2) - size4;
	m_children[5].m_middle(2) = m_middle(2) + size4;
	m_children[6].m_middle(2) = m_middle(2) - size4;
	m_children[7].m_middle(2) = m_middle(2) + size4;

	// putting points in its respective children
	for (unsigned int i = 0; i < m_indexes.size(); i++)
	{
		unsigned int index = 0;
		if (m_root->m_points[m_indexes[i]](0) > m_middle(0))
		{
			index += 4;
		}
		if (m_root->m_points[m_indexes[i]](1) > m_middle(1))
		{
			index += 2;
		}
		if (m_root->m_points[m_indexes[i]](2) > m_middle(2))
		{
			index += 1;
		}
		m_children[index].m_indexes.push_back(m_indexes[i]);
		// Calculating centroid distribution (divided by the number of points below)
		m_children[index].m_centroid += m_root->m_points[m_indexes[i]];
	}

#pragma omp parallel for 
	for (int i = 0; i < 8; i++) {
		m_children[i].m_centroid /= m_children[i].m_indexes.size();

		// Recursive subdivision 
		m_children[i].subdivide(settings);
	}
}

void Octree::remove_outliers()
{
	Eigen::Vector4d centroid;
	for (int i = m_indexes.size() - 1; i >= 0; i--)
	{
		if (distance2plane(m_root->m_points[m_indexes[i]]) > m_size / 10.0) {
			m_indexes.erase(m_indexes.begin() + i);
		}
		else {
			centroid += m_root->m_points[m_indexes[i]];
		}
	}
	if (m_indexes.size() > 0)
		m_centroid = centroid / m_indexes.size();
}

Eigen::Matrix < double, 3, 3> Octree::fast_covariance_matrix()
{
	unsigned int nverts = m_indexes.size();
	double nvertsd = (double)(nverts);
	Eigen::Matrix<double, 3, 3> covariance(3, 3);


	covariance(0, 0) = 0.0;
	for (size_t k = 0; k < nverts; k++)
		covariance(0, 0) += (m_root->m_points[m_indexes[k]][0] - m_centroid[0]) * (m_root->m_points[m_indexes[k]][0] - m_centroid[0]);
	covariance(0, 0) /= (nvertsd);
	if (fabs(m_covariance(0, 0)) < EPS)
		m_covariance(0, 0) = 0.0;

	covariance(1, 1) = 0.0;
	for (size_t k = 0; k < nverts; k++)
		covariance(1, 1) += (m_root->m_points[m_indexes[k]][1] - m_centroid[1]) * (m_root->m_points[m_indexes[k]][1] - m_centroid[1]);
	covariance(1, 1) /= (nvertsd);
	if (fabs(m_covariance(1, 1)) < EPS)
		m_covariance(1, 1) = 0.0;

	covariance(2, 2) = 0.0;
	for (size_t k = 0; k < nverts; k++)
		covariance(2, 2) += (m_root->m_points[m_indexes[k]][2] - m_centroid[2]) * (m_root->m_points[m_indexes[k]][2] - m_centroid[2]);
	covariance(2, 2) /= (nvertsd);
	if (fabs(m_covariance(2, 2)) < EPS)
		m_covariance(2, 2) = 0.0;

	covariance(1, 0) = 0.0;
	for (size_t k = 0; k < nverts; k++)
		covariance(1, 0) += (m_root->m_points[m_indexes[k]][1] - m_centroid[1]) * (m_root->m_points[m_indexes[k]][0] - m_centroid[0]);
	covariance(1, 0) /= (nvertsd);
	if (fabs(m_covariance(1, 0)) < EPS)
		m_covariance(1, 0) = 0.0;

	covariance(2, 0) = 0.0;
	for (size_t k = 0; k < nverts; k++)
		covariance(2, 0) += (m_root->m_points[m_indexes[k]][2] - m_centroid[2]) * (m_root->m_points[m_indexes[k]][0] - m_centroid[0]);
	covariance(2, 0) /= (nvertsd);
	if (fabs(m_covariance(2, 0)) < EPS)
		m_covariance(2, 0) = 0.0;

	covariance(2, 1) = 0.0;
	for (size_t k = 0; k < nverts; k++)
		covariance(2, 1) += (m_root->m_points[m_indexes[k]][2] - m_centroid[2]) * (m_root->m_points[m_indexes[k]][1] - m_centroid[1]);
	covariance(2, 1) /= (nvertsd);
	if (fabs(m_covariance(2, 1)) < EPS)
		m_covariance(2, 1) = 0.0;

	covariance(0, 2) = covariance(2, 0);
	covariance(0, 1) = covariance(1, 0);
	covariance(1, 2) = covariance(2, 1);

	return covariance;
}

void Octree::least_variance_direction()
{
	m_covariance = fast_covariance_matrix();

	Eigen::EigenSolver< Eigen::Matrix<double, 3, 3> > eigenvalue_decomp(m_covariance);
	Eigen::Vector3d eigenvalues_vector = eigenvalue_decomp.eigenvalues().real();

	int min_index = 0, max_index = 0, middle_index = 0;

	if (eigenvalues_vector(1) < eigenvalues_vector(min_index)) {
		min_index = 1;
	}
	else if (eigenvalues_vector(1) > eigenvalues_vector(max_index)) {
		max_index = 1;
	}

	if (eigenvalues_vector(2) < eigenvalues_vector(min_index)) {
		min_index = 2;
	}
	else if (eigenvalues_vector(2) > eigenvalues_vector(max_index)) {
		max_index = 2;
	}

	while (middle_index == min_index || middle_index == max_index) middle_index++;

	variance1 = eigenvalues_vector(min_index);
	variance2 = eigenvalues_vector(middle_index);
	variance3 = eigenvalues_vector(max_index);

	Eigen::Matrix3d eigenvectors_matrix = eigenvalue_decomp.pseudoEigenvectors();

	normal1 = Eigen::Vector4d(eigenvectors_matrix(0, min_index), eigenvectors_matrix(1, min_index), eigenvectors_matrix(2, min_index), 1.0);
	normal2 = Eigen::Vector4d(eigenvectors_matrix(0, middle_index), eigenvectors_matrix(1, middle_index), eigenvectors_matrix(2, middle_index), 1.0);
	normal3 = Eigen::Vector4d(eigenvectors_matrix(0, max_index), eigenvectors_matrix(1, max_index), eigenvectors_matrix(2, max_index), 1.0);
}

double Octree::distance2plane(Eigen::Vector4d &point)
{
	return abs((point - m_centroid).dot(normal1.normalized()));
}

void Octree::get_nodes(std::vector< Octree*> &nodes)
{
	if (m_children != NULL)
	{
		for (short i = 0; i < 8; i++)
		{
			m_children[i].get_nodes(nodes);
		}
	}
	else {
		if (coplanar) {
			nodes.push_back(this);
		}
	}
}

void Octree::print_points()
{
	glPointSize(4.0);
	glBegin(GL_POINTS);
	glNormal3d(1.0, 0.0, 0.0);
	glColor3dv(color.data());
	for (size_t i = 0; i < m_indexes.size(); i++)
	{
		glVertex3d(m_root->m_points[m_indexes[i]](0), m_root->m_points[m_indexes[i]](1), m_root->m_points[m_indexes[i]](2));
	}
	glEnd();
	glPointSize(1.0);
}

void Octree::show(bool type, int height)
{
	height--;
	if (height == 0) {
		glColor3dv(color.data());
		if (type) {
			glPushMatrix();
			glTranslated(m_middle(0), m_middle(1), m_middle(2));
			if (color == Eigen::Vector4d(0.5, 0.5, 0.5, 1.0)) {
				// wire
				draw_cube(m_middle, m_size, false);
			}
			else {
				// solid
				draw_cube(m_middle, m_size);
			}
			glPopMatrix();
		}
		else {
			print_points();
		}
	}
	if (m_children != NULL)
	{
		for (short i = 0; i < 8; i++)
		{
			m_children[i].show(type, height);
		}
	}
}

void Octree::show(bool type)
{
	if (coplanar) {
		glColor3dv(color.data());
		if (type) {
			glPushMatrix();
			glTranslated(m_middle(0), m_middle(1), m_middle(2));

			// wire
			draw_cube(m_middle, m_size, false);

			glPopMatrix();
		}
		else {
			print_points();
		}
	}

	if (m_children != NULL)
	{
		for (short i = 0; i < 8; i++)
		{
			m_children[i].show(type);
		}
	}
}


// needs to be tested for solid draw first and then for wire draw
void Octree::draw_cube(Eigen::Vector4d &middle, double size, bool solid) {
	// initialize (if necessary)
	if (cubeVAO == 0)
	{
		float vertices[] = {
			// back face
			-1.0f, -1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 0.0f, 0.0f, // bottom-left
			 1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 1.0f, 1.0f, // top-right
			 1.0f, -1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 1.0f, 0.0f, // bottom-right         
			 1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 1.0f, 1.0f, // top-right
			-1.0f, -1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 0.0f, 0.0f, // bottom-left
			-1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 0.0f, 1.0f, // top-left
			// front face
			-1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f, 0.0f, // bottom-left
			 1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 1.0f, 0.0f, // bottom-right
			 1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 1.0f, 1.0f, // top-right
			 1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 1.0f, 1.0f, // top-right
			-1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f, 1.0f, // top-left
			-1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f, 0.0f, // bottom-left
			// left face
			-1.0f,  1.0f,  1.0f, -1.0f,  0.0f,  0.0f, 1.0f, 0.0f, // top-right
			-1.0f,  1.0f, -1.0f, -1.0f,  0.0f,  0.0f, 1.0f, 1.0f, // top-left
			-1.0f, -1.0f, -1.0f, -1.0f,  0.0f,  0.0f, 0.0f, 1.0f, // bottom-left
			-1.0f, -1.0f, -1.0f, -1.0f,  0.0f,  0.0f, 0.0f, 1.0f, // bottom-left
			-1.0f, -1.0f,  1.0f, -1.0f,  0.0f,  0.0f, 0.0f, 0.0f, // bottom-right
			-1.0f,  1.0f,  1.0f, -1.0f,  0.0f,  0.0f, 1.0f, 0.0f, // top-right
			// right face
			 1.0f,  1.0f,  1.0f,  1.0f,  0.0f,  0.0f, 1.0f, 0.0f, // top-left
			 1.0f, -1.0f, -1.0f,  1.0f,  0.0f,  0.0f, 0.0f, 1.0f, // bottom-right
			 1.0f,  1.0f, -1.0f,  1.0f,  0.0f,  0.0f, 1.0f, 1.0f, // top-right         
			 1.0f, -1.0f, -1.0f,  1.0f,  0.0f,  0.0f, 0.0f, 1.0f, // bottom-right
			 1.0f,  1.0f,  1.0f,  1.0f,  0.0f,  0.0f, 1.0f, 0.0f, // top-left
			 1.0f, -1.0f,  1.0f,  1.0f,  0.0f,  0.0f, 0.0f, 0.0f, // bottom-left     
			// bottom face
			-1.0f, -1.0f, -1.0f,  0.0f, -1.0f,  0.0f, 0.0f, 1.0f, // top-right
			 1.0f, -1.0f, -1.0f,  0.0f, -1.0f,  0.0f, 1.0f, 1.0f, // top-left
			 1.0f, -1.0f,  1.0f,  0.0f, -1.0f,  0.0f, 1.0f, 0.0f, // bottom-left
			 1.0f, -1.0f,  1.0f,  0.0f, -1.0f,  0.0f, 1.0f, 0.0f, // bottom-left
			-1.0f, -1.0f,  1.0f,  0.0f, -1.0f,  0.0f, 0.0f, 0.0f, // bottom-right
			-1.0f, -1.0f, -1.0f,  0.0f, -1.0f,  0.0f, 0.0f, 1.0f, // top-right
			// top face
			-1.0f,  1.0f, -1.0f,  0.0f,  1.0f,  0.0f, 0.0f, 1.0f, // top-left
			 1.0f,  1.0f , 1.0f,  0.0f,  1.0f,  0.0f, 1.0f, 0.0f, // bottom-right
			 1.0f,  1.0f, -1.0f,  0.0f,  1.0f,  0.0f, 1.0f, 1.0f, // top-right     
			 1.0f,  1.0f,  1.0f,  0.0f,  1.0f,  0.0f, 1.0f, 0.0f, // bottom-right
			-1.0f,  1.0f, -1.0f,  0.0f,  1.0f,  0.0f, 0.0f, 1.0f, // top-left
			-1.0f,  1.0f,  1.0f,  0.0f,  1.0f,  0.0f, 0.0f, 0.0f  // bottom-left        
		};
		glGenVertexArrays(1, &cubeVAO);
		glGenBuffers(1, &cubeVBO);
		// fill buffer
		glBindBuffer(GL_ARRAY_BUFFER, cubeVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
		// link vertex attributes
		glBindVertexArray(cubeVAO);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
		glEnableVertexAttribArray(2);
		glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
	}
	// render Cube
	glBindVertexArray(cubeVAO);
	// should be solid/wire switch
	if (true) glDrawArrays(GL_TRIANGLES, 0, 36);
	glBindVertexArray(0);
}