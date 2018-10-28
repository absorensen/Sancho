#pragma once

// based on the 3DKHT octree implementation
// http://www.inf.ufrgs.br/~oliveira/pubs_files/HT3D/HT3D_page.html

#include "Octree.h"

class Octree;
struct Patch;

Octree::Octree()
{
	m_centroid = Eigen::Vector4d(0.0, 0.0, 0.0, 0.0);
	color = Eigen::Vector4d(0.5, 0.5, 0.5, 1.0);
	m_children = NULL;
	m_is_leaf = false;
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
	_settings = &settings;
	// s_ms verification
	if (m_indexes.size() < 9) {
	//if (m_indexes.size() < (unsigned int)settings.s_ms) {
		m_is_leaf = true;
		least_variance_direction();
		return;
	}
	//if (m_indexes.size() < 9) return;
	if (false)
	//if (m_indexes.size() < settings.max_points_leaf)
	{
		// principal component analysis
		least_variance_direction();

		// plane data
		Patch patch;
		patch.plane_dir1[0] = normal1.x();
		patch.plane_dir1[1] = normal1.y();
		patch.plane_dir1[2] = normal1.z();
		patch.plane_dir1[3] = 1.0;

		patch.plane_dir2[0] = normal2.x();
		patch.plane_dir2[1] = normal2.y();
		patch.plane_dir2[2] = normal2.z();
		patch.plane_dir2[3] = 1.0;

		patch.plane_norm[0] = normal3.x();
		patch.plane_norm[1] = normal3.y();
		patch.plane_norm[2] = normal3.z();
		patch.plane_norm[3] = 1.0;

		double max, mix, may, miy, maz, miz, max_z, min_z;
		max = mix = may = miy = maz = miz = 0.0;

		bool max_z_init = false;
		bool min_z_init = false;

		const int num_points = m_indexes.size();
		Eigen::Vector4d plane_z_axis_projection;
		double projection_length;
		for (unsigned int i = 0; i < m_indexes.size(); ++i) {
			max = max > m_root->m_points[m_indexes[i]].x() ? max : m_root->m_points[m_indexes[i]].x();
			mix = mix < m_root->m_points[m_indexes[i]].x() ? mix : m_root->m_points[m_indexes[i]].x();

			may = may > m_root->m_points[m_indexes[i]].y() ? may : m_root->m_points[m_indexes[i]].y();
			miy = miy < m_root->m_points[m_indexes[i]].y() ? miy : m_root->m_points[m_indexes[i]].y();

			maz = maz > m_root->m_points[m_indexes[i]].z() ? maz : m_root->m_points[m_indexes[i]].z();
			miz = miz < m_root->m_points[m_indexes[i]].z() ? miz : m_root->m_points[m_indexes[i]].z();


			// use a simple distance function - only works if m_centroid indeed represents the best fitting plane
			projection_length = signed_plane_distance(m_root->m_points[m_indexes[i]]);

			if (!max_z_init) {
				max_z_init = true;
				max_z = projection_length;
			}
			if (!min_z_init) {
				min_z_init = true;
				min_z = projection_length;
			}

			max_z = projection_length > max_z ? projection_length : max_z;
			min_z = projection_length < min_z ? projection_length : min_z;
		}

		// re-orient patch with ep1 = n×(1, 0, 0), ep2 = n×ep1, ep3 = n
		// As the coordinate system’s origin we choose the point cluster’s center of mass.
		// needs to be tested -
		Eigen::Vector3d x(1.0, 0.0, 0.0);
		Eigen::Vector3d n(normal3.x(), normal3.y(), normal3.z());
		Eigen::Vector3d ep1 = n.cross(x);
		Eigen::Vector3d ep2 = n.cross(ep1);
		Eigen::Vector3d ep3 = n;

		patch.origin[0] = m_centroid.x();
		patch.origin[1] = m_centroid.y();
		patch.origin[2] = m_centroid.z();
		patch.origin[3] = m_centroid.w();


		// create height map

		// create occlusion map



		return;
	}

	m_children = new Octree[8];
	double newsize = m_size * 0.5;
	for (int i = 0; i < 8; ++i) {
		m_children[i].m_size = newsize;
		m_children[i].m_level = m_level + 1;
		m_children[i].m_root = m_root;
		m_children[i].m_indexes.reserve(m_indexes.size() / 4);
	}

	double size4 = m_size * 0.25;

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
	for (unsigned int i = 0; i < m_indexes.size(); ++i)
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
	for (int i = 0; i < 8; ++i) {
		m_children[i].m_centroid /= m_children[i].m_indexes.size();

		// Recursive subdivision 
		m_children[i].subdivide(settings);
	}
}

Eigen::Matrix3d Octree::fast_covariance_matrix()
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

	Eigen::EigenSolver< Eigen::Matrix3d > eigenvalue_decomp(m_covariance);
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

	while (middle_index == min_index || middle_index == max_index) ++middle_index;

	variance1 = eigenvalues_vector(min_index);
	variance2 = eigenvalues_vector(middle_index);
	variance3 = eigenvalues_vector(max_index);

	Eigen::Matrix3d eigenvectors_matrix = eigenvalue_decomp.pseudoEigenvectors();

	normal1 = Eigen::Vector4d(eigenvectors_matrix(0, min_index), eigenvectors_matrix(1, min_index), eigenvectors_matrix(2, min_index), 1.0);
	normal1.normalize();
	normal2 = Eigen::Vector4d(eigenvectors_matrix(0, middle_index), eigenvectors_matrix(1, middle_index), eigenvectors_matrix(2, middle_index), 1.0);
	normal2.normalize();
	normal3 = Eigen::Vector4d(eigenvectors_matrix(0, max_index), eigenvectors_matrix(1, max_index), eigenvectors_matrix(2, max_index), 1.0);
	normal3.normalize();
}

double Octree::plane_distance(Eigen::Vector4d &point)
{
	return abs((point - m_centroid).dot(normal1.normalized()));
}

double Octree::signed_plane_distance(Eigen::Vector4d &point)
{
	return (point - m_centroid).dot(normal1.normalized());
}


void Octree::get_nodes(std::vector< Octree*> &nodes)
{
	if (m_children != NULL)
	{
		for (short i = 0; i < 8; ++i)
		{
			m_children[i].get_nodes(nodes);
		}
	}
}


void Octree::show_level(int height) {
	draw_points();
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	show(height);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void Octree::show(int height)
{
	height--;
	if (height == 0) {
		draw_wire_cube(m_middle, float(m_size));
		return;
	}
	if (m_children != NULL)
	{
		for (short i = 0; i < 8; ++i)
		{
			m_children[i].show(height);
		}
	}
}

void Octree::show_tree() {
	draw_points();
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	show();
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void Octree::show()
{
	draw_wire_cube(m_middle, float(m_size));
	if (m_children != NULL)
	{
		for (short i = 0; i < 8; ++i)
		{
			m_children[i].show();
		}
	}
}

void Octree::show_normals(const float length) {
	if (m_is_leaf) draw_normal(length);
	if (m_children != NULL)
	{
		for (short i = 0; i < 8; ++i)
		{
			m_children[i].show_normals(length);
		}
	}
}

void Octree::draw_points() {
	// initialize (if necessary)
	if (pointsVAO == 0)
	{
		_settings->point_shader->use();
		_settings->point_shader->setVec3("color", 0.9f, 0.4f, 0.0f);
		_settings->point_shader->setFloat("point_size", _settings->point_size);
		_settings->point_shader->setFloat("z_near", _settings->Z_NEAR);
		_settings->point_shader->setFloat("z_far", _settings->Z_FAR);
		_settings->point_shader->setFloat("height_of_near_plane", _settings->height_of_near_plane);
		float* points = new float[3 * m_indexes.size()];
		int j = 0;
		for (size_t i = 0; i < m_indexes.size(); ++i)
		{
			points[j] = float(m_root->m_points[m_indexes[i]](0));
			++j;
			points[j] = float(m_root->m_points[m_indexes[i]](1));
			++j;
			points[j] = float(m_root->m_points[m_indexes[i]](2));
			++j;
		}

		glGenVertexArrays(1, &pointsVAO);
		glGenBuffers(1, &pointsVBO);
		glBindVertexArray(pointsVAO);

		glBindBuffer(GL_ARRAY_BUFFER, pointsVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*m_indexes.size()*3, points, GL_STATIC_DRAW);
		GLint position_attribute = glGetAttribLocation(_settings->point_shader->ID, "aPos");
		glVertexAttribPointer(position_attribute, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (void*)0);
		glEnableVertexAttribArray(0);

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
	}

	glm::mat4 projection = glm::perspective(glm::radians(_settings->camera->Zoom), (float)_settings->SCR_WIDTH / (float)_settings->SCR_HEIGHT, _settings->Z_NEAR, _settings->Z_FAR);
	glm::mat4 view = _settings->camera->GetViewMatrix();
	glm::mat4 model = glm::mat4(1.0f);
	glm::mat4 MVPmatrix = projection * view * model;

	_settings->point_shader->use();
	_settings->point_shader->setMat4("mvp_matrix", MVPmatrix);
	_settings->point_shader->setVec3("cam_pos", _settings->camera->Position);
	_settings->point_shader->setFloat("point_size", _settings->point_size);

	glBindVertexArray(pointsVAO);
	glDrawArrays(GL_POINTS, 0, m_indexes.size());
	glBindVertexArray(0);
}

void Octree::draw_wire_cube(Eigen::Vector4d &middle, float size) {
	// initialize (if necessary)
	if (wireCubeVAO == 0)
	{
		_settings->cube_shader->use();
		_settings->cube_shader->setVec3("color", 0.0f, 0.9f, 0.6f);
		_settings->cube_shader->setFloat("point_size", _settings->point_size);
		_settings->cube_shader->setFloat("z_near", _settings->Z_NEAR);
		_settings->cube_shader->setFloat("z_far", _settings->Z_FAR);
		_settings->cube_shader->setFloat("height_of_near_plane", _settings->height_of_near_plane);
		float vertices[] = {
			// back face
			-0.5f, -0.5f, -0.5f, // bottom-left
			 0.5f, -0.5f, -0.5f, // bottom-right     

			 0.5f, -0.5f, -0.5f, // bottom-right         
			 0.5f,  0.5f, -0.5f, // top-right

			 0.5f,  0.5f, -0.5f, // top-right
			-0.5f,  0.5f, -0.5f, // top-left

			-0.5f,  0.5f, -0.5f, // top-left
			-0.5f, -0.5f, -0.5f, // bottom-left

			// front face
			-0.5f, -0.5f,  0.5f, // bottom-left
			 0.5f, -0.5f,  0.5f, // bottom-right

			 0.5f, -0.5f,  0.5f, // bottom-right
			 0.5f,  0.5f,  0.5f, // top-right

			 0.5f,  0.5f,  0.5f, // top-right
			-0.5f,  0.5f,  0.5f, // top-left

			-0.5f,  0.5f,  0.5f, // top-left
			-0.5f, -0.5f,  0.5f, // bottom-left

			// left face
			-0.5f,  0.5f,  0.5f, // top-right
			-0.5f,  0.5f, -0.5f, // top-left

			-0.5f,  0.5f, -0.5f, // top-left
			-0.5f, -0.5f, -0.5f, // bottom-left

			-0.5f, -0.5f, -0.5f, // bottom-left
			-0.5f, -0.5f,  0.5f, // bottom-right

			-0.5f, -0.5f,  0.5f, // bottom-right
			-0.5f,  0.5f,  0.5f, // top-right

			// right face
			 0.5f,  0.5f,  0.5f, // top-left
			 0.5f,  0.5f, -0.5f, // top-right

			 0.5f,  0.5f, -0.5f, // top-right
			 0.5f, -0.5f, -0.5f, // bottom-right

			 0.5f, -0.5f, -0.5f, // bottom-right
			 0.5f, -0.5f,  0.5f, // bottom-left

			 0.5f, -0.5f,  0.5f, // bottom-left
			 0.5f,  0.5f,  0.5f, // top-left

			// bottom face
			-0.5f, -0.5f, -0.5f, // top-right
			 0.5f, -0.5f, -0.5f, // top-left

			 0.5f, -0.5f, -0.5f, // top-left
			 0.5f, -0.5f,  0.5f, // bottom-left

			 0.5f, -0.5f,  0.5f, // bottom-left
			-0.5f, -0.5f,  0.5f, // bottom-right

			-0.5f, -0.5f,  0.5f, // bottom-right
			-0.5f, -0.5f, -0.5f, // top-right


			// top face
			-0.5f,  0.5f, -0.5f, // top-left
			 0.5f,  0.5f, -0.5f, // top-right     

			 0.5f,  0.5f, -0.5f, // top-right
			 0.5f,  0.5f , 0.5f, // bottom-right

			 0.5f,  0.5f,  0.5f, // bottom-right
			-0.5f,  0.5f,  0.5f, // bottom-left      

			-0.5f,  0.5f,  0.5f, // bottom-left      
			-0.5f,  0.5f, -0.5f  // top-left


		};

		glGenVertexArrays(1, &wireCubeVAO);
		glGenBuffers(1, &wireCubeVBO);
		// fill buffer
		glBindBuffer(GL_ARRAY_BUFFER, wireCubeVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
		// link vertex attributes
		glBindVertexArray(wireCubeVAO);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
	}

	glm::mat4 projection = glm::perspective(glm::radians(_settings->camera->Zoom), (float)_settings->SCR_WIDTH / (float)_settings->SCR_HEIGHT, _settings->Z_NEAR, _settings->Z_FAR);
	glm::mat4 view = _settings->camera->GetViewMatrix();
	glm::vec3 translate(float(middle.x()), float(middle.y()), float(middle.z()));
	glm::mat4 model = glm::translate(glm::mat4(1.0f), translate);
	model = glm::scale(model, glm::vec3(size, size, size));
	glm::mat4 MVPmatrix = projection * view * model;

	_settings->cube_shader->use();
	_settings->cube_shader->setMat4("mvp_matrix", MVPmatrix);
	_settings->cube_shader->setVec3("cam_pos", _settings->camera->Position);
	_settings->cube_shader->setFloat("point_size", _settings->point_size);
	//_settings->cube_shader->setVec3("color", float(std::rand())/RAND_MAX, float(std::rand()) / RAND_MAX, float(std::rand()) / RAND_MAX);

	// render Cube
	glBindVertexArray(wireCubeVAO);
	glDrawArrays(GL_LINES, 0, 48);
	glBindVertexArray(0);
}

void Octree::draw_normal(const float length) {
	// initialize (if necessary)
	if (normalVAO == 0)
	{
		_settings->normals_shader->use();
		_settings->normals_shader->setVec3("color", 0.8f, 0.0f, 0.7f);
		_settings->normals_shader->setFloat("point_size", _settings->point_size);
		_settings->normals_shader->setFloat("z_near", _settings->Z_NEAR);
		_settings->normals_shader->setFloat("z_far", _settings->Z_FAR);
		_settings->normals_shader->setFloat("height_of_near_plane", _settings->height_of_near_plane);
		float vertices[] = {
			float(m_centroid.x()), float(m_centroid.y()), float(m_centroid.z()), // bottom-left
			float(m_centroid.x()+normal1.x()), float(m_centroid.y() + normal1.y()), float(m_centroid.z() + normal1.z()) // bottom-right     
		};

		glGenVertexArrays(1, &normalVAO);
		glGenBuffers(1, &normalVBO);
		// fill buffer
		glBindBuffer(GL_ARRAY_BUFFER, normalVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
		// link vertex attributes
		glBindVertexArray(normalVAO);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
	}

	glm::mat4 projection = glm::perspective(glm::radians(_settings->camera->Zoom), (float)_settings->SCR_WIDTH / (float)_settings->SCR_HEIGHT, _settings->Z_NEAR, _settings->Z_FAR);
	glm::mat4 view = _settings->camera->GetViewMatrix();
	glm::vec3 translate(float(m_centroid.x()), float(m_centroid.y()), float(m_centroid.z()));
	glm::mat4 model = glm::translate(glm::mat4(1.0f), translate);
	model = glm::scale(model, glm::vec3(length, length, length));
	glm::mat4 MVPmatrix = projection * view * model;

	_settings->normals_shader->use();
	_settings->normals_shader->setMat4("mvp_matrix", MVPmatrix);
	_settings->normals_shader->setVec3("cam_pos", _settings->camera->Position);
	_settings->normals_shader->setFloat("point_size", _settings->point_size);
	//_settings->cube_shader->setVec3("color", float(std::rand())/RAND_MAX, float(std::rand()) / RAND_MAX, float(std::rand()) / RAND_MAX);

	// render Cube
	glBindVertexArray(normalVAO);
	glDrawArrays(GL_LINES, 0, 2);
	glBindVertexArray(0);
}
