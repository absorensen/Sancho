#pragma once

// based on the 3DKHT octree implementation
// http://www.inf.ufrgs.br/~oliveira/pubs_files/HT3D/HT3D_page.html

#include "Octree.h"

Octree::Octree()
{
	m_centroid = Eigen::Vector4d(0.0, 0.0, 0.0, 0.0);
	color = Eigen::Vector4d(0.5, 0.5, 0.5, 1.0);
	m_children = NULL;
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
	if (m_indexes.size() < (unsigned int)settings.s_ms) return;
	//if (m_indexes.size() < 9) return;

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
	//glm::vec3 translate(float(middle.x()*0.5f), float(middle.y()*0.5f), float(middle.z())*0.5f);


	glm::mat4 model = glm::translate(glm::mat4(1.0f), translate);
	model = glm::scale(model, glm::vec3(size, size, size));
	glm::mat4 MVPmatrix = projection * view * model;

	_settings->cube_shader->use();
	_settings->cube_shader->setMat4("mvp_matrix", MVPmatrix);
	_settings->cube_shader->setVec3("cam_pos", _settings->camera->Position);
	_settings->cube_shader->setFloat("point_size", _settings->point_size);

	// render Cube
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glBindVertexArray(wireCubeVAO);
	// should be solid/wire switch
	glDrawArrays(GL_LINES, 0, 48);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glBindVertexArray(0);
}
