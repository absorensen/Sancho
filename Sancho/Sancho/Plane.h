#pragma once

#include "common_includes.h"
#include "Octree.h"

class Octree;

class Plane {
public:

	Plane(void) {}
	~Plane(void) {}

	inline void calculate()
	{

		m_normal.x = sin(m_phi) * cos(m_theta);
		m_normal.y = sin(m_phi) * sin(m_theta);
		m_normal.z = cos(m_phi);
		m_position = m_normal * m_rho;

		Eigen::VectorXd c_u(0.0, 0.0, 1.0, 1.0);
		if (c_u == m_normal)
			c_u = Eigen::VectorXd(1.0, 0.0, 0.0, 1.0);

		m_cross = (c_u * m_normal).normalized;
		m_cross2 = m_normal * m_cross;

		m_centroid = Eigen::VectorXd(0.0, 0.0, 0.0, 1.0);
		m_scale = Eigen::VectorXd(0.0, 0.0, 0.0, 0.0);
		m_showing = true;
		m_rotate = 0.0;
		int cont = 0;
		for (unsigned int i = 0; i < nodes.size(); i++)
		{
			cont += nodes[i]->m_indexes.size();
			for (unsigned int j = 0; j < nodes[i]->m_indexes.size(); j++)
			{
				m_centroid += nodes[i]->m_root->m_points[nodes[i]->m_indexes[j]];
			}
		}
		if (cont != 0)
			m_centroid /= (double)cont;

		m_desloc = m_centroid - m_position;
		if (m_desloc.norm() != 0) {
			Eigen::VectorXd normproj = (m_desloc.normalized & m_normal) * m_desloc.norm() * m_normal;
			m_desloc -= normproj;
		}


	}

	inline bool operator < (Plane p) { return (representativeness > p.representativeness); }

	inline void draw(double size, bool type, bool pc, bool selected)
	{

		if (selected) {
			glColor3d(0.0, 1.0, 0.0);
			glPointSize(6.0);
		}
		else {
			glColor3d(0.0, 0.0, 0.5);
			glPointSize(5.0);
		}

		if (pc) {
			if (type) {
				for (Octree *node : nodes)
				{
					glPushMatrix();
					glTranslated(node->m_middle.x, node->m_middle.y, node->m_middle.z);
					glutSolidCube(node->m_size + 0.5);
					glPopMatrix();
				}
			}
			else {
				for (Octree *node : nodes)
				{
					glBegin(GL_POINTS);
					for (size_t i = 0; i < node->m_indexes.size(); i++)
					{
						glVertex3d(node->m_root->m_points[node->m_indexes[i]].x, node->m_root->m_points[node->m_indexes[i]].y, node->m_root->m_points[node->m_indexes[i]].z);
					}
					glEnd();
				}
			}
		}

		if (m_showing) {
			glPushMatrix();
			{
				glBegin(GL_QUADS);
				if (selected)
					glColor4d(0.0, 0.0, 0.0, 0.6);
				else
					glColor4d(m_color.data.r, m_color.data.g, m_color.data.b, 0.4);

				Eigen::Rotation2Dd rot(m_rotate);
				Eigen::VectorXd cross = rot._transformVector(m_normal) * m_cross;
				Eigen::VectorXd cross2 = rot._transformVector(m_normal) * m_cross2;

				glNormal3dv(m_normal.data);
				glVertex3dv((m_position + cross * size + cross2 * size + m_desloc + cross * m_scale.x + cross2 * m_scale.z).data);
				glVertex3dv((m_position + cross * size - cross2 * size + m_desloc + cross * m_scale.x + cross2 * m_scale.w).data);
				glVertex3dv((m_position - cross * size - cross2 * size + m_desloc + cross * m_scale.y + cross2 * m_scale.w).data);
				glVertex3dv((m_position - cross * size + cross2 * size + m_desloc + cross * m_scale.y + cross2 * m_scale.z).data);

				glEnd();
			}
			glPopMatrix();
		}
	}

	double distance2plane(Eigen::VectorXd &point)
	{
		return abs((point - m_position).dot(m_normal.normalized()));
	}

	double m_theta;
	double m_phi;
	double m_rho;

	Eigen::VectorXd m_cross;
	Eigen::VectorXd m_cross2;
	Eigen::VectorXd m_position;
	Eigen::VectorXd m_centroid;
	Eigen::VectorXd m_normal;
	Eigen::VectorXd m_desloc;
	Eigen::VectorXd m_scale;
	Eigen::VectorXd m_color;

	std::vector<Eigen::VectorXd> m_points;


	double ti, m_rotate;
	int pi, ri;
	bool m_showing;

	std::vector<Octree*> nodes;
	ACCUM_BIN_TYPE votes;
	double representativeness;

};
