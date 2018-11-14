#pragma once

// based on the 3DKHT octree implementation
// http://www.inf.ufrgs.br/~oliveira/pubs_files/HT3D/HT3D_page.html

#include "Octree.h"

class Octree;
class Patch;

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

void Octree::write_patches_to_file(const std::string file_name) {
	std::ofstream file;
	if(_settings->easily_decodeable) file.open(file_name.c_str());
	else file.open(file_name.c_str(), std::ios::out | std::ios::trunc | std::ios::binary);
	std::vector<Patch*> patches;
	add_node_patches_to_vector(patches);
	print_patch_vector(file, patches);
	//file << "E";
	file.close();
}

void Octree::print_patch_vector(std::ofstream &file, std::vector<Patch*> &patches) {
	const int size = patches.size();
	for (int i = 0; i < size; ++i) {
		print_patch(file, *patches[i]);
	}
}

void Octree::print_patch(std::ofstream &file, Patch &patch) {
	
	if (_settings->easily_decodeable) {
		// origin
		file << patch.origin[0] << " ";
		file << patch.origin[1] << " ";
		file << patch.origin[2] << " ";
		
		// plane dir1
		file << patch.plane_dir1[0] << " ";
		file << patch.plane_dir1[1] << " ";
		file << patch.plane_dir1[2] << " ";
		
		// plane dir2
		file << patch.plane_dir2[0] << " ";
		file << patch.plane_dir2[1] << " ";
		file << patch.plane_dir2[2] << " ";

		// plane norm
		file << patch.plane_norm[0] << " ";
		file << patch.plane_norm[1] << " ";
		file << patch.plane_norm[2] << " ";

		// quants
		file << patch.quant_x << " ";
		file << patch.quant_y << " ";
		file << patch.quant_z << " ";

		// number of points
		file << patch.num_points;

		const int num_coords = 3 * patch.num_points;
		for (int i = 0; i < num_coords; ++i) {
			file << patch.points[i] << " ";
		}
		file << "\n";
	} else {
		// origin
		file.write((char*)&(patch.origin[0]), sizeof(patch.origin[0]));
		file.write((char*)&(patch.origin[1]), sizeof(patch.origin[1]));
		file.write((char*)&(patch.origin[2]), sizeof(patch.origin[2]));

		// plane dir1
		file.write((char*)&(patch.plane_dir1[0]), sizeof(patch.plane_dir1[0]));
		file.write((char*)&(patch.plane_dir1[1]), sizeof(patch.plane_dir1[1]));
		file.write((char*)&(patch.plane_dir1[2]), sizeof(patch.plane_dir1[2]));

		// plane dir2
		file.write((char*)&(patch.plane_dir2[0]), sizeof(patch.plane_dir2[0]));
		file.write((char*)&(patch.plane_dir2[1]), sizeof(patch.plane_dir2[1]));
		file.write((char*)&(patch.plane_dir2[2]), sizeof(patch.plane_dir2[2]));

		// plane norm
		file.write((char*)&(patch.plane_norm[0]), sizeof(patch.plane_norm[0]));
		file.write((char*)&(patch.plane_norm[1]), sizeof(patch.plane_norm[1]));		
		file.write((char*)&(patch.plane_norm[2]), sizeof(patch.plane_norm[2]));

		// quants
		file.write((char*)&(patch.quant_x), sizeof(patch.quant_x));
		file.write((char*)&(patch.quant_y), sizeof(patch.quant_y));
		file.write((char*)&(patch.quant_z), sizeof(patch.quant_z));

		// number of points
		file.write((char*)&(patch.num_points), sizeof(patch.num_points));

		const int num_coords = 3 * patch.num_points;
		for (int i = 0; i < num_coords; ++i) {
			file.write((char*)&(patch.points[i]), sizeof(patch.points[i]));
		}
		//file << "\n";
	}
}

void Octree::add_node_patches_to_vector(std::vector<Patch*> &patches) {
	if (m_is_leaf) { 
		patches.push_back(&m_patch); 
		return;
	}
	if (m_children != NULL)
	{
		for (short i = 0; i < 8; ++i)
		{
			m_children[i].add_node_patches_to_vector(patches);
		}
	}
}

void Octree::subdivide(Settings &settings)
{
	_settings = &settings;
	// s_ms verification
	//if (m_indexes.size() < 33) {
	////if (m_indexes.size() < (unsigned int)settings.s_ms) {
	//	m_is_leaf = true;
	//	least_variance_direction();
	//	return;
	//}
	//if(false)
	if (m_indexes.size() == 0) return;
	if (m_indexes.size() <= _settings->max_points_leaf)
	{
		m_is_leaf = true;

		// principal component analysis
		least_variance_direction();

		// plane data
		m_patch = new Patch;
		m_patch.plane_norm[0] = static_cast<float>(normal1.x());
		m_patch.plane_norm[1] = static_cast<float>(normal1.y());
		m_patch.plane_norm[2] = static_cast<float>(normal1.z());

		m_patch.plane_dir1[0] = static_cast<float>(normal2.x());
		m_patch.plane_dir1[1] = static_cast<float>(normal2.y());
		m_patch.plane_dir1[2] = static_cast<float>(normal2.z());

		m_patch.plane_dir2[0] = static_cast<float>(normal3.x());
		m_patch.plane_dir2[1] = static_cast<float>(normal3.y());
		m_patch.plane_dir2[2] = static_cast<float>(normal3.z());

		// compute plane bounds
		m_patch.num_points = static_cast<uint8_t>(m_indexes.size());

		Eigen::Matrix4d world_space_to_patch_space;
		world_space_to_patch_space = Eigen::Matrix4d::Identity();
		world_space_to_patch_space(0, 0) = normal2(0);
		world_space_to_patch_space(0, 1) = normal2(1);
		world_space_to_patch_space(0, 2) = normal2(2);
		world_space_to_patch_space(0, 3) = 0.0;

		world_space_to_patch_space(1, 0) = normal3(0);
		world_space_to_patch_space(1, 1) = normal3(1);
		world_space_to_patch_space(1, 2) = normal3(2);
		world_space_to_patch_space(1, 3) = 0.0;

		world_space_to_patch_space(2, 0) = normal1(0);
		world_space_to_patch_space(2, 1) = normal1(1);
		world_space_to_patch_space(2, 2) = normal1(2);
		world_space_to_patch_space(2, 3) = 0.0;

		m_patch.origin[0] = m_centroid(0);
		m_patch.origin[1] = m_centroid(1);
		m_patch.origin[2] = m_centroid(2);


		float* points = new float[m_patch.num_points * 3];
		m_patch.points = new int8_t[m_patch.num_points * 3];
		
		Eigen::Vector4d new_point;

		double max, mix, may, miy, maz, miz;
		max = mix = may = miy = maz = miz = 0.0;
		for (uint8_t i = 0, j = 0; j < m_patch.num_points; i += 3, ++j) {

			new_point = world_space_to_patch_space * (m_root->m_points[m_indexes[j]] - m_centroid);
			points[i] = new_point.x();
			points[i + 1] = new_point.y();
			points[i + 2] = new_point.z();

			max = max > points[i] ? max : points[i];
			mix = mix < points[i] ? mix : points[i];

			may = may > points[i + 1] ? may : points[i + 1];
			miy = miy < points[i + 1] ? miy : points[i + 1];

			maz = maz > points[i + 2] ? maz : points[i + 2];
			miz = miz < points[i + 2] ? miz : points[i + 2];

		}
		// is this really correct?
		// max(x, eps) is due to possible division by zero number
		// all values that would have been inf are 127000 instead
		m_patch.quant_x = static_cast<float>(_settings->bits_reserved_axes / std::max(std::max(abs(max), abs(mix)), EPS));
		m_patch.quant_y = static_cast<float>(_settings->bits_reserved_axes / std::max(std::max(abs(may), abs(miy)), EPS));
		m_patch.quant_z = static_cast<float>(_settings->bits_reserved_axes / std::max(std::max(abs(maz), abs(miz)), EPS));

		//std::cout << "quant x: " << patch.quant_x << std::endl;
		//std::cout << "quant y: " << patch.quant_y << std::endl;
		//std::cout << "quant z: " << patch.quant_z << std::endl << std::endl;

		const unsigned int no_of_coords = m_patch.num_points * 3;
		for (unsigned int i = 0; i < no_of_coords; i += 3) {
			m_patch.points[i] = static_cast<int8_t>(points[i] * m_patch.quant_x);
			m_patch.points[i + 1] = static_cast<int8_t>(points[i + 1] * m_patch.quant_y);
			m_patch.points[i + 2] = static_cast<int8_t>(points[i + 2] * m_patch.quant_z);

			//std::cout << "quantified point x: " << patch.points[i] << std::endl;
			//std::cout << "quantified point y: " << patch.points[i + 1] << std::endl;
			//std::cout << "quantified point z: " << patch.points[i + 2] << std::endl << std::endl;

			//std::cout << "quantified point x: " << static_cast<int16_t>(patch.points[i]) << std::endl;
			//std::cout << "quantified point y: " << static_cast<int16_t>(patch.points[i + 1]) << std::endl;
			//std::cout << "quantified point z: " << static_cast<int16_t>(patch.points[i + 2]) << std::endl << std::endl;
		}

		const bool test_decoding = false;
		const bool visualize_decoded_point_cloud = true;
		if (test_decoding) {
			Eigen::Matrix4d patch_space_to_world_space = world_space_to_patch_space.inverse();
			Eigen::Vector4d decoded_point, difference_point;
			double accumulated_error = 0.0;
			for (unsigned int i = 0, j = 0; i < no_of_coords; i += 3, ++j) {
				decoded_point.x() = m_patch.points[i] / m_patch.quant_x;
				decoded_point.y() = m_patch.points[i+1] / m_patch.quant_y;
				decoded_point.z() = m_patch.points[i+2] / m_patch.quant_z;
				decoded_point.w() = 1.0;

				// comment in for hilarity
				//decoded_point = m_centroid + world_space_to_patch_space * decoded_point;
				decoded_point = m_centroid + patch_space_to_world_space * decoded_point;
				decoded_point.w() = 1.0;
				difference_point = m_root->m_points[m_indexes[j]] - decoded_point;
				difference_point.w() = 0.0;
				accumulated_error += std::abs(difference_point.norm());
				if (visualize_decoded_point_cloud) m_root->m_points[m_indexes[j]] = decoded_point;
			}
			const double average_error = accumulated_error / m_patch.num_points;
			//std::cout << "Number of points in node: " << static_cast<int16_t>(patch.num_points) << std::endl;
			//std::cout << "Accumulated error: " << accumulated_error << std::endl;
			//std::cout << "Average error: " << average_error << std::endl << std::endl;
		}


		delete[] points;

		// PROBLEM AREA
		//m_root->m_patches.push_back(new Patch(patch));








		// re-orient patch with ep1 = n�(1, 0, 0), ep2 = n�ep1, ep3 = n
		// As the coordinate system�s origin we choose the point cluster�s center of mass.
		// needs to be tested -
		if (_settings->reorient_patches) {
			Eigen::Vector3d x(1.0, 0.0, 0.0);
			Eigen::Vector3d n(normal1.x(), normal1.y(), normal1.z());
			Eigen::Vector3d ep1 = n.cross(x);
			Eigen::Vector3d ep2 = n.cross(ep1);
			Eigen::Vector3d ep3 = n;

			m_patch.origin[0] = static_cast<float>(m_centroid.x());
			m_patch.origin[1] = static_cast<float>(m_centroid.y());
			m_patch.origin[2] = static_cast<float>(m_centroid.z());
			m_patch.origin[3] = static_cast<float>(m_centroid.w());
		}

		// create height map

		// create occlusion map



		return;
	}

	m_children = new Octree[8];
	const double newsize = m_size * 0.5;
	for (int i = 0; i < 8; ++i) {
		m_children[i].m_size = newsize;
		m_children[i].m_level = m_level + 1;
		m_children[i].m_root = m_root;
		m_children[i].m_indexes.reserve(m_indexes.size() / 4);
	}

	const double size4 = m_size * 0.25;

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
		draw_wire_cube(m_middle, static_cast<float>(m_size));
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
	draw_wire_cube(m_middle, static_cast<float>(m_size));
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

void Octree::show_patch_planes(const float size) {
	if (m_is_leaf) draw_patch_plane(size);
	if (m_children != NULL)
	{
		for (short i = 0; i < 8; ++i)
		{
			m_children[i].show_patch_planes(size);
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
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*m_indexes.size() * 3, points, GL_STATIC_DRAW);
		GLint position_attribute = glGetAttribLocation(_settings->point_shader->ID, "aPos");
		glVertexAttribPointer(position_attribute, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (void*)0);
		glEnableVertexAttribArray(0);

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
	}

	glm::mat4 projection = glm::perspective(glm::radians(_settings->camera->Zoom), _settings->ASPECT_RATIO, _settings->Z_NEAR, _settings->Z_FAR);
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

	glm::mat4 projection = glm::perspective(glm::radians(_settings->camera->Zoom), _settings->ASPECT_RATIO, _settings->Z_NEAR, _settings->Z_FAR);
	glm::mat4 view = _settings->camera->GetViewMatrix();
	glm::vec3 translate(static_cast<float>(middle.x()), static_cast<float>(middle.y()), static_cast<float>(middle.z()));
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

void Octree::draw_patch_plane(const float size) {
	// initialize (if necessary)
	if (planeVAO == 0)
	{
		_settings->patch_planes_shader->use();
		_settings->patch_planes_shader->setVec3("color", 0.0f, 0.9f, 0.0f);
		_settings->patch_planes_shader->setFloat("point_size", _settings->point_size);
		_settings->patch_planes_shader->setFloat("z_near", _settings->Z_NEAR);
		_settings->patch_planes_shader->setFloat("z_far", _settings->Z_FAR);
		_settings->patch_planes_shader->setFloat("height_of_near_plane", _settings->height_of_near_plane);
		float vertices[] = {
			// triangle 1
			static_cast<float>(m_centroid.x() + normal2.x()), static_cast<float>(m_centroid.y() + normal2.y()), static_cast<float>(m_centroid.z() + normal2.z()),
			static_cast<float>(m_centroid.x() - normal2.x()), static_cast<float>(m_centroid.y() - normal2.y()), static_cast<float>(m_centroid.z() - normal2.z()),
			static_cast<float>(m_centroid.x() + normal3.x()), static_cast<float>(m_centroid.y() + normal3.y()), static_cast<float>(m_centroid.z() + normal3.z()),

			// triangle 2
			static_cast<float>(m_centroid.x() + normal2.x()), static_cast<float>(m_centroid.y() + normal2.y()), static_cast<float>(m_centroid.z() + normal2.z()),
			static_cast<float>(m_centroid.x() - normal2.x()), static_cast<float>(m_centroid.y() - normal2.y()), static_cast<float>(m_centroid.z() - normal2.z()),
			static_cast<float>(m_centroid.x() - normal3.x()), static_cast<float>(m_centroid.y() - normal3.y()), static_cast<float>(m_centroid.z() - normal3.z())
		};

		glGenVertexArrays(1, &planeVAO);
		glGenBuffers(1, &planeVBO);
		// fill buffer
		glBindBuffer(GL_ARRAY_BUFFER, planeVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
		// link vertex attributes
		glBindVertexArray(planeVAO);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
	}

	glm::mat4 projection = glm::perspective(glm::radians(_settings->camera->Zoom), _settings->ASPECT_RATIO, _settings->Z_NEAR, _settings->Z_FAR);
	glm::mat4 view = _settings->camera->GetViewMatrix();
	glm::vec3 translate(static_cast<float>(m_centroid.x()), static_cast<float>(m_centroid.y()), static_cast<float>(m_centroid.z()));
	glm::mat4 model = glm::translate(glm::mat4(1.0f), translate);
	model = glm::scale(model, glm::vec3(size, size, size));
	glm::mat4 MVPmatrix = projection * view * model;

	_settings->patch_planes_shader->use();
	_settings->patch_planes_shader->setMat4("mvp_matrix", MVPmatrix);
	_settings->patch_planes_shader->setVec3("cam_pos", _settings->camera->Position);
	_settings->patch_planes_shader->setFloat("point_size", _settings->point_size);
	//_settings->cube_shader->setVec3("color", float(std::rand())/RAND_MAX, float(std::rand()) / RAND_MAX, float(std::rand()) / RAND_MAX);

	// render Cube
	glBindVertexArray(planeVAO);
	glDrawArrays(GL_TRIANGLES, 0, 6);
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
			static_cast<float>(m_centroid.x()), static_cast<float>(m_centroid.y()), static_cast<float>(m_centroid.z()), // bottom-left
			static_cast<float>(m_centroid.x() + normal1.x()), static_cast<float>(m_centroid.y() + normal1.y()), static_cast<float>(m_centroid.z() + normal1.z()) // bottom-right     
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

	glm::mat4 projection = glm::perspective(glm::radians(_settings->camera->Zoom), _settings->ASPECT_RATIO, _settings->Z_NEAR, _settings->Z_FAR);
	glm::mat4 view = _settings->camera->GetViewMatrix();
	glm::vec3 translate(static_cast<float>(m_centroid.x()), static_cast<float>(m_centroid.y()), static_cast<float>(m_centroid.z()));
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
