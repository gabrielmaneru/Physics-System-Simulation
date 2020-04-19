/**
 * @file raw_mesh.cpp
 * @author Gabriel Maneru, gabriel.m, gabriel.m@digipen.edu
 * @date 01/28/2020
 * @brief Raw Mesh structure
 * @copyright Copyright (C) 2020 DigiPen Institute of Technology.
**/
#include "raw_mesh.h"
#include <fstream>

/**
 * Local functions for parsing the files
**/
void parse_vec3(std::string line, glm::vec3& v)
{
	size_t s = 0;
	while (line[s] == 'v' || line[s] == ' '){s++;}
	line = line.substr(s);

	s = line.find(' ');
	v[0] = static_cast<float>(std::atof(line.substr(0, s).c_str()));
	while (line[s] == ' ') { s++; }
	line = line.substr(s);

	s = line.find(' ');
	v[1] = static_cast<float>(std::atof(line.substr(0, s).c_str()));
	while (line[s] == ' ') { s++; }
	line = line.substr(s);

	v[2] = static_cast<float>(std::atof(line.c_str()));
}
void parse_indices(std::string line, std::vector<uint>& f)
{
	size_t s = 0;
	while (line[s] == 'f' || line[s] == ' ') { s++; }
	line = line.substr(s);

	while (line.size() > 0)
	{
		size_t b = line.find('/');
		s = line.find(' ');
		if (s > line.length()) s = line.length();
		if (b > s) b = s;

		int i = std::atoi(line.substr(0, b).c_str()) - 1;
		f.push_back(static_cast<uint>(i));
		while (line[s] == ' ') { s++; }
		line = line.substr(s);
	}
}

/**
 * Constructor from file
**/
raw_mesh::raw_mesh(const std::string & path)
{
	std::ifstream file;
	file.open(path);
	assert(file.is_open());
	std::string line;
	while (std::getline(file, line))
	{
		if (line.size() == 0U)
			continue;
		if (line.rfind("v ") == 0)
		{
			m_vertices.push_back({});
			parse_vec3(line, m_vertices.back());
		}
		else if (line.rfind("f ") == 0)
		{
			std::vector<uint> f;
			parse_indices(line, f);
			m_triangles.push_back(f);
		}
	}
	file.close();

	post_process_mesh();
}
void raw_mesh::post_process_mesh()
{
	glm::vec3 min{ m_vertices[0] };
	glm::vec3 max{ m_vertices[0] };

	for (uint i = 1; i < m_vertices.size(); ++i)
		for (uint j = 0; j < 3; ++j)
		{
			float f = m_vertices[i][j];
			if (f < min[j])
				min[j] = f;
			else if (f > max[j])
				max[j] = f;
		}
	fix_scale(max - min);
	const glm::vec3 cm = compute_inertia();
	fix_cm(cm);
}
void raw_mesh::fix_scale(const glm::vec3& scale)
{
	float max_scl = glm::max(scale.x, scale.z);
	for (auto& v : m_vertices)
		v /= max_scl;
}
glm::vec3 raw_mesh::compute_inertia()
{
	float I0 = 0.0f;
	glm::vec3 Iw{ 0.0f };
	glm::vec3 Iw2{ 0.0f };
	float Ixy = 0.0f;
	float Iyz = 0.0f;
	float Izx = 0.0f;

	for (uint f =0; f < m_triangles.size(); ++f)
	{
		const std::vector<uint>& face{ m_triangles[f] };
		const glm::vec3& p0{ m_vertices[face[0]] };
		for (uint t = 1; t < face.size() - 1; ++t)
		{
			const glm::vec3& p1{ m_vertices[face[t]] };
			const glm::vec3& p2{ m_vertices[face[t + 1]] };

			const glm::vec3 d{ glm::cross(p1 - p0,p2 - p0) };
			const glm::vec3& w0{ p0 };
			const glm::vec3& w1{ p1 };
			const glm::vec3& w2{ p2 };

			const glm::vec3 w0aw1 = w0 + w1;
			const glm::vec3 w0mw0 = w0 * w0;
			const glm::vec3 w0mw0_a_w1_m_w0aw1 = w0mw0 + w1 * w0aw1;

			const glm::vec3 f1w = w0aw1 + w2;
			const glm::vec3 f2w = w0mw0_a_w1_m_w0aw1 + w2 * f1w;
			const glm::vec3 f3w = w0 * w0mw0 + w1 * w0mw0_a_w1_m_w0aw1 + w2 * f2w;

			const glm::vec3 g0w = f2w + w0 * (f1w + w0);
			const glm::vec3 g1w = f2w + w1 * (f1w + w1);
			const glm::vec3 g2w = f2w + w2 * (f1w + w2);


			I0  += d[0] * f1w[0];
			Iw  += d * f2w;
			Iw2 += d * f3w;

			Ixy += d[0] * (w0[1] * g0w[0] + w1[1] * g1w[0] + w2[1] * g2w[0]);
			Iyz += d[1] * (w0[2] * g0w[1] + w1[2] * g1w[1] + w2[2] * g2w[1]);
			Izx += d[2] * (w0[0] * g0w[2] + w1[0] * g1w[2] + w2[0] * g2w[2]);
		}
	}

	// Apply constant factors
	I0  /=  6.0f;
	Iw  /= 24.0f;
	Iw2 /= 60.0f;
	Ixy /= 120.0f;
	Iyz /= 120.0f;
	Izx /= 120.0f;

	if (I0 == 0.0f)
	{
		m_mass = 1.0f;
		m_inertia = glm::mat3(1.0f);
		return glm::vec3(0.0f);
	}

	// Store Mass & Center of Mass
	m_mass = I0;
	glm::vec3 cm = Iw / m_mass;


	// Compute inertia factor relative to the center of mass
	const float Ixx = Iw2[1] + Iw2[2] - m_mass * (cm.y*cm.y + cm.z + cm.z);
	const float Iyy = Iw2[2] + Iw2[0] - m_mass * (cm.z*cm.z + cm.x + cm.x);
	const float Izz = Iw2[0] + Iw2[1] - m_mass * (cm.x*cm.x + cm.y + cm.y);
	Ixy = -(Ixy - m_mass * cm.x*cm.y);
	Iyz = -(Iyz - m_mass * cm.y*cm.z);
	Izx = -(Izx - m_mass * cm.z*cm.x);

	// Compute the final Inertia Tensor
	m_inertia = {
		Ixx,Ixy,Izx,
		Ixy,Iyy,Iyz,
		Izx,Iyz,Izz
	};
	return cm;
}
void raw_mesh::fix_cm(const glm::vec3& cm)
{
	for (auto& v : m_vertices)
		v -= cm;
}
