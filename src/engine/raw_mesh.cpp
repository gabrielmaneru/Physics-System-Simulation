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
			m_faces.push_back(f);
		}
	}
	file.close();

	fix_mesh();
	compute_inertia();
}
void raw_mesh::fix_mesh()
{
	glm::vec3 min{ m_vertices[0] };
	glm::vec3 max{ m_vertices[0] };
	for(uint i = 1; i < m_vertices.size(); ++i)
		for (uint j = 0; j < 3; ++j)
		{
			float f = m_vertices[i][j];
			if (f < min[j])
				min[j] = f;
			else if (f > max[j])
				max[j] = f;
		}
	glm::vec3 c = (max + min) * .5f;
	min -= c;
	max -= c;
	glm::vec3 scl = max - min;
	float max_scl = glm::max(scl.x, scl.z);

	for (auto& v : m_vertices)
		v = (v - c) / max_scl;
}

const float f1(const glm::vec3& w)
{
	return w.x + w.y + w.z;
}
const float f2(const glm::vec3& w)
{
	const float w2x = w.x*w.x;
	const float w2y = w.y*w.y;
	return w2x + w.x*w.y + w2y + w.z*f1(w);
}
const float f3(const glm::vec3& w)
{
	const float w2x = w.x*w.x;
	const float w3x = w2x * w.x;
	const float w2y = w.y*w.y;
	const float w3y = w2y * w.y;
	return w3x + w2x * w.y + w.x*w2y + w3y + w.z*f2(w);
}
template <uint i> const float g(const glm::vec3& w)
{
	return f2(w) + w[i] * (f1(w) + w[i]);
}

void raw_mesh::compute_inertia()
{
	float integral_ = 0.0f;

	float integral_x = 0.0f;
	float integral_y = 0.0f;
	float integral_z = 0.0f;

	float integral_x2 = 0.0f;
	float integral_y2 = 0.0f;
	float integral_z2 = 0.0f;

	float Ixy = 0.0f;
	float Iyz = 0.0f;
	float Izx = 0.0f;

	for (uint f =0; f < m_faces.size(); ++f)
	{
		const std::vector<uint>& face{ m_faces[f] };
		const glm::vec3& p0{ m_vertices[face[0]] };
		for (uint t = 1; t < face.size() - 1; ++t)
		{
			const glm::vec3& p1{ m_vertices[face[t]] };
			const glm::vec3& p2{ m_vertices[face[t + 1]] };

			const glm::vec3 d{ glm::cross(p1 - p0,p2 - p0) };
			const glm::vec3 x{ p0.x,p1.x,p2.x };
			const glm::vec3 y{ p0.y,p1.y,p2.y };
			const glm::vec3 z{ p0.z,p1.z,p2.z };

			integral_   += d[0] /  6.0f * f1(x);

			integral_x  += d[0] / 12.0f * f2(x);
			integral_y  += d[1] / 12.0f * f2(y);
			integral_z  += d[2] / 12.0f * f2(z);

			integral_x2 += d[0] / 20.0f * f3(x);
			integral_y2 += d[1] / 20.0f * f3(y);
			integral_z2 += d[2] / 20.0f * f3(z);

			Ixy += d[0] / 60.0f * (y[0]*g<0>(x) + y[1]*g<1>(x) + y[2]*g<2>(x));
			Iyz += d[1] / 60.0f * (z[0]*g<0>(x) + z[1]*g<1>(y) + z[2]*g<2>(x));
			Izx += d[2] / 60.0f * (x[0]*g<0>(x) + x[1]*g<1>(x) + x[2]*g<2>(z));
		}
	}

	float Ixx = integral_y2 + integral_z2;
	float Iyy = integral_z2 + integral_x2;
	float Izz = integral_x2 + integral_y2;
	
	m_inertia = {
		 Ixx,-Ixy,-Izx,
		-Ixy, Iyy,-Iyz,
		-Izx,-Iyz, Izz
	};
}
