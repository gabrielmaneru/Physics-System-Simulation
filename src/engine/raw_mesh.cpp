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
	fix_centroid();
}

 void raw_mesh::fix_centroid()
 {
	 glm::vec3 centroid{ 0.0f };
	 for (auto v : m_vertices)
		 centroid += v;
	 centroid /= static_cast<float>(m_vertices.size());

	 for (auto& v : m_vertices)
		 v -= centroid;
 }
