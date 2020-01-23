#pragma once
#include <vector>
#include <string>
#include <glm/glm.hpp>

using uint = unsigned int;
struct raw_mesh
{
	raw_mesh() = default;
	raw_mesh(const std::string& path);

	std::vector<glm::vec3> m_vertices;
	std::vector< std::vector<uint> > m_faces;
};