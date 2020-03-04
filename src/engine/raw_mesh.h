/**
 * @file raw_mesh.h
 * @author Gabriel Maneru, gabriel.m, gabriel.m@digipen.edu
 * @date 01/28/2020
 * @brief Raw Mesh structure
 * @copyright Copyright (C) 2020 DigiPen Institute of Technology.
**/
#pragma once
#include <vector>
#include <string>
#include <glm/glm.hpp>

using uint = unsigned int;
struct raw_mesh
{
	raw_mesh() = default;
	raw_mesh(const std::string& path);
	void post_process_mesh();
	void fix_scale(const glm::vec3& scale);
	glm::vec3 compute_inertia();
	void fix_cm(const glm::vec3&);

	
	glm::mat3 m_inertia;
	float m_mass;
	std::vector<glm::vec3> m_vertices;
	std::vector< std::vector<uint> > m_faces;
};