/**
 * @file face.h
 * @author Gabriel Maneru, gabriel.m, gabriel.m@digipen.edu
 * @date 01/28/2020
 * @brief Face structure
 * @copyright Copyright (C) 2020 DigiPen Institute of Technology.
**/
#pragma once
#include <vector>
#include <glm/glm.hpp>

struct half_edge;
using uint = unsigned int;
struct physical_mesh;
struct face
{
	face(const physical_mesh*);
	void refresh();

	const physical_mesh* m_owner{ nullptr };
	half_edge* m_hedge_start;
	std::vector<uint> m_indices;
	glm::vec4 m_plane;
	float m_distance{-1.0f};
	glm::vec3 m_center;
};