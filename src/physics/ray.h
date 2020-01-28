/**
 * @file ray.h
 * @author Gabriel Maneru, gabriel.m, gabriel.m@digipen.edu
 * @date 01/28/2020
 * @brief Ray basic structure for raycasting
 * @copyright Copyright (C) 2020 DigiPen Institute of Technology.
**/
#pragma once
#include <glm/glm.hpp>

struct ray
{
	glm::vec3 m_start;
	glm::vec3 m_direction;

	glm::vec3 get_point(float t)const;
	float ray_cast_plane(glm::vec4 plane)const;
};