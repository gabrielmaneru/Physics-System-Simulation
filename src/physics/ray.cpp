/**
 * @file ray.cpp
 * @author Gabriel Maneru, gabriel.m, gabriel.m@digipen.edu
 * @date 01/28/2020
 * @brief Ray basic structure for raycasting
 * @copyright Copyright (C) 2020 DigiPen Institute of Technology.
**/
#include "ray.h"
#include "math_utils.h"

/**
 * Evaluates the ray at the given time
**/
glm::vec3 ray::get_point(float time) const
{
	return m_start + m_direction * time;
}


/**
 * Perform the ray intersection against a plane
**/
float ray::ray_cast_plane(glm::vec4 plane) const
{
	if (glm::length2(m_direction) > c_epsilon)
	{
		glm::vec3 normal{ plane };
		float dot_ray_normal = glm::dot(m_direction, normal);
		if (std::abs(dot_ray_normal) > c_epsilon)
		{
			glm::vec3 plane_point = normal * plane.w;
			glm::vec3 delta_pos = plane_point - m_start;
			return glm::dot(delta_pos, normal) / dot_ray_normal;
		}
	}
	return -1.0f;
}
