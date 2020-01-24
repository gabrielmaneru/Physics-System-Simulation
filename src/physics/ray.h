#pragma once
#include <glm/glm.hpp>

struct ray
{
	glm::vec3 m_start;
	glm::vec3 m_direction;

	glm::vec3 get_point(float t)const;
	float ray_cast_plane(glm::vec4 plane)const;
};