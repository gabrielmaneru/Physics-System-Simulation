#pragma once
#include <glm/glm.hpp>

struct vertex
{
	glm::vec3 m_position;
	glm::vec3 m_normal;

	vertex(glm::vec3 position, glm::vec3 normal = glm::vec3{0.0f})
		: m_position(position), m_normal(normal) {}
};