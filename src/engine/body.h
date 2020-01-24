#pragma once
#include <glm/glm.hpp>

struct body
{
	glm::mat4 get_model()const;

	glm::vec3 m_position;
	glm::vec3 m_l_velocity;
	glm::quat m_rotation;
	glm::quat m_a_velocity;
	glm::mat3 m_inertia;
};