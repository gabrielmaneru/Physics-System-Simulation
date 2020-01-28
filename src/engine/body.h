#pragma once
#include <glm/glm.hpp>

glm::vec3 tr_point(glm::mat4 m, glm::vec3 v);
glm::vec3 tr_vector(glm::mat4 m, glm::vec3 v);

struct body
{
	void integrate(float dt);
	body& set_inertia(glm::mat3 i);
	void add_force(glm::vec3 force, glm::vec3 point);
	glm::mat4 get_model()const;

	glm::vec3 m_position;
	glm::quat m_rotation;

	glm::vec3 m_linear_momentum{ 0.0f };
	glm::vec3 m_angular_momentum{ 0.0f };

	glm::vec3 m_forces_accumulation{ 0.0f };
	glm::vec3 m_torques_accumulation{ 0.0f };

	float     m_mass{ 1.0f };
	glm::mat3 m_inv_inertia{ 1.0f };
	glm::vec3 m_mass_center{ 0.0f };
};