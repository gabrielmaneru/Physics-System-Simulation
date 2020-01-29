/**
 * @file body.cpp
 * @author Gabriel Maneru, gabriel.m, gabriel.m@digipen.edu
 * @date 01/28/2020
 * @brief Camera structure
 * @copyright Copyright (C) 2020 DigiPen Institute of Technology.
**/
#include "body.h"

void body::integrate(float dt)
{
	// Add Forces to Linear Momentum
	m_linear_momentum += m_forces_accumulation;
	m_forces_accumulation = glm::vec3{ 0.0f };

	// Apply velocity
	if (m_mass != 0.0f)
	{
		glm::vec3 v = m_linear_momentum / m_mass;
		m_position += v * dt;
	}

	// Add Torques to Angular Momentum
	m_angular_momentum += m_torques_accumulation;
	m_torques_accumulation = glm::vec3{ 0.0f };

	// Transform inertia
	glm::mat3 R = glm::toMat3(m_rotation);
	glm::mat3 w_inv_inertia = R * m_inv_inertia * glm::transpose(R);

	// Apply rotation
	glm::vec3 w = w_inv_inertia * m_angular_momentum;
	glm::quat w_quat{ 0.0f, w.x, w.y, w.z };
	m_rotation = glm::normalize(m_rotation + .5f * w_quat * m_rotation * dt);
}
body & body::set_position(glm::vec3 pos)
{
	m_position = pos;
	return *this;
}
body & body::set_inertia(glm::mat3 i)
{
	m_inv_inertia = glm::inverse(i);
	return *this;
}
void body::add_force(glm::vec3 force, glm::vec3 point)
{
	glm::vec3 R = point - m_position;

	m_forces_accumulation += force;
	m_torques_accumulation += glm::cross(R, force);
}
glm::mat4 body::get_model()const
{
	return glm::translate(glm::mat4(1.0f), m_position) * glm::mat4_cast(m_rotation);
}

glm::vec3 tr_point(glm::mat4 m, glm::vec3 v)
{
	return glm::vec3(m*glm::vec4(v, 1.0f));
}
glm::vec3 tr_vector(glm::mat4 m, glm::vec3 v)
{
	return glm::vec3(m*glm::vec4(v, 0.0f));
}
