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
	if (m_freeze)
	{
		m_forces_accumulation = glm::vec3{ 0.0f };
		m_torques_accumulation = glm::vec3{ 0.0f };
		return;
	}

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

	// Apply rotation
	glm::mat3 R = glm::toMat3(m_rotation);
	glm::mat3 w_inv_inertia = R * m_inv_inertia * glm::transpose(R);
	glm::vec3 w = w_inv_inertia * m_angular_momentum;
	glm::quat w_quat{ 0.0f, w.x, w.y, w.z };
	m_rotation = glm::normalize(m_rotation + .5f * w_quat * m_rotation * dt);
}
void body::add_force(glm::vec3 force, glm::vec3 point)
{
	glm::vec3 R = point - m_position;

	m_forces_accumulation += force;
	m_torques_accumulation += glm::cross(R, force);
}
body & body::set_position(glm::vec3 pos)
{
	m_position = pos;
	return *this;
}
body & body::set_rotation(glm::quat rot)
{
	m_rotation = rot;
	return *this;
}
body & body::set_inertia(glm::mat3 i)
{
	m_inv_inertia = glm::inverse(i);
	return *this;
}
body & body::set_freeze(bool freeze)
{
	m_freeze = freeze;
	return *this;
}
void body::stop()
{
	m_linear_momentum = glm::vec3{};
	m_angular_momentum = glm::vec3{};

	m_forces_accumulation = glm::vec3{};
	m_angular_momentum = glm::vec3{};
}
glm::mat4 body::get_model()const
{
	return glm::translate(glm::mat4(1.0f), m_position) * glm::mat4_cast(m_rotation);
}

