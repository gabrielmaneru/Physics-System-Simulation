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
	m_position += get_linear_velocity() * dt;

	// Add Torques to Angular Momentum
	m_angular_momentum += m_torques_accumulation;
	m_torques_accumulation = glm::vec3{ 0.0f };

	// Apply rotation
	glm::vec3 w = get_angular_velocity();
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
body & body::set_mass(float mass)
{
	m_inv_mass = 1.0f / mass;
	return *this;
}
float body::get_invmass() const
{
	if (m_is_static)
		return 0.0f;
	return m_inv_mass;
}
body & body::set_inertia(glm::mat3 i)
{
	m_inv_inertia = glm::inverse(i);
	return *this;
}
glm::mat3 body::get_local_invinertia() const
{
	if (m_is_static)
		return glm::mat3{ 0.0f };
	return m_inv_inertia;
}
glm::mat3 body::get_oriented_invinertia() const
{
	glm::mat3 R = glm::toMat3(m_rotation);
	return R * get_local_invinertia() * glm::transpose(R);
}
body & body::set_static(bool is_static)
{
	m_is_static = is_static;
	clear_momentum();
	return *this;
}
void body::clear_momentum()
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

glm::mat4 body::get_invmodel() const
{
	return glm::inverse(get_model());
}

glm::mat3 body::get_basis() const
{
	return glm::mat3_cast(m_rotation);
}

glm::vec3 body::get_linear_velocity() const
{
	return get_invmass() * m_linear_momentum;
}

glm::vec3 body::get_angular_velocity() const
{
	return get_oriented_invinertia() * m_angular_momentum;
}

glm::vec3 body::get_point_velocity(glm::vec3 point)
{
	return get_linear_velocity() + glm::cross(get_angular_velocity(), point - m_position);
}

