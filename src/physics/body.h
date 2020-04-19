/**
 * @file body.h
 * @author Gabriel Maneru, gabriel.m, gabriel.m@digipen.edu
 * @date 01/28/2020
 * @brief Camera structure
 * @copyright Copyright (C) 2020 DigiPen Institute of Technology.
**/
#pragma once
#include <glm/glm.hpp>

struct body
{
	void add_impulse(glm::vec3 impulse, glm::vec3 point);
	void add_impulse_angular(glm::vec3 impulse);
	void add_impulse_linear(glm::vec3 impulse);
	void integrate_velocities(const float dt, const glm::vec3& gravity);
	void integrate_positions(const float dt);
	body& set_position(glm::vec3 pos);
	body& set_rotation(glm::quat rot);
	body& set_mass(float mass);
	float get_mass()const;
	float get_invmass()const;
	body& set_inertia(glm::mat3 i);
	glm::mat3 get_local_invinertia()const;
	glm::mat3 get_oriented_invinertia()const;
	body& set_static(bool is_static);
	body& set_friction(float f);
	body& set_roll(float r);
	body& set_restitution(float r);
	void clear_momentum();

	glm::mat4 get_model()const;
	glm::mat4 get_invmodel()const;
	glm::mat3 get_basis()const;
	glm::vec3 get_linear_velocity()const;
	glm::vec3 get_angular_velocity()const;
	glm::vec3 get_velocity_at_point(glm::vec3 point)const;

	glm::vec3 m_position{0.0f, 0.0f, 0.0f};
	glm::quat m_rotation{1.0f, 0.0f, 0.0f, 0.0f};
	glm::vec3 m_linear_momentum{ 0.0f };
	glm::vec3 m_angular_momentum{ 0.0f };
	bool	  m_is_static{ false };
	float     m_inv_mass{ 1.0f };
	glm::mat3 m_inv_inertia{ 1.0f };
	float m_linear_damping{ 0.05f };
	float m_angular_damping{ 0.05f };
	float m_friction_coef{ 0.00f };
	float m_roll_coef{ 0.00f };
	float m_restitution_coef{ 0.2f };
};

extern float physics_dt;