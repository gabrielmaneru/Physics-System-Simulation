/**
 * @file body.h
 * @author Gabriel Maneru, gabriel.m, gabriel.m@digipen.edu
 * @date 01/28/2020
 * @brief Camera structure
 * @copyright Copyright (C) 2020 DigiPen Institute of Technology.
**/
#pragma once
#include <glm/glm.hpp>

glm::vec3 tr_point(glm::mat4 m, glm::vec3 v);
glm::vec3 tr_vector(glm::mat4 m, glm::vec3 v);

struct body
{
	body& set_position(glm::vec3 pos);
	body& set_inertia(glm::mat3 i);

	void integrate(float dt);
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
};