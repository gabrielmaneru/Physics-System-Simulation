/**
 * @file camera.h
 * @author Gabriel Maneru, gabriel.m, gabriel.m@digipen.edu
 * @date 01/28/2020
 * @brief Camera structure
 * @copyright Copyright (C) 2020 DigiPen Institute of Technology.
**/
#pragma once
#include <glm/glm.hpp>
struct camera
{
	camera() { update(); }
	void update();

	float m_fov{ 65.0f };
	float m_near{ .001f };
	float m_far{ 100.0f };
	float m_yaw{ -120.0f };
	float m_pitch{ -30.0f };

	glm::vec3 m_eye{2,2,-1};
	glm::vec3 m_front;
	glm::vec3 m_up;
	glm::vec3 m_right;

	glm::mat4 m_proj;
	glm::mat4 m_view;
	glm::mat4 get_vp()const { return m_proj * m_view; }

	float m_angular_speed{ 0.2f };
	float m_linear_speed{ 0.05f };
};