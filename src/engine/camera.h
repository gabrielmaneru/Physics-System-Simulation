/* Start Header -------------------------------------------------------
Copyright (C) 2019 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written consent of
DigiPen Institute of Technology is prohibited.
File Name:	camera.h
Purpose: Camera Base
Author: Gabriel Mañeru - gabriel.m
- End Header --------------------------------------------------------*/

#pragma once
#include <glm/glm.hpp>
struct camera
{
	camera() { update(); }
	void update();

	float m_fov{ 65.0f };
	float m_near{ .1f };
	float m_far{ 500.0f };
	float m_yaw{ -90.0f };
	float m_pitch{ 0.0f };

	glm::vec3 m_eye{0,0,1};
	glm::vec3 m_front;
	glm::vec3 m_up;
	glm::vec3 m_right;

	glm::mat4 m_proj;
	glm::mat4 m_view;

	float m_angular_speed{ 0.2f };
	float m_linear_speed{ 0.05f };
};