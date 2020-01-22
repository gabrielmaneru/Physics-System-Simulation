/* Start Header -------------------------------------------------------
Copyright (C) 2019 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written consent of
DigiPen Institute of Technology is prohibited.
File Name:	camera.cpp
Purpose: Camera Base
Author: Gabriel Mañeru - gabriel.m
- End Header --------------------------------------------------------*/
#include "camera.h"
#include "window.h"
#include "input.h"
#include <glm/glm.hpp>
#include <GLFW/glfw3.h>

void camera::update()
{
	if (input.m_mouse_pressed[1])
	{
		float speed = m_linear_speed * (input.is_key_down(GLFW_KEY_LEFT_SHIFT) ? 10.0f : 1.0f);

		if (input.is_key_down(GLFW_KEY_W))
			m_eye += m_front * speed;
		if (input.is_key_down(GLFW_KEY_S))
			m_eye -= m_front * speed;
		if (input.is_key_down(GLFW_KEY_D))
			m_eye += m_right * speed;
		if (input.is_key_down(GLFW_KEY_A))
			m_eye -= m_right * speed;
		if (input.is_key_down(GLFW_KEY_Q))
			m_eye += m_up * speed;
		if (input.is_key_down(GLFW_KEY_E))
			m_eye -= m_up * speed;

		// Apply Offset
		m_yaw += input.m_mouse_offset[0] * m_angular_speed;
		m_pitch += input.m_mouse_offset[1] * m_angular_speed;

		// Check Bounds
		if (m_pitch > 89.0f)
			m_pitch = 89.0f;
		if (m_pitch < -89.0f)
			m_pitch = -89.0f;
	}

	// Recompute Front
	glm::vec3 front;
	front.x = cos(glm::radians(m_yaw)) * cos(glm::radians(m_pitch));
	front.y = sin(glm::radians(m_pitch));
	front.z = sin(glm::radians(m_yaw)) * cos(glm::radians(m_pitch));
	m_front = glm::normalize(front);

	// Recompute Right and Up
	m_right = glm::normalize(glm::cross(m_front, glm::vec3{0,1,0}));
	m_up = glm::normalize(glm::cross(m_right, m_front));

	// Update View
	m_view = glm::lookAt(m_eye, m_eye + m_front, m_up);

	// Update Projection
	float aspect = static_cast<float>(window.m_width) / static_cast<float>(window.m_height);
	m_proj = glm::perspective(glm::radians(m_fov), aspect, m_near, m_far);
}
