/**
 * @file camera.cpp
 * @author Gabriel Maneru, gabriel.m, gabriel.m@digipen.edu
 * @date 01/28/2020
 * @brief Camera structure
 * @copyright Copyright (C) 2020 DigiPen Institute of Technology.
**/
#include "camera.h"
#include "window.h"
#include "input.h"
#include <glm/glm.hpp>
#include <GLFW/glfw3.h>

void camera::update()
{
	// Check Mouse input
	if (input.m_mouse_pressed[1])
	{
		// Compute current camera speed
		float cam_speed = m_linear_speed * (input.is_key_down(GLFW_KEY_LEFT_SHIFT) ? 10.0f : 1.0f);
		// Check WASDQE input keys
		if (input.is_key_down(GLFW_KEY_W))
			m_eye += m_front * cam_speed;
		if (input.is_key_down(GLFW_KEY_S))
			m_eye -= m_front * cam_speed;
		if (input.is_key_down(GLFW_KEY_D))
			m_eye += m_right * cam_speed;
		if (input.is_key_down(GLFW_KEY_A))
			m_eye -= m_right * cam_speed;
		if (input.is_key_down(GLFW_KEY_Q))
			m_eye += m_up * cam_speed;
		if (input.is_key_down(GLFW_KEY_E))
			m_eye -= m_up * cam_speed;
		// Add mouse step
		m_yaw += static_cast<float>(input.m_mouse_offset[0]) * m_angular_speed;
		m_pitch += static_cast<float>(input.m_mouse_offset[1]) * m_angular_speed;
		// Check Altitude Bounds
		if (m_pitch > 89.0f)
			m_pitch = 89.0f;
		if (m_pitch < -89.0f)
			m_pitch = -89.0f;
	}
	// Recompute front vector
	glm::vec3 front;
	front.x = cos(glm::radians(m_yaw)) * cos(glm::radians(m_pitch));
	front.y = sin(glm::radians(m_pitch));
	front.z = sin(glm::radians(m_yaw)) * cos(glm::radians(m_pitch));
	m_front = glm::normalize(front);
	// Recompute right and up vectors
	m_right = glm::normalize(glm::cross(m_front, glm::vec3{0,1,0}));
	m_up = glm::normalize(glm::cross(m_right, m_front));
	// Update view matrix
	m_view = glm::lookAt(m_eye, m_eye + m_front, m_up);
	// Compute aspect ratio
	float aspect = static_cast<float>(window.m_width) / static_cast<float>(window.m_height);
	// Update projection matrix
	m_proj = glm::perspective(glm::radians(m_fov), aspect, m_near, m_far);
}
