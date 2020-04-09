/**
 * @file window.h
 * @author Gabriel Maneru, gabriel.m, gabriel.m@digipen.edu
 * @date 01/28/2020
 * @brief Window manager
 * @copyright Copyright (C) 2020 DigiPen Institute of Technology.
**/
#pragma once
#include <GLFW/glfw3.h>
#include <unordered_set>
#include <glm/glm.hpp>

struct GLFWwindow;
class c_window
{
	GLFWwindow* m_window{ nullptr };

public:
	bool initialize();
	bool should_exit();
	void update();
	void present();
	void shutdown();
	glm::vec2 get_mouse_ndc();
	GLFWwindow* get_handler();
	static c_window& get_instance();

	double m_dt{ 0.0f };
	int m_width{1920};
	int m_height{1080};
};
#define window c_window::get_instance()
