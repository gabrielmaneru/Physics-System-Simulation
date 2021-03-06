/**
 * @file input.cpp
 * @author Gabriel Maneru, gabriel.m, gabriel.m@digipen.edu
 * @date 01/28/2020
 * @brief Input manager
 * @copyright Copyright (C) 2020 DigiPen Institute of Technology.
**/
#include "input.h"
#include <GLFW/glfw3.h>

/**
 * GLFW callbacks
**/
static void glfw_key_callback(GLFWwindow*, int key, int, int action, int)
{
	if (key > 0)
	{
		if (action == GLFW_PRESS)
			input.m_keyboard[key - 1] = 1;
		else if (action == GLFW_RELEASE)
			input.m_keyboard[key - 1] = 0;
		input.m_triggered_keys.insert(key);
	}
}
static void glfw_mouse_callback(GLFWwindow*, double xpos, double ypos)
{
	static bool firstMouse{ true };
	if (firstMouse)
		input.m_mouse_pos[0] = xpos,
		input.m_mouse_pos[1] = ypos,
		firstMouse = false;

	input.m_mouse_offset[0] = xpos - input.m_mouse_pos[0];
	input.m_mouse_offset[1] = input.m_mouse_pos[1] - ypos;
	input.m_mouse_pos[0] = xpos;
	input.m_mouse_pos[1] = ypos;
}
static void glfw_mouse_button_callback(GLFWwindow*, int button, int action, int)
{
	if (action == GLFW_PRESS)
		input.m_mouse_triggered[button] = input.m_mouse_pressed[button] = true;
	else if (action == GLFW_RELEASE)
		input.m_mouse_pressed[button] = false;
}

/**
 * Initialize the manager
**/
void s_input::setup_callback(GLFWwindow * w)const
{
	glfwSetKeyCallback(w, glfw_key_callback);
	glfwSetCursorPosCallback(w, glfw_mouse_callback);
	glfwSetMouseButtonCallback(w, glfw_mouse_button_callback);
}

/**
 * Clear trigger data
**/
void s_input::clear_triggers()
{
	m_triggered_keys.clear();
	input.m_mouse_offset[0] = 0.0;
	input.m_mouse_offset[1] = 0.0;
	input.m_mouse_triggered[0] = false;
	input.m_mouse_triggered[1] = false;
}

/**
 * Key checkers
**/
bool s_input::is_key_up(int key)const
{
	return m_keyboard[key - 1] == 0;
}
bool s_input::is_key_down(int key)const
{
	return m_keyboard[key - 1] == 1;
}
bool s_input::is_key_triggered(int key)const
{
	return is_key_down(key)
	&&  m_triggered_keys.find(key) != m_triggered_keys.cend();
}
bool s_input::is_key_released(int key)const
{
	return is_key_up(key)
	&& m_triggered_keys.find(key) != m_triggered_keys.cend();
}

/**
 * Singletone instanciation
**/
s_input & s_input::get_instance()
{
	static s_input instance;
	return instance;
}
