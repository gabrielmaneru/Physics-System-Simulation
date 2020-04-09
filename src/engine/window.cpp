/**
 * @file window.cpp
 * @author Gabriel Maneru, gabriel.m, gabriel.m@digipen.edu
 * @date 01/28/2020
 * @brief Window manager
 * @copyright Copyright (C) 2020 DigiPen Institute of Technology.
**/
#include "window.h"
#include "input.h"
#include <GLFW/glfw3.h>

/**
 * Initialize the manager
**/
bool c_window::initialize()
{
	// Initialize GLFW
	if (!glfwInit()) return false;
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
	glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
	glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
	m_window = glfwCreateWindow(m_width, m_height, "Physics System", nullptr, nullptr);
	if (!m_window) return false;
	glfwMakeContextCurrent(m_window);
	glfwSwapInterval(1);
	input.setup_callback(m_window);
	return true;
}

/**
 * Check is the window has been closed
**/
bool c_window::should_exit()
{
	return glfwWindowShouldClose(m_window);
}

/**
 * Update the manager
**/
void c_window::update()
{
	static double time{ glfwGetTime()-1.0f/60.0f };
	const double currentTime{ glfwGetTime() };
	m_dt = currentTime - time;
	time = currentTime;

	input.clear_triggers();
	glfwPollEvents();
	glfwGetFramebufferSize(m_window, &m_width, &m_height);
	m_width = glm::max(m_width, 1);
	m_height = glm::max(m_height, 1);
}

/**
 * Swap buffers
**/
void c_window::present()
{
	glfwSwapBuffers(m_window);
}

/**
 * Shutdown the manager
**/
void c_window::shutdown()
{
	glfwDestroyWindow(m_window);
	glfwTerminate();
}

/**
 * Get Mouse position in NDC coordinates
**/
glm::vec2 c_window::get_mouse_ndc()
{
	double xfactor = input.m_mouse_pos[0] / static_cast<double>(m_width);
	double yfactor = 1.0 - (input.m_mouse_pos[1] / static_cast<double>(m_height));
	return glm::vec2(static_cast<float>(xfactor),static_cast<float>(yfactor))*2.0f-1.0f;
}

/**
 * Get the window handler (ImGui handle)
**/
GLFWwindow * c_window::get_handler()
{
	return m_window;
}

/**
 * Singletone instanciator
**/
c_window & c_window::get_instance()
{
	static c_window instance;
	return instance;
}
