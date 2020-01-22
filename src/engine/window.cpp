#include "window.h"
#include "input.h"
#include <GLFW/glfw3.h>

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

bool c_window::should_exit()
{
	return glfwWindowShouldClose(m_window);
}

void c_window::update()
{
	input.clear_triggers();
	glfwPollEvents();
	glfwGetFramebufferSize(m_window, &m_width, &m_height);
}

void c_window::present()
{
	glfwSwapBuffers(m_window);
}

void c_window::shutdown()
{
	glfwDestroyWindow(m_window);
	glfwTerminate();
}

c_window & c_window::get_instance()
{
	static c_window instance;
	return instance;
}
