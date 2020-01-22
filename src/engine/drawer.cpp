#include "drawer.h"
#include <glad/glad.h>
#include <GLFW/glfw3.h>

bool c_drawer::initialize()
{
	// Initialize GLFW
	if (!glfwInit()) return false;
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
	glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
	glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
	window = glfwCreateWindow(1280, 720, "CS550", nullptr, nullptr);
	if (!window) return false;
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	// Initialize GLAD
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) return false;
	if (!GLAD_GL_VERSION_3_3) return false;
	return true;
}

bool c_drawer::should_exit()
{
	return glfwWindowShouldClose(window);
}

void c_drawer::update_window()
{
	glfwPollEvents();
	int display_w, display_h;
	glfwMakeContextCurrent(window);
	glfwGetFramebufferSize(window, &display_w, &display_h);
}

void c_drawer::render_window()
{
	glClearColor(1, 0, 1, 1);
	glClear(GL_COLOR_BUFFER_BIT);

	//RENDER

	glfwMakeContextCurrent(window);
	glfwSwapBuffers(window);
}

void c_drawer::shutdown()
{
	glfwDestroyWindow(window);
	glfwTerminate();
}

c_drawer & c_drawer::get_instance()
{
	static c_drawer instance;
	return instance;
}
