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
	static c_window& get_instance();

	int m_width{1920};
	int m_height{1080};
};
#define window c_window::get_instance()
