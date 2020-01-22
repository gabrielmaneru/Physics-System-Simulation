#pragma once

struct GLFWwindow;
class c_drawer
{
	GLFWwindow* window{ nullptr };

public:
	bool initialize();
	bool should_exit();
	void update_window();
	void render_window();
	void shutdown();
	static c_drawer& get_instance();
};
#define drawer c_drawer::get_instance()