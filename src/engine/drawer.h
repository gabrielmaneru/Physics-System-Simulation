#pragma once
#include "camera.h"
#include <vector>

struct debug_vertex
{
	glm::vec3 m_pos;
	glm::vec3 m_color;
};

using uint = unsigned int;
class Shader_Program;
class c_drawer
{
	uint m_debug_vao{0};
	uint m_debug_vbo{0};
	uint m_debug_size{0};
	std::vector<debug_vertex> m_debug_lines;
	Shader_Program* m_debug_shader{nullptr};

public:
	bool initialize();
	void render();
	void shutdown();
	void add_debug_line(glm::vec3 p0, glm::vec3 p1, glm::vec3 color);
	void add_debug_cube(glm::vec3 p, float size, glm::vec3 color);
	static c_drawer& get_instance();

	camera m_camera{};
};
#define drawer c_drawer::get_instance()