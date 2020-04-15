/**
 * @file drawer.h
 * @author Gabriel Maneru, gabriel.m, gabriel.m@digipen.edu
 * @date 01/28/2020
 * @brief Renderer manager
 * @copyright Copyright (C) 2020 DigiPen Institute of Technology.
**/
#pragma once
#include "camera.h"
#include <vector>

struct debug_vertex
{
	glm::vec3 m_pos;
	glm::vec3 m_color;
	glm::vec3 m_normal{0.0f, 0.0f, 0.0f};
};
struct debug_mesh
{
	unsigned m_vao{ 0 };
	unsigned m_vbo{ 0 };
};

using uint = unsigned int;
class shader_program;
class c_drawer
{
	debug_mesh m_line;
	debug_mesh m_tri;
	std::vector<debug_vertex> m_debug_lines;
	std::vector<debug_vertex> m_debug_tri;
	shader_program* m_debug_shader{nullptr};

public:
	bool initialize();
	void render();
	void shutdown();
	void add_debugline(glm::vec3 p0, glm::vec3 p1, glm::vec3 color);
	void add_debugline_parallelepiped(glm::vec3 p0, glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, glm::vec3 color);
	void add_debugline_cube(glm::vec3 p, float size, glm::vec3 color);
	void add_debugline_list(const std::vector<glm::vec3>& pts, glm::vec3 color);
	void add_debugtri_list(const std::vector<glm::vec3>& pts, const std::vector<glm::vec3>& norm, glm::vec3 color);
	static c_drawer& get_instance();

	camera m_camera{};
};
#define drawer c_drawer::get_instance()

const glm::vec3 red{ 1.f, .0f, .0f };
const glm::vec3 yellow{ 1.f, 1.f, .0f };
const glm::vec3 green{ .0f, 1.f, .0f };
const glm::vec3 cyan{ .0f, 1.f, 1.f };
const glm::vec3 blue{ .0f, .0f, 1.f };
const glm::vec3 magenta{ 1.f, .0f, 1.f };
const glm::vec3 white{ 1.f, 1.f, 1.f };
const glm::vec3 black{ .0f, .0f, .0f };