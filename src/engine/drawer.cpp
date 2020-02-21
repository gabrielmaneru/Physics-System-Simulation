/**
 * @file drawer.cpp
 * @author Gabriel Maneru, gabriel.m, gabriel.m@digipen.edu
 * @date 01/28/2020
 * @brief Renderer manager
 * @copyright Copyright (C) 2020 DigiPen Institute of Technology.
**/
#include "drawer.h"
#include "shader_program.h"
#include <Windows.h>
extern "C" { // Enable High performance GPU
	_declspec(dllexport) DWORD NvOptimusEnablement = 0x00000001; }
//#undef APIENTRY
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>


bool c_drawer::initialize()
{
	// Initialize GLAD
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) return false;
	if (!GLAD_GL_VERSION_3_3) return false;

	// Create shaders
	try {
		m_debug_shader = new shader_program("debug.vert", "debug.frag");
	}
	catch (const std::string & log) { std::cout << log; }
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_BLEND);
	glEnable(GL_CULL_FACE); //glEnable(GL_DEPTH_TEST);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	return true;
}

void c_drawer::render()
{
	m_camera.update();
	glClearColor(0.1f, 0.1f, 0.1f, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glm::mat4 vp = m_camera.get_vp();

	if (!m_debug_tri.empty())
	{
		if (m_tri.m_vao == 0)
			glGenVertexArrays(1, &m_tri.m_vao);
		glBindVertexArray(m_tri.m_vao);

		if (m_tri.m_vbo == 0)
			glGenBuffers(1, &m_tri.m_vbo);
		glBindBuffer(GL_ARRAY_BUFFER, m_tri.m_vbo);
		glBufferData(GL_ARRAY_BUFFER, sizeof(debug_vertex)*m_debug_tri.size(), m_debug_tri.data(), GL_DYNAMIC_DRAW);

		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
		glEnableVertexAttribArray(1);

		m_debug_shader->use();
		m_debug_shader->set_uniform("vp", vp);
		m_debug_shader->set_uniform("alpha", 0.5f);
		glDrawArrays(GL_TRIANGLES, 0, (GLsizei)m_debug_tri.size());

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
		m_debug_tri.clear();
	}
	if (!m_debug_lines.empty())
	{
		if (m_line.m_vao == 0)
			glGenVertexArrays(1, &m_line.m_vao);
		glBindVertexArray(m_line.m_vao);

		if (m_line.m_vbo == 0)
			glGenBuffers(1, &m_line.m_vbo);
		glBindBuffer(GL_ARRAY_BUFFER, m_line.m_vbo);
		glBufferData(GL_ARRAY_BUFFER, sizeof(debug_vertex)*m_debug_lines.size(), m_debug_lines.data(), GL_DYNAMIC_DRAW);

		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
		glEnableVertexAttribArray(1);

		m_debug_shader->use();
		m_debug_shader->set_uniform("vp", vp);
		m_debug_shader->set_uniform("alpha", 1.0f);
		glDrawArrays(GL_LINES, 0, (GLsizei)m_debug_lines.size());

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
		m_debug_lines.clear();
	}

}

void c_drawer::shutdown()
{
}

void c_drawer::add_debugline(glm::vec3 p0, glm::vec3 p1, glm::vec3 color)
{
	m_debug_lines.push_back({ p0,color });
	m_debug_lines.push_back({ p1,color });
}

void c_drawer::add_debugline_parallelepiped(glm::vec3 p0, glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, glm::vec3 color)
{
	glm::vec3 h0 = v0 * 0.5f;
	glm::vec3 h1 = v1 * 0.5f;
	glm::vec3 h2 = v2 * 0.5f;

	glm::vec3 p_min = p0 - h0 - h1 - h2;
	glm::vec3 p_max = p0 + h0 + h1 + h2;

	add_debugline(p_min, p0 + h0 - h1 - h2, color);
	add_debugline(p_min, p0 - h0 + h1 - h2, color);
	add_debugline(p_min, p0 - h0 - h1 + h2, color);

	add_debugline(p0 + h0 - h1 - h2, p0 + h0 - h1 + h2, color);
	add_debugline(p0 - h0 - h1 + h2, p0 + h0 - h1 + h2, color);
	add_debugline(p0 - h0 - h1 + h2, p0 - h0 + h1 + h2, color);
	add_debugline(p0 - h0 + h1 - h2, p0 - h0 + h1 + h2, color);
	add_debugline(p0 - h0 + h1 - h2, p0 + h0 + h1 - h2, color);
	add_debugline(p0 + h0 - h1 - h2, p0 + h0 + h1 - h2, color);

	add_debugline(p0 - h0 + h1 + h2, p_max, color);
	add_debugline(p0 + h0 - h1 + h2, p_max, color);
	add_debugline(p0 + h0 + h1 - h2, p_max, color);
}

void c_drawer::add_debugline_cube(glm::vec3 p, float size, glm::vec3 color)
{
	float hs = size * 0.5f;
	add_debugline_parallelepiped(p,
		glm::vec3{ hs,0,0 },
		glm::vec3{ 0,hs,0 },
		glm::vec3{ 0,0,hs },
		color);
}

void c_drawer::add_debugline_list(const std::vector<glm::vec3>& pts, glm::vec3 color)
{
	for (auto p : pts)
		m_debug_lines.push_back({ p,color });
}

void c_drawer::add_debugtri_list(const std::vector<glm::vec3>& pts, glm::vec3 color)
{
	for (auto p : pts)
		m_debug_tri.push_back({ p,color });
}

c_drawer & c_drawer::get_instance()
{
	static c_drawer instance;
	return instance;
}
