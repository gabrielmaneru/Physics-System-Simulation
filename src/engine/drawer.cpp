#include "drawer.h"
#include "shader_program.h"
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
		m_debug_shader = new Shader_Program("debug.vert", "debug.frag");
	}
	catch (const std::string & log) { std::cout << log; }

	return true;
}

void c_drawer::render()
{
	m_camera.update();
	glClearColor(0.1f, 0.1f, 0.1f, 1);
	glClear(GL_COLOR_BUFFER_BIT);
	glm::mat4 vp = m_camera.get_vp();

	glDrawArrays(GL_TRIANGLES, 0, 3);

	if (!m_debug_lines.empty())
	{
		if (m_debug_vao == 0)
			glGenVertexArrays(1, &m_debug_vao);
		glBindVertexArray(m_debug_vao);

		if (m_debug_vbo == 0)
			glGenBuffers(1, &m_debug_vbo);
		glBindBuffer(GL_ARRAY_BUFFER, m_debug_vbo);
		glBufferData(GL_ARRAY_BUFFER, sizeof(debug_vertex)*m_debug_lines.size(), m_debug_lines.data(), GL_DYNAMIC_DRAW);

		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
		glEnableVertexAttribArray(1);

		m_debug_shader->use();
		m_debug_shader->set_uniform("vp", vp);
		glDrawArrays(GL_LINES, 0, m_debug_lines.size());

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
		m_debug_lines.clear();
	}
}

void c_drawer::shutdown()
{
}

void c_drawer::add_debug_line(glm::vec3 p0, glm::vec3 p1, glm::vec3 color)
{
	m_debug_lines.push_back({ p0,color });
	m_debug_lines.push_back({ p1,color });
}

void c_drawer::add_debug_cube(glm::vec3 p, float size, glm::vec3 color)
{
	float hs = size * 0.5f;
	glm::vec3 p_min = p - glm::vec3(hs);
	glm::vec3 p_max = p + glm::vec3(hs);

	add_debug_line(p_min, p + glm::vec3{ hs,-hs,-hs }, color);
	add_debug_line(p_min, p + glm::vec3{ -hs,hs,-hs }, color);
	add_debug_line(p_min, p + glm::vec3{ -hs,-hs,hs }, color);

	add_debug_line(p + glm::vec3{ hs,-hs,-hs }, p + glm::vec3{ hs,-hs,hs }, color);
	add_debug_line(p + glm::vec3{ -hs,-hs,hs }, p + glm::vec3{ hs,-hs,hs }, color);
	add_debug_line(p + glm::vec3{ -hs,-hs,hs }, p + glm::vec3{ -hs,hs,hs }, color);
	add_debug_line(p + glm::vec3{ -hs,hs,-hs }, p + glm::vec3{ -hs,hs,hs }, color);
	add_debug_line(p + glm::vec3{ -hs,hs,-hs }, p + glm::vec3{ hs,hs,-hs }, color);
	add_debug_line(p + glm::vec3{ hs,-hs,-hs }, p + glm::vec3{ hs,hs,-hs }, color);

	add_debug_line(p + glm::vec3{ -hs,hs,hs }, p_max, color);
	add_debug_line(p + glm::vec3{ hs,-hs,hs }, p_max, color);
	add_debug_line(p + glm::vec3{ hs,hs,-hs }, p_max, color);
}

c_drawer & c_drawer::get_instance()
{
	static c_drawer instance;
	return instance;
}
