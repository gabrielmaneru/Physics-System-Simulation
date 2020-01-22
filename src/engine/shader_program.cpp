#include "shader_program.h"
#include <glad/glad.h>
#include <fstream>
#include <sstream>
#include <iostream>

Shader_Program::Shader_Program(const std::string & vtx, const std::string & frag)
{
	const char * path = "../resources/shaders/";
	paths[0] = path + vtx;
	paths[1] = path + frag;
	if (create_handle())
		compile_program();
}
Shader_Program::~Shader_Program()
{
	if (m_handle > 0)
		glDeleteProgram(m_handle);
}

bool Shader_Program::is_valid()const
{
	return m_handle > 0 && m_linked;
}

void Shader_Program::recompile()
{
	try {
		compile_program();
	}
	catch (const std::string & log) { std::cout << log; }
}

void Shader_Program::use() const
{
	if (is_valid())
		glUseProgram(m_handle);
}

void Shader_Program::set_uniform(const char * name, bool val) const
{
	set_uniform(name, val ? 1 : 0);
}

void Shader_Program::set_uniform(const char * name, int val) const
{
	int loc = uniform_location(name);
	if (loc >= 0)
		glUniform1i(loc, val);
}

void Shader_Program::set_uniform(const char * name, float val) const
{
	int loc = uniform_location(name);
	if (loc >= 0)
		glUniform1f(loc, val);
}

void Shader_Program::set_uniform(const char * name, const glm::vec2 & v) const
{
	int loc = uniform_location(name);
	if (loc >= 0)
		glUniform2f(loc, v.x, v.y);
}

void Shader_Program::set_uniform(const char * name, const glm::vec3 & v) const
{
	int loc = uniform_location(name);
	if (loc >= 0)
		glUniform3f(loc, v.x, v.y, v.z);
}

void Shader_Program::set_uniform(const char * name, const glm::vec4 & v) const
{
	int loc = uniform_location(name);
	if (loc >= 0)
		glUniform4f(loc, v.x, v.y, v.z, v.w);
}

void Shader_Program::set_uniform(const char * name, const glm::mat3 & m) const
{
	int loc = uniform_location(name);
	if (loc >= 0)
		glUniformMatrix3fv(loc, 1, GL_FALSE, &m[0][0]);
}

void Shader_Program::set_uniform(const char * name, const glm::mat4 & m) const
{
	int loc = uniform_location(name);
	if (loc >= 0)
		glUniformMatrix4fv(loc, 1, GL_FALSE, &m[0][0]);
}

bool Shader_Program::create_handle()
{
	if (m_handle <= 0)
	{
		m_handle = glCreateProgram();
		if (m_handle == 0)
		{
			std::cout << "Unable to create shader program." << std::endl;
			return false;
		}
	}
	return true;
}

void Shader_Program::compile_program()
{
	if (compile_shader(paths[0], e_shader_type::VERTEX))
		if (compile_shader(paths[1], e_shader_type::FRAGMENT))
			link();

	if (!is_valid())
		throw std::string("Compile Error");
}

bool Shader_Program::compile_shader(const std::string & filename, const e_shader_type & type)
{
	// Open file
	std::ifstream code_file(filename.c_str(), std::ios::in);
	if (!code_file.is_open())
	{
		std::cout << "File not found." << std::endl;
		return false;
	}

	// Dump code
	std::ostringstream tmp_code;
	while (code_file.good())
	{
		int c = code_file.get();
		if (!code_file.eof())
			tmp_code << static_cast<char>(c);
	}
	code_file.close();
	std::string str_code{ tmp_code.str() };
	const char * code = str_code.c_str();

	// Create Handle
	GLuint shader = 0;
	switch (type)
	{
	case e_shader_type::VERTEX:
		shader = glCreateShader(GL_VERTEX_SHADER);
		break;
	case e_shader_type::FRAGMENT:
		shader = glCreateShader(GL_FRAGMENT_SHADER);
		break;
	default:
		return false;
	}
	glShaderSource(shader, 1, &code, NULL);
	glCompileShader(shader);

	// Check errors
	int result;
	glGetShaderiv(shader, GL_COMPILE_STATUS, &result);
	if (GL_FALSE == result)
	{
		int length{ 0 };
		glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length);
		if (length > 0)
		{
			char * c_log = new char[length];
			int written = 0;
			glGetShaderInfoLog(shader, length, &written, c_log);
			std::cout << c_log << std::endl;
			delete[] c_log;
		}

		return false;
	}

	// Attach
	glAttachShader(m_handle, shader);
	glDeleteShader(shader);
	
	return true;
}
void Shader_Program::link()
{
	if (m_linked || m_handle <= 0)
		return;

	// Link
	glLinkProgram(m_handle);

	// Check Errors
	int status = 0;
	glGetProgramiv(m_handle, GL_LINK_STATUS, &status);
	if (GL_FALSE == status)
	{
		int length = 0;
		glGetProgramiv(m_handle, GL_INFO_LOG_LENGTH, &length);
		if (length > 0)
		{
			char * c_log = new char[length];
			int written = 0;
			glGetProgramInfoLog(m_handle, length, &written, c_log);
			std::cout << c_log << std::endl;
			delete[] c_log;
		}
	}
	else
		m_linked = true;
}

int Shader_Program::uniform_location(const char * name) const
{
	auto it = m_uniform_location_map.find(name);
	if (it != m_uniform_location_map.end())
		return it->second;

	int tmp_handle = glGetUniformLocation(m_handle, name);
	if (tmp_handle >= 0)
		m_uniform_location_map[name] = tmp_handle;
	else
		std::cout << "Uniform: " << name << " not found." << std::endl;
	return tmp_handle;
}
