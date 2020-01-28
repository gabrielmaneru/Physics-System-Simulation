/**
 * @file shader_program.h
 * @author Gabriel Maneru, gabriel.m, gabriel.m@digipen.edu
 * @date 01/28/2020
 * @brief Shader Program structure
 * @copyright Copyright (C) 2020 DigiPen Institute of Technology.
**/
#pragma once
#include <glm/glm.hpp>
#include <string>
#include <map>

class Shader_Program
{
public:
	Shader_Program(const std::string& vtx, const std::string& frag);
	~Shader_Program();

	bool is_valid()const;
	void recompile();
	void use()const;
	void set_uniform(const char *name, bool val) const;
	void set_uniform(const char *name, int val) const;
	void set_uniform(const char *name, float val) const;
	void set_uniform(const char *name, const glm::vec2 & v) const;
	void set_uniform(const char *name, const glm::vec3 & v) const;
	void set_uniform(const char *name, const glm::vec4 & v) const;
	void set_uniform(const char *name, const glm::mat3 & m) const;
	void set_uniform(const char *name, const glm::mat4 & m) const;
	std::string paths[2]{};

private:
	enum class e_shader_type { VERTEX, FRAGMENT};
	bool create_handle();
	void compile_program();
	bool compile_shader(const std::string& filename, const e_shader_type& type);
	void link();
	int uniform_location(const char * name) const;

	mutable std::map<std::string, int> m_uniform_location_map{};
	int m_handle{0};
	bool m_linked{false};
};