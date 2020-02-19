#pragma once
#include "physical_mesh.h"
#include <array>

struct simplex
{
	std::array<glm::vec3, 4> m_points;
	std::array<glm::vec3, 4> m_dirs;
	glm::vec4 m_bary{ 0.0f };
	uint m_dim{ 0u };
};
struct gjk
{
	gjk(const physical_mesh& A, const physical_mesh& B, const glm::mat4& modA, const glm::mat4& modB);
	bool evaluate(glm::vec3 initial_dir);
	glm::vec3 support(glm::vec3 dir)const;
	void add_vertex(simplex& simp, glm::vec3 dir)const;
	float project_origin_2D(
		glm::vec3 a, glm::vec3 b,
		glm::vec4& bary, uint& voronoi_mask)const;
	float project_origin_3D(
		glm::vec3 a, glm::vec3 b, glm::vec3 c,
		glm::vec4& bary, uint& voronoi_mask)const;
	float project_origin_4D(
		glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 d,
		glm::vec4& bary, uint& voronoi_mask)const;
	
	const physical_mesh& m_mesh_A;
	const physical_mesh& m_mesh_B;
	const glm::mat4 m_mod_A;
	const glm::mat4 m_mod_B;
	const glm::mat4 m_invmod_A;
	const glm::mat4 m_invmod_B;

	enum status{
		e_Running,
		e_Success,
		e_Failed
	} m_status;
	uint m_iterations{ 0u };
	glm::vec3 m_dir{ 0.0f };
	simplex m_simplex;
	float m_angle{ 0.0f };

	const static float c_min_distance;
	const static uint c_max_iterations;
	const static uint voronoi_flag[4];
};