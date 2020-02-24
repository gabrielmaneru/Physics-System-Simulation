#pragma once
#include "physical_mesh.h"
#include <array>

namespace voronoi
{
	constexpr uint flag[]{ 1<<0, 1<<1, 1<<2, 1<<3};
	const uint seg_region = flag[0] | flag[1];
	const uint face_region = flag[0] | flag[1] | flag[2];
	const uint tetra_region = flag[0] | flag[1]| flag[2] | flag[3];
}

struct simplex
{
	simplex() = default;
	simplex(glm::vec3 a, glm::vec3 b);
	simplex(glm::vec3 a, glm::vec3 b, glm::vec3 c);
	simplex(glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 d);
	float project_origin();
	const glm::vec3& last()const;
	simplex get_next(float& distance, glm::vec3& next_dir);

	uint m_dim{ 0u };
	std::array<glm::vec3, 4> m_points;
	std::array<glm::vec3, 4> m_dirs;
	glm::vec4 m_bary{ 0.0f };
	uint m_voronoi{ 0u };
};
struct gjk
{
	enum status {
		e_Running,
		e_Success,
		e_Fail_NoFurthestPoint,
		e_Fail_ProjectionFail,
		e_Fail_IterationLimit
	};

	gjk(const physical_mesh& A, const physical_mesh& B, const glm::mat4& modA, const glm::mat4& modB, const glm::mat3& basisA, const glm::mat3& basisB);
	status evaluate(glm::vec3 initial_dir);
	glm::vec3 supportA(glm::vec3 dir, const physical_mesh& target)const;
	glm::vec3 supportB(glm::vec3 dir, const physical_mesh& target)const;
	glm::vec3 support(glm::vec3 dir)const;
	glm::vec3 support(glm::vec3 dir, uint index)const;
	void add_vertex(simplex& simp, glm::vec3 dir)const;
	void rem_vertex(simplex& simp)const;
	bool complete_simplex();
	
	const physical_mesh& m_mesh_A;
	const physical_mesh& m_mesh_B;
	const glm::mat4 m_mod_to_A;
	const glm::mat3 m_mod_to_B;

	status m_status{ e_Running };
	uint m_iterations{ 0u };
	glm::vec3 m_dir{ 0.0f };
	simplex m_simplex;
	glm::vec3 m_prev[4];

	const static float c_min_distance;
	const static uint c_max_iterations;
};