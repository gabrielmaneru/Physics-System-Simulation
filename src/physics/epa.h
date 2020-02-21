#pragma once
#include "gjk.h"
#include "physical_mesh.h"

struct epa
{
	enum status {
		e_Running,
		e_Success,
		e_Fail_InvalidSimpler,
		e_Fail_IterationLimit
	};

	status evaluate(gjk& solver, glm::vec3 initial_dir);
	face* find_closer_face();
	void expand(face* f, glm::vec3 w);
	bool get_edge_dist(glm::vec3 n, uint a, uint b, float& dist)const;
	float get_face_dist(face* f)const;

	status m_status{ e_Running };
	uint m_iterations{ 0u };
	physical_mesh m_start;
	physical_mesh m_polytope;
	glm::vec3 m_normal;
	float m_depth;
	simplex m_result;

	const static uint c_max_iterations;
};