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
	epa(gjk& solver);
	status evaluate();
	face* find_closer_face();
	void expand(face*& f, glm::vec3 w); 
	void expand_pyramid(face* f, uint w);
	void expand_concave(face* a, face* b, uint w);
	bool check_convexity(face* a, face* b)const;

	gjk& solver;
	status m_status{ e_Running };
	uint m_iterations{ 0u };
	std::vector<glm::vec3> m_dirs;
	physical_mesh m_polytope;
	glm::vec3 m_normal;
	float m_depth;
	simplex m_result;

	const static uint c_max_iterations;
};