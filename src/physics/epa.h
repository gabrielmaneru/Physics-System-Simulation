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
	std::vector<face*> expand_pyramid(face* f, uint w);
	void correct_concavity(face* a, face* b);
	void clear_obsolete_polytope();
	bool check_convexity(face* a, face* b)const;

	gjk& solver;
	status m_status{ e_Running };
	uint m_iterations{ 0u };
	std::vector<glm::vec3> m_dirs;
	physical_mesh m_polytope;
	simplex m_result;

	static int c_max_iterations;
};