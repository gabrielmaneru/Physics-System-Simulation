#include "epa.h"

void epa::evaluate(gjk & solver, glm::vec3 initial_dir)
{
	if (solver.m_simplex.m_dim <= 1 || !solver.enclose_simplex())
	{
		m_status = epa::e_Failed;
		return;
	}
		
	simplex& s = solver.m_simplex;
}

