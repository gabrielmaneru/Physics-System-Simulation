#pragma once
#include"gjk.h"

struct epa
{
	void evaluate(gjk& solver, glm::vec3 initial_dir);

	enum status {
		e_Running,
		e_Success,
		e_Failed
	} m_status{ e_Running };
};