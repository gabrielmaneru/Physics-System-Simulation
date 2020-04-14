#pragma once
#include "contact_info.h"
#include <vector>

struct constraint_contact_solver
{
	const int m_iteration_count{ 1 };
	const float m_baumgarte{ 0.0f };
	void evaluate(std::vector<overlap_pair*>& overlaps);
};