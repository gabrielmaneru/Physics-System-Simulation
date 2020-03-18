#pragma once
#include "contact_info.h"
#include <vector>

struct naive_contact_solver
{
	void evaluate(std::vector<contact>& contacts);
};

struct constraint_contact_solver
{
	const int m_iteration_count{ 1 };
	void evaluate(std::vector<contact>& contacts);
};