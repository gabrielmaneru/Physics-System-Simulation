#pragma once
#include "contact_info.h"
#include <vector>

struct naive_contact_solver
{
	void evaluate(std::vector<contact_point>& contacts);
};

struct constraint_contact_solver
{
	const int m_iteration_count{ 1 };
	const float m_baumgarte{ 0.0f };
	const float m_restitution{ 0.0f };
	void evaluate(std::vector<contact_point>& contacts);
};