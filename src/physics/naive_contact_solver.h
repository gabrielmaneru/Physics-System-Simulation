#pragma once
#include "contact_info.h"
#include <vector>

struct naive_contact_solver
{
	void evaluate(const std::vector<contact_info>& contacts);
};