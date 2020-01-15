#pragma once
#include <vector>

struct half_edge;
struct face
{
	std::vector<unsigned int> m_indices;
	std::vector<half_edge*> m_p_hedges;
};