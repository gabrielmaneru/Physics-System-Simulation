#pragma once
#include <vector>

struct half_edge;
using uint = unsigned int;

struct face
{
	std::vector<uint> m_indices;
	half_edge* m_hedge_start;
};