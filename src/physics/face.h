/**
 * @file face.h
 * @author Gabriel Maneru, gabriel.m, gabriel.m@digipen.edu
 * @date 01/28/2020
 * @brief Face structure
 * @copyright Copyright (C) 2020 DigiPen Institute of Technology.
**/
#pragma once
#include <vector>

struct half_edge;
using uint = unsigned int;

struct face
{
	void refresh();

	std::vector<uint> m_indices;
	half_edge* m_hedge_start;
};