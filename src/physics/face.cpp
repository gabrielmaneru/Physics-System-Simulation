/**
 * @file face.cpp
 * @author Gabriel Maneru, gabriel.m, gabriel.m@digipen.edu
 * @date 01/28/2020
 * @brief Face structure
 * @copyright Copyright (C) 2020 DigiPen Institute of Technology.
**/
#include "face.h"
#include "half_edge.h"

/**
 * Recompute face indices base on the cylclic edges
**/
void face::refresh()
{
	m_indices.clear();
	half_edge* hedge = m_hedge_start;
	do
	{
		m_indices.push_back(hedge->m_vertex_idx);
		hedge->m_face = this;
		hedge = hedge->m_next;
	} while (hedge != m_hedge_start);
}
