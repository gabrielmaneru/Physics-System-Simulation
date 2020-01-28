/**
 * @file half_edge.cpp
 * @author Gabriel Maneru, gabriel.m, gabriel.m@digipen.edu
 * @date 01/28/2020
 * @brief Half-edge structure
 * @copyright Copyright (C) 2020 DigiPen Institute of Technology.
**/
#include "half_edge.h"

/**
 * Helper functions for retrieve the
 * neighboor edges
**/
uint half_edge::get_other() const
{
	return m_next->m_vertex_idx;
}
uint half_edge::get_start() const
{
	return m_prev->m_vertex_idx;
}
uint half_edge::get_end() const
{
	return m_vertex_idx;
}
