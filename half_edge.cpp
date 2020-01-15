#include "half_edge.h"

uint half_edge::get_start() const
{
	return m_prev->m_vertex_idx;
}

uint half_edge::get_end() const
{
	return m_vertex_idx;
}
