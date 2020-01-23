#include "face.h"
#include "half_edge.h"

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
