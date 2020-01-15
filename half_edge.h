#pragma once
#include "face.h"

struct half_edge
{
	face * m_face;
	uint m_vertex_idx;
	half_edge* m_prev;
	half_edge* m_next;
	half_edge* m_twin;

	uint get_other()const;
	uint get_start()const;
	uint get_end()const;
};