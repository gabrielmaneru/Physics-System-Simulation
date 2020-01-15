#include "physical_mesh.h"

glm::vec4 physical_mesh::get_face_plane(const half_edge * hedge) const
{
	glm::vec3 p0 = m_vertices[hedge->get_start()].m_position;
	glm::vec3 p1 = m_vertices[hedge->get_end()].m_position;
	glm::vec3 p2 = m_vertices[hedge->get_other()].m_position;

	glm::vec3 v0 = glm::normalize(p1 - p0);
	glm::vec3 v1 = glm::normalize(p2 - p1);

	glm::vec3 n = glm::normalize(glm::cross(v0, v1));
	return glm::vec4(n.x, n.y, n.z, glm::dot(n, p0));
}

bool physical_mesh::is_coplanar(const half_edge * hedge) const
{
	if (hedge->m_twin == nullptr)
		return false;

	glm::vec4 plane1 = get_face_plane(hedge);
	glm::vec3 p_in_edge2 = m_vertices[hedge->m_twin->get_other()].m_position;
	float dot = glm::dot(glm::vec3(plane1), p_in_edge2);
	float dist_to_plane = dot - plane1.w;
	return abs(dist_to_plane) < FLT_EPSILON;
}

void physical_mesh::add_face(const std::vector<uint>& indices)
{
	// Assert minimum face is a triangle
	assert(indices.size() >= 3);

	// Create a face
	m_faces.push_back({});
	face* f = &m_faces.back();
	f->m_indices = indices;

	// Create all half edges
	half_edge* last{ nullptr };
	for (uint i = 0; i < indices.size(); i++)
	{
		// Create a half_edge
		m_hedges.push_back({});
		half_edge * cur = &m_hedges.back();

		// Initialize
		cur->m_face = f;
		cur->m_vertex_idx = indices[i];

		// Set up double linked-list
		if (last != nullptr)
		{
			cur->m_prev = last;
			last->m_next = cur;
		}
		else
			f->m_hedge_start = cur;

		cur->m_twin = nullptr;
		last = cur;
	}

	// Finish cyclic linked-list
	f->m_hedge_start->m_prev = last;
	last->m_next = f->m_hedge_start;
}

void physical_mesh::create_twins()
{
	// For each face
	std::list<face>::const_iterator it = m_faces.cbegin();
	for (; it != m_faces.end(); it++)
	{
		// Get Start of linked list
		half_edge * edge1 = it->m_hedge_start;

		// Do one cycle through the list
		do
		{
			// For each other face until you find a twin
			std::list<face>::const_iterator other = it;
			for (other++; other != m_faces.end() && edge1->m_twin == nullptr; other++)
			{
				// Get Start of other linked list
				half_edge * edge2 = other->m_hedge_start;

				// Do one cycle through the other list
				do
				{
					if (edge2->m_twin == nullptr
					&&  edge2->get_start() == edge1->get_end()
					&&  edge2->get_end() == edge1->get_start())
					{
						edge1->m_twin = edge2;
						edge2->m_twin = edge1;
						break;
					}
					edge2 = edge2->m_next;
				} while (edge2 != other->m_hedge_start);
			}
			edge1 = edge1->m_next;
		} while (edge1 != it->m_hedge_start);
	}
}

void physical_mesh::merge_coplanar()
{
	// For each face
	std::list<face>::const_iterator it = m_faces.cbegin();
	for (; it != m_faces.end(); it++)
	{
		// Get Start of linked list
		half_edge * edge1 = it->m_hedge_start;

		// Do one cycle through the list
		do
		{
			// If the edge has a twin
			if (edge1->m_twin != nullptr && is_coplanar(edge1))
			{
				//merge them
			}
			edge1 = edge1->m_next;
		} while (edge1 != it->m_hedge_start);
	}
}
