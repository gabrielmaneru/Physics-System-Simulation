/**
 * @file face.cpp
 * @author Gabriel Maneru, gabriel.m, gabriel.m@digipen.edu
 * @date 01/28/2020
 * @brief Face structure
 * @copyright Copyright (C) 2020 DigiPen Institute of Technology.
**/
#include "face.h"
#include "half_edge.h"
#include "physical_mesh.h"

face::face(const physical_mesh *m)
	:m_owner(m)
{}

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

	// Compute normal
	glm::vec3 p0 = m_owner->m_vertices[hedge->get_start()];
	glm::vec3 p1 = m_owner->m_vertices[hedge->get_end()];
	glm::vec3 p2 = m_owner->m_vertices[hedge->get_other()];
	glm::vec3 v0 = glm::normalize(p1 - p0);
	glm::vec3 v1 = glm::normalize(p2 - p1);
	glm::vec3 n = glm::normalize(glm::cross(v0, v1));
	m_plane = glm::vec4(n.x, n.y, n.z, glm::dot(n, p0));

	// Compute distance
	uint s = m_indices.size();
	for (uint i = 0; i < s && m_distance < 0.0f; ++i)
	{
		glm::vec3 va = m_owner->m_vertices[m_indices[i]];
		glm::vec3 vb = m_owner->m_vertices[m_indices[(i+1)%s]];
		glm::vec3 diff = vb - va;
		glm::vec3 not_ab = glm::cross(diff, n);

		// Outside
		if (glm::dot(va, not_ab) < 0.0f)
		{
			if (glm::dot(va, diff) > 0.0f)
				m_distance = glm::length(va);
			else if (glm::dot(vb, diff) < 0.0f)
				m_distance = glm::length(vb);
			else
			{
				float diff_len = glm::length2(diff);
				float a_b = glm::dot(va, vb);
				m_distance = glm::sqrt(glm::max((glm::length2(va)*glm::length2(vb) - a_b * a_b) / glm::length2(diff), 0.0f));
			}
		}
		// Inside or on other edge
	}
	// If inside, use plane dist
	if (m_distance < 0.0f)
		m_distance = m_plane.w;

	// Copmute middle point
	m_center = glm::vec3{ 0.0f };
	for (uint i = 0; i < s; ++i)
		m_center += m_owner->m_vertices[m_indices[i]];
	m_center /= (float)m_indices.size();

}
