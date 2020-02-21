#include "epa.h"

const uint epa::c_max_iterations = 255u;
epa::status epa::evaluate(gjk & solver, glm::vec3 initial_dir)
{
	if (solver.m_simplex.m_dim <= 1 || !solver.complete_simplex())
		return m_status = e_Fail_InvalidSimpler;

	// Orient simplex
	float v = glm::determinant(glm::mat3{
		solver.m_simplex.m_points[0] - solver.m_simplex.m_points[3],
		solver.m_simplex.m_points[0] - solver.m_simplex.m_points[3],
		solver.m_simplex.m_points[0] - solver.m_simplex.m_points[3]
	});
	if (v < 0.0f)
	{
		std::swap(solver.m_simplex.m_points[0], solver.m_simplex.m_points[1]);
		std::swap(solver.m_simplex.m_dirs[0], solver.m_simplex.m_dirs[1]);
		std::swap(solver.m_simplex.m_bary[0], solver.m_simplex.m_bary[1]);
	}

	// Create polytope
	m_polytope.m_vertices = { 
		solver.m_simplex.m_points.begin(),
		solver.m_simplex.m_points.end()
	};
	m_polytope.add_face({ 0u, 1u, 2u });
	m_polytope.add_face({ 1u, 0u, 3u });
	m_polytope.add_face({ 2u, 1u, 3u });
	m_polytope.add_face({ 0u, 2u, 3u });

	m_start.m_vertices = {
		solver.m_simplex.m_points.begin(),
		solver.m_simplex.m_points.end()
	};
	m_start.add_face({ 0u, 1u, 2u });
	m_start.add_face({ 1u, 0u, 3u });
	m_start.add_face({ 2u, 1u, 3u });
	m_start.add_face({ 0u, 2u, 3u });

	// Keep track of original directions
	std::vector<glm::vec3> dirs = {
		solver.m_simplex.m_dirs.begin(),
		solver.m_simplex.m_dirs.end()
	};


	// Find starting best face
	face* closer = find_closer_face();
	face* outer = nullptr;
	float dist = FLT_MAX;

	for (;;)
	{
		glm::vec3 n = m_polytope.get_face_plane(closer->m_hedge_start);
		glm::vec3 w = solver.support(n);
		float wdist = glm::dot(n, w) - get_face_dist(closer);

		// Check accuracy limit
		if (wdist < dist)
			dist = wdist;
		else
			break;

		// Expand Face
		dirs.push_back(n);
		expand(closer, w);

		// Get newer face
		closer = find_closer_face();

		// Prevent infinite loops
		if (m_status == e_Running && ++m_iterations == c_max_iterations)
			return m_status = e_Fail_IterationLimit;
	}

	m_normal = m_polytope.get_face_plane(closer->m_hedge_start);
	m_depth = get_face_dist(closer);

	m_result.m_dim = 3;
	m_result.m_points[0] = m_polytope.m_vertices[closer->m_indices[0]];
	m_result.m_points[1] = m_polytope.m_vertices[closer->m_indices[1]];
	m_result.m_points[2] = m_polytope.m_vertices[closer->m_indices[2]];

	m_result.m_dirs[0] = dirs[closer->m_indices[0]];
	m_result.m_dirs[1] = dirs[closer->m_indices[1]];
	m_result.m_dirs[2] = dirs[closer->m_indices[2]];

	glm::vec3 proj = m_normal * m_depth;
	float b0 = glm::length(glm::cross(m_result.m_points[1] - proj, m_result.m_points[2] - proj));
	float b1 = glm::length(glm::cross(m_result.m_points[2] - proj, m_result.m_points[0] - proj));
	float b3 = glm::length(glm::cross(m_result.m_points[0] - proj, m_result.m_points[1] - proj));

	float tot = b0 + b1 + b3;
	m_result.m_bary[0] = b0 / tot;
	m_result.m_bary[1] = b1 / tot;
	m_result.m_bary[2] = b3 / tot;

	return m_status = e_Success;
}

face * epa::find_closer_face()
{
	float dist = -1.f;
	face * best{ nullptr };
	for (auto& f : m_polytope.m_faces)
	{
		float d = get_face_dist(&f);
		if (dist < 0.0f || d < dist)
			dist = d, best = &f;
	}
	return best;
}

void epa::expand(face * f, glm::vec3 w)
{
	std::vector<uint>v;
	half_edge* edge = f->m_hedge_start;
	do
	{
		v.push_back(edge->m_vertex_idx);
		edge->m_prev = nullptr;
		edge = edge->m_next;
	} while (edge != f->m_hedge_start);
	
	uint new_w = m_polytope.m_vertices.size();
	m_polytope.m_vertices.push_back(w);
	for (uint i = 0; i < v.size(); i++)
		m_polytope.add_face({ v[i], v[(i + 1) % v.size()], new_w });
	
	for (auto it = m_polytope.m_hedges.begin(); it != m_polytope.m_hedges.end();)
	{
		if (it->m_prev == nullptr)
			it = m_polytope.m_hedges.erase(it);
		else
			it++;
	}
	for (auto it = m_polytope.m_faces.begin(); it != m_polytope.m_faces.end();it++)
		if (&*it == f)
		{
			m_polytope.m_faces.erase(it);
			break;
		}
}

bool epa::get_edge_dist(glm::vec3 n, uint a, uint b, float & dist) const
{
	glm::vec3 va = m_polytope.m_vertices[a];
	glm::vec3 vb = m_polytope.m_vertices[b];
	glm::vec3 diff = vb - va;
	glm::vec3 not_ab = glm::cross(diff,n);

	// Outside
	if (glm::dot(va, not_ab) < 0.0f)
	{
		if (glm::dot(va, diff) > 0.0f)
			dist = glm::length(va);
		else if(glm::dot(vb, diff) < 0.0f)
			dist = glm::length(vb);
		else
		{
			float diff_len = glm::length2(diff);
			float a_b = glm::dot(va, vb);
			dist = glm::sqrt(glm::max((glm::length2(va)*glm::length2(vb) - a_b * a_b) / glm::length2(diff), 0.0f));
		}
		return true;
	}

	// Inside or other edge
	return false;
}

float epa::get_face_dist(face * f) const
{
	float dist = -1.f;
	glm::vec3 n = m_polytope.get_face_plane(f->m_hedge_start);
	uint s = f->m_indices.size();
	for (uint i = 0; i < s; ++i)
	{
		if (get_edge_dist(n, f->m_indices[i], f->m_indices[(i + 1) % s], dist))
			break;
	}

	// If inside, use plane dist
	if (dist < 0.0f)
		dist = glm::dot(m_polytope.m_vertices[f->m_indices[0]], n);
	return dist;
}

