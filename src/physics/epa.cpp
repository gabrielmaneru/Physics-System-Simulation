#include "epa.h"

const uint epa::c_max_iterations = 255u;
epa::epa(gjk & asolver)
	:solver(asolver)
{
	// Expand the simplex into tetrahedron
	if (solver.m_simplex.m_dim <= 1 || !solver.complete_simplex())
		m_status = e_Fail_InvalidSimpler;
	else
	{
		// Orient simplex
		float v = glm::determinant(glm::mat3{
			solver.m_simplex.m_points[0] - solver.m_simplex.m_points[3],
			solver.m_simplex.m_points[0] - solver.m_simplex.m_points[3],
			solver.m_simplex.m_points[0] - solver.m_simplex.m_points[3]
			});

		// Swap p0<->p1
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
		m_polytope.create_twins();

		// Keep track of original directions
		m_dirs = {
		   solver.m_simplex.m_dirs.begin(),
		   solver.m_simplex.m_dirs.end()
		};
	}

}

epa::status epa::evaluate()
{
	// Check if initialisation failed
	if (m_status != e_Running)
		return m_status;

	// Find starting best face
	face* closer = find_closer_face();
	face* outer = nullptr;
	float dist = FLT_MAX;

	for (;;)
	{
		glm::vec3 n = m_polytope.get_face_plane(closer->m_hedge_start);
		glm::vec3 w = solver.support(n);
		float wdist = glm::dot(n, w) - closer->m_distance;

		// Check accuracy limit
		if (wdist < dist)
			dist = wdist;
		else
			break;

		// Expand Face
		m_dirs.push_back(n);
		expand(closer, w);

		// Get newer face
		closer = find_closer_face();

		// Prevent infinite loops
		if (m_status == e_Running && ++m_iterations == c_max_iterations)
			return m_status = e_Fail_IterationLimit;
	}

	m_normal = closer->m_plane;
	m_depth = closer->m_distance;

	m_result.m_dim = 3;
	m_result.m_points[0] = m_polytope.m_vertices[closer->m_indices[0]];
	m_result.m_points[1] = m_polytope.m_vertices[closer->m_indices[1]];
	m_result.m_points[2] = m_polytope.m_vertices[closer->m_indices[2]];

	m_result.m_dirs[0] = m_dirs[closer->m_indices[0]];
	m_result.m_dirs[1] = m_dirs[closer->m_indices[1]];
	m_result.m_dirs[2] = m_dirs[closer->m_indices[2]];

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
		float d = f.m_distance;
		if (dist < 0.0f || d < dist)
			dist = d, best = &f;
	}
	return best;
}

void epa::expand(face *& f, glm::vec3 w)
{
	uint new_w = m_polytope.m_vertices.size();
	m_polytope.m_vertices.push_back(w);

	// Find if some face is breaking convexity
	face* other_concave{ nullptr };
	half_edge* edge = f->m_hedge_start;
	do
	{
		if(edge->m_twin
		&& check_convexity(f,edge->m_twin->m_face))
			other_concave = edge->m_twin->m_face;

		edge = edge->m_next;
	} while (edge != f->m_hedge_start);

	// Expand using other concave face
	if (other_concave)
	{
		expand_concave(f, other_concave, new_w);
		other_concave = nullptr;
	}
	// Expand as a pyramid
	else
		expand_pyramid(f, new_w);

	// Remove obsolete edges
	for (auto it = m_polytope.m_hedges.begin(); it != m_polytope.m_hedges.end();)
		it = (it->m_prev == nullptr) ? m_polytope.m_hedges.erase(it) : it++;

	// Remove obsolete faces
	for (auto it = m_polytope.m_faces.begin(); it != m_polytope.m_faces.end();)
		it = (it->m_hedge_start == nullptr) ? m_polytope.m_faces.erase(it) : it++;

	// Avoid access to deleted items
	f = nullptr;
	m_polytope.create_twins();
}

void epa::expand_pyramid(face * f, uint w)
{
	std::vector<uint>v;
	half_edge* edge = f->m_hedge_start;
	do
	{
		// Add new pyramid vertex
		v.push_back(edge->m_vertex_idx);

		// Disconnect from twin
		if (edge->m_twin)
			edge->m_twin->m_twin = nullptr;

		// Mark as obsolete
		edge->m_prev = nullptr;

		// Proceed iteration
		edge = edge->m_next;
	} while (edge != f->m_hedge_start);

	// Add pyramidal faces
	for (uint i = 0; i < v.size(); i++)
		m_polytope.add_face({ v[i], v[(i + 1) % v.size()], w });

	// Mark initial face as obsolete
	f->m_hedge_start = nullptr;
}

void epa::expand_concave(face * a, face * b, uint w)
{
	//TODO
	half_edge* edge = a->m_hedge_start;
	do
	{
		if(edge->m_twin
		&& edge->m_twin->m_face == b)
		// Disconnect from twin
		if (edge->m_twin)
			edge->m_twin->m_twin = nullptr;

		// Mark as obsolete
		edge->m_prev = nullptr;

		// Proceed iteration
		edge = edge->m_next;
	} while (edge != f->m_hedge_start);
}

bool epa::check_convexity(face * a, face * b) const
{
	glm::vec3 diff = b->m_center - a->m_center;
	glm::vec3 n = a->m_plane;
	return glm::dot(n, diff) > 0.0f;
}
