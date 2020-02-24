#include "epa.h"

int epa::c_max_iterations = 256u;
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
			solver.m_simplex.m_points[1] - solver.m_simplex.m_points[3],
			solver.m_simplex.m_points[2] - solver.m_simplex.m_points[3]
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

	face* closer{nullptr};
	glm::vec3 last{ FLT_MAX };	
	for (;;)
	{
		// Prevent infinite loops
		if (m_status == e_Running && m_iterations++ == c_max_iterations)
			return m_status = e_Fail_IterationLimit;

		// Get newer face
		closer = find_closer_face();
		glm::vec3 n = m_polytope.get_face_plane(closer->m_hedge_start);
		glm::vec3 w = solver.support(n);

		// Check limit reached
		if (w == last)
			break;
		else
			last = w;

		// Expand Face
		m_dirs.push_back(n);
		expand(closer, w);
	}
	
	m_result.m_dim = 3;
	m_result.m_points[0] = m_polytope.m_vertices[closer->m_indices[0]];
	m_result.m_points[1] = m_polytope.m_vertices[closer->m_indices[1]];
	m_result.m_points[2] = m_polytope.m_vertices[closer->m_indices[2]];

	m_result.m_dirs[0] = m_dirs[closer->m_indices[0]];
	m_result.m_dirs[1] = m_dirs[closer->m_indices[1]];
	m_result.m_dirs[2] = m_dirs[closer->m_indices[2]];

	glm::vec3 proj = closer->m_plane * closer->m_distance;
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
	// Add new point
	uint new_w = m_polytope.m_vertices.size();
	m_polytope.m_vertices.push_back(w);

	// Expand adding the new point
	std::vector<face*> new_faces{ expand_pyramid(f, new_w) };

	// Find if some face is breaking convexity
	for (auto face : new_faces)
	{
		half_edge* edge = face->m_hedge_start;
		if (edge->m_next->m_twin
		&&  check_convexity(face, edge->m_next->m_twin->m_face))
			correct_concavity(face, edge->m_next->m_twin->m_face);
	}
}

std::vector<face*> epa::expand_pyramid(face * f, uint w)
{
	// Add pyramidal faces
	std::vector<face*> faces;
	m_polytope.add_face({ f->m_indices[0], f->m_indices[1], w });
	faces.push_back(&m_polytope.m_faces.back());
	m_polytope.add_face({ f->m_indices[1], f->m_indices[2], w });
	faces.push_back(&m_polytope.m_faces.back());
	m_polytope.add_face({ f->m_indices[2], f->m_indices[0], w });
	faces.push_back(&m_polytope.m_faces.back());

	half_edge* edge = f->m_hedge_start;

	// Remove other twins
	if (edge->m_twin) edge->m_twin->m_twin = nullptr;
	if (edge->m_next->m_twin) edge->m_next->m_twin->m_twin = nullptr;
	if (edge->m_prev->m_twin) edge->m_prev->m_twin->m_twin = nullptr;

	// Mark the 3 edges as obsolete
	edge->m_next->m_prev = nullptr;
	edge->m_prev->m_prev = nullptr;
	edge->m_prev = nullptr;

	// Mark initial face as obsolete
	f->m_hedge_start = nullptr;

	clear_obsolete_polytope();
	return faces;
}

void epa::correct_concavity(face * a, face * b)
{
	// Choose intermediate hedge
	half_edge* edge = a->m_hedge_start;
	do
	{
		if (edge->m_twin && edge->m_twin->m_face == b)
			break;
		edge = edge->m_next;
	} while (edge != a->m_hedge_start);

	// Get new faces C, D
	std::vector<uint>
		indices_c{
		edge->get_end(),
		edge->m_next->get_end(),
		edge->m_twin->m_next->get_end()
	},
		indices_d{
		edge->m_twin->get_end(),
		edge->m_twin->m_next->get_end(),
		edge->m_next->get_end()
	};

	// Build new faces
	m_polytope.add_face(indices_c);
	m_polytope.add_face(indices_d);

	// Remove other twins
	if (edge->m_next->m_twin) edge->m_next->m_twin->m_twin = nullptr;
	if (edge->m_prev->m_twin) edge->m_prev->m_twin->m_twin = nullptr;
	if (edge->m_twin->m_next->m_twin) edge->m_twin->m_next->m_twin->m_twin = nullptr;
	if (edge->m_twin->m_prev->m_twin) edge->m_twin->m_prev->m_twin->m_twin = nullptr;

	// Mark the 6 edges as obsolete
	edge->m_next->m_prev = nullptr;
	edge->m_prev->m_prev = nullptr;
	edge->m_prev = nullptr;
	edge->m_twin->m_next->m_prev = nullptr;
	edge->m_twin->m_prev->m_prev = nullptr;
	edge->m_twin->m_prev = nullptr;

	// Mark the faces as obsolete
	a->m_hedge_start = nullptr;
	b->m_hedge_start = nullptr;

	clear_obsolete_polytope();
}

void epa::clear_obsolete_polytope()
{
	// Remove obsolete edges
	for (auto it = m_polytope.m_hedges.begin(); it != m_polytope.m_hedges.end();)
		it = (it->m_prev == nullptr) ? m_polytope.m_hedges.erase(it) : ++it;

	// Remove obsolete faces
	for (auto it = m_polytope.m_faces.begin(); it != m_polytope.m_faces.end();)
		it = (it->m_hedge_start == nullptr) ? m_polytope.m_faces.erase(it) : ++it;

	m_polytope.create_twins();
}

bool epa::check_convexity(face * a, face * b) const
{
	glm::vec3 diff = b->m_center - a->m_center;
	glm::vec3 n = a->m_plane;
	return glm::dot(n, diff) > 0.0f;
}
