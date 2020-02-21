#include "gjk.h"

simplex::simplex(glm::vec3 a, glm::vec3 b)
{
	m_points[m_dim++] = a;
	m_points[m_dim++] = b;
}
simplex::simplex(glm::vec3 a, glm::vec3 b, glm::vec3 c)
	:simplex(a, b)
{
	m_points[m_dim++] = c;
}
simplex::simplex(glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 d)
	:simplex(a, b, c)
{
	m_points[m_dim++] = d;
}

float simplex::project_origin()
{
	switch (m_dim)
	{
	case 2:
	{
		glm::vec3 diff = m_points[1] - m_points[0];

		// Check points are not equal
		float len2 = glm::length2(diff);
		if (len2 < c_epsilon)
			return -1.f;

		// Compute projection
		float t = -glm::dot(m_points[0], diff) / len2;

		// Fill voronoi mask
		if (t <= 0)
			m_voronoi = voronoi::flag[0];
		else if (t >= 1)
			m_voronoi = voronoi::flag[1];
		// Keep both vertices
		else
			m_voronoi = voronoi::seg_region;

		//Clamp projection onto line segment
		t = glm::clamp(t, 0.0f, 1.0f);

		// Store barycentric
		m_bary[0] = 1 - t;
		m_bary[1] = t;

		// return distance
		return glm::length2(m_points[0] + diff * t);
	}
	break;
	case 3:
	{
		glm::vec3 diff[] = {
			m_points[0] - m_points[1],
			m_points[1] - m_points[2],
			m_points[2] - m_points[0]
		};
		glm::vec3 norm = glm::cross(diff[0], diff[1]);

		// Check points are not equal
		float len2 = glm::length2(norm);
		if (len2 < c_epsilon)
			return -1.f;

		// Create simplexes of the three
		// segments of the face
		simplex segs[]{
			{m_points[0],m_points[1]},
			{m_points[1],m_points[2]},
			{m_points[2],m_points[0]}
		};

		// Compute distance to every edge
		float dist[] = { -1.f, -1.f, -1.f };
		for (uint cur = 0; cur < 3; ++cur)
			if (glm::dot(m_points[cur], glm::cross(diff[cur], norm)) > 0.0f)
				dist[cur] = segs[cur].project_origin();

		// Get close edge to origin
		uint closest{ 0u };
		float closest_dist{ -1.f };
		for (uint i = 0; i < 3; ++i)
			if (dist[i] != -1.f && (closest_dist < 0.f || dist[i] < closest_dist))
				closest_dist = dist[i], closest = i;

		// If origin lies closer to an edge region
		if (closest_dist >= 0.0f)
		{
			uint nex = (closest + 1) % 3;
			uint oth = (closest + 2) % 3;

			// Fill voronoi mask
			m_voronoi = 0u;
			if (segs[closest].m_voronoi & voronoi::flag[0])
				m_voronoi |= voronoi::flag[closest];
			if (segs[closest].m_voronoi & voronoi::flag[1])
				m_voronoi |= voronoi::flag[nex];

			// Assign barycentric
			m_bary[closest] = segs[closest].m_bary[0];
			m_bary[nex] = segs[closest].m_bary[1];
			m_bary[oth] = 0.f;
		}

		// If origin lies closer to the face region
		else
		{
			// Project origin onto the face
			float d = glm::dot(m_points[0], norm);
			glm::vec3 p = norm * (d / len2);
			closest_dist = glm::length2(p);

			// Keep the entire face
			m_voronoi = voronoi::face_region;

			// Assign barycentric
			float len = glm::sqrt(len2);
			m_bary[0] = glm::length(glm::cross(diff[1], m_points[1] - p)) / len;
			m_bary[1] = glm::length(glm::cross(diff[2], m_points[2] - p)) / len;
			m_bary[2] = 1 - m_bary[0] - m_bary[1];
		}
		return closest_dist;
	}
	break;
	case 4:
	{
		glm::vec3 diff[] = {
			m_points[0] - m_points[3],
			m_points[1] - m_points[3],
			m_points[2] - m_points[3]
		};
		float v = glm::determinant(glm::mat3(diff[0], diff[1], diff[2]));

		// Check points are not equal
		if (glm::abs(v) < c_epsilon)
			return -1.f;

		// Check points are not coplanar
		glm::vec3 v0_1 = m_points[0] - m_points[1];
		glm::vec3 v1_2 = m_points[1] - m_points[2];
		if (v * glm::dot(m_points[0], glm::cross(v1_2, v0_1)) > 0.0f)
			return -1.f;

		// Create simplexes of the three
		// frontal faces of the tetrahedron
		simplex faces[3]{
			{m_points[0],m_points[1],m_points[3]},
			{m_points[1],m_points[2],m_points[3]},
			{m_points[2],m_points[0],m_points[3]},
		};

		// Compute distance to every face
		float dist[] = { -1.f, -1.f, -1.f };
		for (uint cur = 0; cur < 3; ++cur)
			if (glm::dot(m_points[3], glm::cross(diff[cur], diff[(cur + 1) % 3])) * v > 0.f)
				dist[cur] = faces[cur].project_origin();

		// Get closer face to origin
		uint closest{ 0u };
		float closest_dist{ -1.f };
		for (uint i = 0; i < 3; ++i)
			if (dist[i] != -1.f && ( closest_dist < 0.f || dist[i] < closest_dist))
				closest_dist = dist[i], closest = i;

		// If origin lies closer to a face region
		if (closest_dist >= 0.0f)
		{
			uint nex = (closest + 1) % 3;
			uint oth = (closest + 2) % 3;

			// Fill voronoi mask
			m_voronoi = 0u;
			if (faces[closest].m_voronoi & voronoi::flag[0])
				m_voronoi |= voronoi::flag[closest];
			if (faces[closest].m_voronoi & voronoi::flag[1])
				m_voronoi |= voronoi::flag[nex];
			if (faces[closest].m_voronoi & voronoi::flag[2])
				m_voronoi |= voronoi::flag[3];

			// Assign barycentric
			m_bary[closest] = faces[closest].m_bary[0];
			m_bary[nex] = faces[closest].m_bary[1];
			m_bary[oth] = 0.f;
			m_bary[3] = faces[closest].m_bary[2];
		}

		// If origin lies closer to the tetrahedron region
		else
		{
			// It's inside
			closest_dist = 0.f;

			// Keep the entire tetrahedron
			m_voronoi = voronoi::tetra_region;

			// Assign barycentric
			m_bary[0] = glm::determinant(glm::mat3{ m_points[2],m_points[1],m_points[3] }) / v;
			m_bary[1] = glm::determinant(glm::mat3{ m_points[0],m_points[2],m_points[3] }) / v;
			m_bary[2] = glm::determinant(glm::mat3{ m_points[1],m_points[0],m_points[3] }) / v;
			m_bary[3] = 1 - m_bary[0] - m_bary[1] - m_bary[2];
		}
		return closest_dist;
	}
	break;
	default:
		break;
	}
	return -1.f;
}

const glm::vec3 & simplex::last()const
{
	assert(m_dim > 0u);
	return m_points[m_dim-1];
}

simplex simplex::get_next(float & distance, glm::vec3 & next_dir)
{
	distance = project_origin();
	
	// Store next simplex data
	simplex s;
	for (uint i = 0; i < m_dim; ++i)
	{
		// If flagged
		if (m_voronoi & voronoi::flag[i])
		{
			// copy vertex
			s.m_points[s.m_dim] = m_points[i];
			s.m_dirs[s.m_dim] = m_dirs[i];
			s.m_bary[s.m_dim++] = m_bary[i];

			// contribute to next direction
			next_dir += m_points[i] * m_bary[i];
		}
	}
	return s;
}

const float gjk::c_min_distance = 1e-3f;
const uint gjk::c_max_iterations = 64u;
gjk::gjk(const physical_mesh & A, const physical_mesh & B, const glm::mat4 & modA, const glm::mat4 & modB)
	:m_mesh_A(A), m_mesh_B(B),
	m_mod_A(modA), m_mod_B(modB),
	m_invmod_A(glm::inverse(modA)), m_invmod_B(glm::inverse(modB))
{}
gjk::status gjk::evaluate(glm::vec3 initial_dir)
{
	// Get initial direction
	float len = glm::length(initial_dir);
	if (len < c_epsilon)
		initial_dir = glm::vec3{ 1.f, 0.f, 0.f };
	m_dir = initial_dir;
	
	// Initialize the simplex
	add_vertex(m_simplex, -m_dir);
	m_simplex.m_bary[0] = 1.f;

	// Setup previous point data
	m_prev[0] = m_prev[1] = m_prev[2]
		= m_prev[3] = m_simplex.last();
	uint prev_dx{ 1u };

	// Select next direction
	m_dir = m_simplex.last();
	
	for(;;)
	{
		// Check if origin layed on the projection
		len = glm::length(m_dir);
		if (len < c_min_distance)
			return m_status = e_Success;
		
		// Add new point to it
		add_vertex(m_simplex, -m_dir);
		
		// Check if the point is already in the simplex
		for (uint i = 0; i < 4; ++i)
			if (glm::length2(m_simplex.last() - m_prev[i]) < c_min_distance)
				// Exit is no furthest point has been found
				return m_status = e_Fail_NoFurthestPoint;

		// Update PrevPoints
		m_prev[prev_dx] = m_simplex.last();
		prev_dx = (prev_dx + 1) % 4;
		
		// Compute next simplex
		m_dir = { 0.0f };
		m_simplex = m_simplex.get_next(len, m_dir);

		// Projection failed
		if (len < 0.0f)
			return m_status = e_Fail_ProjectionFail;

		// If origin is inside the tetrahedron
		if(m_simplex.m_dim == 4u)
			return m_status = e_Success;

		// Prevent infinite loops
		if (m_status == e_Running && ++m_iterations == c_max_iterations)
			return m_status = e_Fail_IterationLimit;
	}
}
glm::vec3 gjk::support(glm::vec3 dir)const
{
	glm::vec3 dirA = tr_vector(m_invmod_A, dir);
	glm::vec3 dirB = tr_vector(m_invmod_B, dir);

	glm::vec3 localA = m_mesh_A.support_point_hillclimb(dirA);
	glm::vec3 localB = m_mesh_B.support_point_hillclimb(-dirB);

	glm::vec3 supA = tr_point(m_mod_A, localA);
	glm::vec3 supB = tr_point(m_mod_B, localB);

	return supA - supB;
}
void gjk::add_vertex(simplex & simp, glm::vec3 dir)const
{
	glm::vec3 d{ glm::normalize(dir) };
	simp.m_dirs[simp.m_dim] = d;
	simp.m_points[simp.m_dim] = support(d);
	simp.m_bary[simp.m_dim] = 0;
	++simp.m_dim;
}
void gjk::rem_vertex(simplex & simp) const
{
	--simp.m_dim;
}
bool gjk::complete_simplex()
{
	glm::vec3 axis[]{
		glm::vec3{1,0,0},
		glm::vec3{0,1,0},
		glm::vec3{0,0,1}
	};
	switch (m_simplex.m_dim)
	{
	case 1:
		for (uint i = 0; i < 3; i++)
		{
			add_vertex(m_simplex, axis[i]);
			if (complete_simplex())
				return true;
			rem_vertex(m_simplex);

			add_vertex(m_simplex, -axis[i]);
			if (complete_simplex())
				return true;
			rem_vertex(m_simplex);
		}
		break;

	case 2:
	{
		glm::vec3 diff = m_simplex.m_points[1] - m_simplex.m_points[0];
		for (uint i = 0; i < 3; i++)
		{
			glm::vec3 p = glm::cross(diff, axis[i]);
			if (glm::length2(p) < c_epsilon)
				continue;

			add_vertex(m_simplex, p);
			if (complete_simplex())
				return true;
			rem_vertex(m_simplex);

			add_vertex(m_simplex, -p);
			if (complete_simplex())
				return true;
			rem_vertex(m_simplex);
		}
	}
	break;

	case 3:
	{
		glm::vec3 norm = glm::cross(
			m_simplex.m_points[1] - m_simplex.m_points[0],
			m_simplex.m_points[2] - m_simplex.m_points[0]);
		if (glm::length2(norm) > c_epsilon)
		{
			add_vertex(m_simplex, norm);
			if (complete_simplex())
				return true;
			rem_vertex(m_simplex);

			add_vertex(m_simplex, -norm);
			if (complete_simplex())
				return true;
			rem_vertex(m_simplex);
		}
	}
	break;
	case 4:
		float v = glm::determinant(glm::mat3(
			m_simplex.m_points[0] - m_simplex.m_points[3],
			m_simplex.m_points[1] - m_simplex.m_points[3],
			m_simplex.m_points[2] - m_simplex.m_points[3]
		));
		if (glm::abs(v) > c_epsilon)
			return true;
		break;
	}
	return false;
}
