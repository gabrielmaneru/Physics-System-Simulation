#include "gjk.h"

const float gjk::c_min_distance = 1e-3f;
const uint gjk::c_max_iterations = 64u;
const uint gjk::voronoi_flag[4] = { 1, 2, 4, 8 };

gjk::gjk(const physical_mesh & A, const physical_mesh & B, const glm::mat4 & modA, const glm::mat4 & modB)
	:m_mesh_A(A), m_mesh_B(B),
	m_mod_A(modA), m_mod_B(modB),
	m_invmod_A(glm::inverse(modA)), m_invmod_B(glm::inverse(modB))
{}

bool gjk::evaluate(glm::vec3 initial_dir)
{
	// Get Initial Direction
	float len = glm::length(initial_dir);
	m_dir = len > c_epsilon ? initial_dir : glm::vec3{1.f, 0.f, 0.f};
	
	// Initialize the simplex
	add_vertex(m_simplex, -m_dir);
	m_simplex.m_bary[0] = 1;
	m_dir = m_simplex.m_points[0];

	// Setup PrevPoint data
	m_prev[0] = m_dir;
	m_prev[1] = m_dir;
	m_prev[2] = m_dir;
	m_prev[3] = m_dir;
	uint prev_dx{ 0u };
	
	do
	{
		// Check point if prev point is the origin
		len = glm::length(m_dir);
		if (len < c_min_distance)
		{
			m_status = e_Success;
			break;
		}

		// Make a copy of the simplex
		simplex s = m_simplex;
		// Add new point to it
		add_vertex(s, -m_dir);
		const glm::vec3& p = s.m_points[s.m_dim - 1];

		// Check if the point is already in the simplex
		bool finish{ false };
		for (uint i = 0; i < 4; ++i)
			if (glm::length2(p - m_prev[i]) < c_min_distance)
			{
				finish = true;
				break;
			}
		// Exit is no furthest point has been found
		if (finish)
			break;
		// Update PrevPoint
		else
		{
			prev_dx = (prev_dx + 1) & 3;
			m_prev[prev_dx] = p;
		}

		//Check ending condition
		float ang = glm::dot(m_dir, p) / len;
		m_angle = glm::max(m_angle, ang);
		if (len * (1.0f - c_epsilon) - m_angle <= 0.0f)
			break;

		// Barycenetri coord of the origin based on the simplex
		glm::vec4 bary;
		// Mask flagging the valid vertices 
		unsigned voronoi_mask = 0;
		// Project origin onto simplex
		switch (s.m_dim)
		{
		case 2:
			len = project_origin_2D(
				s.m_points[0], s.m_points[1],
				bary, voronoi_mask);
			break;
		case 3:
			len = project_origin_3D(
				s.m_points[0], s.m_points[1], s.m_points[2],
				bary, voronoi_mask);
			break;
		case 4:
			len = project_origin_4D(
				s.m_points[0], s.m_points[1], s.m_points[2], s.m_points[3],
				bary, voronoi_mask);
			break;
		}

		// Is projection okay
		if (len >= 0.0f)
		{
			// Clean stored simplex
			m_simplex.m_dim = 0u;

			// Clean next direction
			m_dir = { 0.0f };

			// Store current simplex
			for (uint i = 0; i < s.m_dim; ++i)
			{
				// If flagged
				if (voronoi_mask & voronoi_flag[i])
				{
					// copy vertex
					m_simplex.m_points[m_simplex.m_dim] = s.m_points[i];
					m_simplex.m_dirs[m_simplex.m_dim] = s.m_dirs[i];
					m_simplex.m_bary[m_simplex.m_dim] = bary[i];
					m_simplex.m_dim++;

					// compute next direction
					m_dir += s.m_points[i] * bary[i];
				}
			}
			if (voronoi_mask == (voronoi_flag[0] | voronoi_flag[1] | voronoi_flag[2] | voronoi_flag[3]))
			{
				m_status = e_Success;
				break;
			}
		}
		else
			break;
		if (m_status == e_Running && ++m_iterations == c_max_iterations)
			m_status = e_Failed;
	} while (m_status == e_Running);

	return m_status == e_Success;
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

float gjk::project_origin_2D(glm::vec3 a, glm::vec3 b, glm::vec4& bary, uint& voronoi_mask)const
{
	glm::vec3 diff = b - a;
	float len2 = glm::length2(diff);
	if(len2 < c_epsilon)
		return -1;

	float t = -glm::dot(a, diff) / len2;
	if (t <= 0)
		voronoi_mask = voronoi_flag[0];
	else if( t >=1)
		voronoi_mask = voronoi_flag[1];
	else
		voronoi_mask = voronoi_flag[2];

	t = glm::clamp(t, 0.0f, 1.0f);
	bary[0] = 1 - t;
	bary[1] = t;
	return glm::length2(a + diff * t);
}

float gjk::project_origin_3D(glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec4& bary, uint& voronoi_mask)const
{

	return -1;
}

float gjk::project_origin_4D(glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 d, glm::vec4& bary, uint& voronoi_mask)const
{
	return -1;
}
