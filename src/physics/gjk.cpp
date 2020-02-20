#include "gjk.h"

const float gjk::c_min_distance = 1e-3f;
const uint gjk::c_max_iterations = 64u;
const uint gjk::voronoi_flag[4] = { 1, 2, 4, 8 };

gjk::gjk(const physical_mesh & A, const physical_mesh & B, const glm::mat4 & modA, const glm::mat4 & modB)
	:m_mesh_A(A), m_mesh_B(B),
	m_mod_A(modA), m_mod_B(modB),
	m_invmod_A(glm::inverse(modA)), m_invmod_B(glm::inverse(modB))
{}

void gjk::evaluate(glm::vec3 initial_dir)
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
		
		// Barycenetric coord of the origin based on the simplex
		glm::vec4 bary;
		// Mask flagging the valid vertices 
		uint voronoi_mask = 0;
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

bool gjk::close_simplex()
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
			if (close_simplex())
				return true;
			rem_vertex(m_simplex);

			add_vertex(m_simplex, -axis[i]);
			if (close_simplex())
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
			if (close_simplex())
				return true;
			rem_vertex(m_simplex);

			add_vertex(m_simplex, -p);
			if (close_simplex())
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
			if (close_simplex())
				return true;
			rem_vertex(m_simplex);

			add_vertex(m_simplex, -norm);
			if (close_simplex())
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

float gjk::project_origin_2D(glm::vec3 a, glm::vec3 b, glm::vec4& bary, uint& voronoi_mask)const
{
	glm::vec3 diff = b - a;

	// Check points are not equal
	float len2 = glm::length2(diff);
	if(len2 < c_epsilon)
		return -1.f;

	// Compute time projection
	float t = -glm::dot(a, diff) / len2;

	// Fill voronoi mask
	if (t <= 0)
		voronoi_mask = voronoi_flag[0];
	else if( t >=1)
		voronoi_mask = voronoi_flag[1];
	// Keep both vertices
	else
		voronoi_mask = voronoi_flag[0]|voronoi_flag[1];

	//Clamp projection onto line segment
	t = glm::clamp(t, 0.0f, 1.0f);

	// Store barycentric
	bary[0] = 1 - t;
	bary[1] = t;

	// return distance
	return glm::length2(a + diff * t);
}

float gjk::project_origin_3D(glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec4& bary, uint& voronoi_mask)const
{
	glm::vec3 diff[] = { a - b, b - c, c - a };
	glm::vec3 norm = glm::cross(diff[0], diff[1]);

	// Check points are not equal
	float len2 = glm::length2(norm);
	if (len2 < c_epsilon)
		return -1.f;
	
	// Compute distance to every edge
	uint ma[3];
	glm::vec4 ba[3];
	glm::vec3 points[] = { a,b,c };
	float dist[] = { -1.f, -1.f, -1.f };
	for (uint cur = 0; cur < 3; ++cur)
	{
		uint nex = (cur + 1) % 3;
		if (glm::dot(points[cur], glm::cross(diff[cur], norm)) > 0.0f)
			dist[cur] = project_origin_2D(points[cur], points[nex], ba[cur], ma[cur]);
	}

	// Get close edge to origin
	uint closest;
	float closest_dist{ -1.f };
	for (uint i = 0; i < 3; ++i)
		if (dist[i] != -1.f)
			if (closest_dist < 0.f || dist[i] < closest_dist)
				closest_dist = dist[i], closest = i;

	// If origin lies closer to an edge region
	if (closest_dist >= 0.0f)
	{
		uint nex = (closest + 1) % 3;
		uint oth = (closest + 2) % 3;

		// Fill voronoi mask
		voronoi_mask = 0u;
		if (ma[closest] & voronoi_flag[0])
			voronoi_mask |= voronoi_flag[closest];
		if (ma[closest] & voronoi_flag[1])
			voronoi_mask |= voronoi_flag[nex];

		// Assign barycentric
		bary[closest] = ba[closest][0];
		bary[nex] = ba[closest][1];
		bary[oth] = 0.f;
	}

	// If origin lies closer to the face region
	else
	{
		// Project origin onto the face
		float d = glm::dot(a, norm);
		glm::vec3 p = norm * (d / len2);
		closest_dist = glm::length2(p);

		// Keep the entire face
		voronoi_mask = voronoi_flag[0]|voronoi_flag[1]|voronoi_flag[2];

		// Assign barycentric
		float len = glm::sqrt(len2);
		bary[0] = glm::length(glm::cross(diff[1], b - p)) / len;
		bary[1] = glm::length(glm::cross(diff[2], c - p)) / len;
		bary[2] = 1-bary[0]-bary[1];
	}
	return closest_dist;
}

float gjk::project_origin_4D(glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 d, glm::vec4& bary, uint& voronoi_mask)const
{
	glm::vec3 diff[] = { a - d, b - d, c - d };
	float v = glm::determinant(glm::mat3(diff[0], diff[1], diff[2]));


	// Check points are not equal
	if (glm::abs(v) < c_epsilon)
		return -1.f;
	if (v * glm::dot(a, glm::cross(b - c, a - b)) > 0.0f)
		return -1.f;

	// Compute distance to every face
	// ( ignore last possible face )
	uint ma[3];
	glm::vec4 ba[3];
	glm::vec3 points[] = { a,b,c,d };
	float dist[] = { -1.f, -1.f, -1.f };
	for (uint cur = 0; cur < 3; ++cur)
	{
		uint nex = (cur + 1) % 3;
		if (glm::dot(d, glm::cross(diff[cur], diff[nex])) * v > 0.f)
			dist[cur] = project_origin_3D(points[cur], points[nex], d, ba[cur], ma[cur]);
	}

	// Get closer face to origin
	uint closest;
	float closest_dist{ -1.f };
	for (uint i = 0; i < 3; ++i)
		if(dist[i] != -1.f)
			if (closest_dist < 0.f || dist[i] < closest_dist)
				closest_dist = dist[i], closest = i;

	// If origin lies closer to a face region
	if (closest_dist >= 0.0f)
	{
		uint nex = (closest + 1) % 3;
		uint oth = (closest + 2) % 3;

		// Fill voronoi mask
		voronoi_mask = 0u;
		if (ma[closest] & voronoi_flag[0])
			voronoi_mask |= voronoi_flag[closest];
		if (ma[closest] & voronoi_flag[1])
			voronoi_mask |= voronoi_flag[nex];
		if (ma[closest] & voronoi_flag[2])
			voronoi_mask |= voronoi_flag[3];

		// Assign barycentric
		bary[closest] = ba[closest][0];
		bary[nex] = ba[closest][1];
		bary[oth] = 0.f;
		bary[3] = ba[closest][2];
	}

	// If origin lies closer to the tetrahedron region
	else
	{
		// It's inside
		closest_dist = 0.f;

		// Keep the entire face
		voronoi_mask = voronoi_flag[0] | voronoi_flag[1]
			| voronoi_flag[2] | voronoi_flag[3];

		// Assign barycentric
		bary[0] = glm::determinant(glm::mat3{ c,b,d }) / v;
		bary[1] = glm::determinant(glm::mat3{ a,c,d }) / v;
		bary[2] = glm::determinant(glm::mat3{ b,a,d }) / v;
		bary[3] = 1 - bary[0] - bary[1] - bary[2];
	}
	return closest_dist;
}
