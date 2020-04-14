#include "contact_info.h"
#include "math_utils.h"
#include "body.h"

overlap_pair::overlap_pair(body * bA, body * bB, const physical_mesh * mA, const physical_mesh * mB)
	:m_body_A(bA),m_body_B(bB), m_mesh_A(mA), m_mesh_B(mB)
{
	m_manifold.oldvec_U = glm::zero<glm::vec3>();
	m_manifold.oldvec_V = glm::zero<glm::vec3>();
}

void overlap_pair::update()
{
	m_manifold.coef_friction = std::sqrtf(m_body_A->m_friction_coef * m_body_B->m_friction_coef);
	m_manifold.coef_roll = std::sqrtf(m_body_A->m_roll_coef * m_body_B->m_roll_coef);
	m_manifold.coef_restitution = std::fmax(m_body_A->m_restitution_coef, m_body_B->m_restitution_coef);

	m_manifold.invM_A = m_body_A->get_invmass();
	m_manifold.invM_B = m_body_B->get_invmass();
	m_manifold.invI_A = m_body_A->get_oriented_invinertia();
	m_manifold.invI_B = m_body_B->get_oriented_invinertia();
	// Get data
	const glm::vec3 normal = m_manifold.normal;
	const glm::vec3 Pos_A = m_body_A->m_position;
	const glm::vec3 Pos_B = m_body_B->m_position;
	// Get transformation matrices
	const glm::mat4 AtoW = m_body_A->get_model();
	const glm::mat4 BtoW = m_body_B->get_model();
	// Accumulate friction point
	m_manifold.avg_point_A =glm::zero<glm::vec3>();
	m_manifold.avg_point_B =glm::zero<glm::vec3>();


	// Update contact points
	for (auto& p : m_manifold.points)
	{
		// Compute world position
		p.point_A = tr_point(AtoW, p.local_A);
		p.point_B = tr_point(BtoW, p.local_B);
		// Compute R vectors
		const glm::vec3 R_A = p.point_A - Pos_A;
		const glm::vec3 R_B = p.point_B - Pos_B;
		// Accumulate average points
		m_manifold.avg_point_A += p.point_A;
		m_manifold.avg_point_B += p.point_B;


		// Compute inverse mass of velocity constraint
		const glm::vec3 rAxN = glm::cross(R_A, normal);
		const glm::vec3 rBxN = glm::cross(R_B, normal);
		const float eff_mass
			= m_manifold.invM_A
			+ m_manifold.invM_B
			+ glm::dot(glm::cross(m_manifold.invI_A * rAxN, R_A), normal)
			+ glm::dot(glm::cross(m_manifold.invI_B * rBxN, R_B), normal);
		p.invM_Vel = eff_mass > 0.0f ? 1.f / (eff_mass * (float)m_manifold.points.size()) : 0.0f;


		// Compute velocities at contact points
		const glm::vec3 vpA = m_body_A->get_velocity_at_point(p.point_A);
		const glm::vec3 vpB = m_body_B->get_velocity_at_point(p.point_B);
		// Compute initial Jv_vel at contact point
		const float Jv0_vel = glm::dot(vpB- vpA, normal);


		// Compute restitution bias
		p.restitution_bias = 0.0f;
		if (Jv0_vel < -c_rest_vel_threshold)
			p.restitution_bias = Jv0_vel * m_manifold.coef_restitution;
	}


	// Compute average points
	float inv_c = 1.0f / static_cast<float>(m_manifold.points.size());
	m_manifold.avg_point_A *= inv_c;
	m_manifold.avg_point_B *= inv_c;
	// Compute R vectors
	const glm::vec3 R_A = m_manifold.avg_point_A - Pos_A;
	const glm::vec3 R_B = m_manifold.avg_point_B - Pos_B;


	// Prepare data for warm start
	m_manifold.oldvec_U = m_manifold.vec_U;
	m_manifold.oldvec_V = m_manifold.vec_V;


	// Compute friction vectors
	const glm::vec3 vpA = m_body_A->get_velocity_at_point(m_manifold.avg_point_A);
	const glm::vec3 vpB = m_body_B->get_velocity_at_point(m_manifold.avg_point_B);
	// Compute difference vectors
	const glm::vec3 delta_vp = vpB - vpA;
	// Compute tangent velocity
	const glm::vec3 normal_velocity = delta_vp * normal * normal;
	const glm::vec3 tangent_velocity = delta_vp - normal_velocity;
	// If tangent is valid use it as first constraint vector for consistency
	if (glm::length2(tangent_velocity) > c_epsilon)
		m_manifold.vec_U = glm::normalize(tangent_velocity);
	// Otherwise get a vector orthogonal to the normal
	else 
		m_manifold.vec_U = make_ortho(normal);
	// Compute second constraint vector
	m_manifold.vec_V = glm::normalize(glm::cross(normal, m_manifold.vec_U));


	// Compute inverse mass of first friction constraint
	const glm::vec3 rAxU = glm::cross(R_A, m_manifold.vec_U);
	const glm::vec3 rBxU = glm::cross(R_B, m_manifold.vec_U);
	const float M_u
		= m_manifold.invM_A
		+ m_manifold.invM_B
		+ glm::dot(glm::cross(m_manifold.invI_A*rAxU, R_A), m_manifold.vec_U)
		+ glm::dot(glm::cross(m_manifold.invI_B*rBxU, R_B), m_manifold.vec_U);
	m_manifold.invM_U = M_u > 0.0f ? 1.f / M_u : 0.0f;


	// Compute inverse mass of second friction constraint
	const glm::vec3 rAxV = glm::cross(R_A, m_manifold.vec_V);
	const glm::vec3 rBxV = glm::cross(R_B, m_manifold.vec_V);
	const float M_V
		= m_manifold.invM_A
		+ m_manifold.invM_B
		+ glm::dot(glm::cross(m_manifold.invI_A*rAxV, R_A), m_manifold.vec_V)
		+ glm::dot(glm::cross(m_manifold.invI_B*rBxV, R_B), m_manifold.vec_V);
	m_manifold.invM_V = M_V > 0.0f ? 1.f / M_V : 0.0f;


	// Compute inverse mass of twist friction constraint
	const float M_Twist
		= glm::dot(normal, m_manifold.invI_A*normal)
		+ glm::dot(normal, m_manifold.invI_B*normal);
	m_manifold.invM_Twist = M_Twist > 0.0f ? 1.f / M_Twist : 0.0f;


	// Compute inverse mass of roll resistance constraint
	m_manifold.invM_Roll = glm::mat3(0.0f);
	if (m_manifold.coef_roll > 0.0f)
	{
		m_manifold.invM_Roll = m_manifold.invI_A + m_manifold.invI_B;

		if (glm::determinant(m_manifold.invM_Roll) != 0.0f)
			m_manifold.invM_Roll = glm::inverse(m_manifold.invM_Roll);
		else
			m_manifold.invM_Roll = glm::mat3(0.0f);
	}
}

void overlap_pair::add_manifold(const simple_manifold & other)
{
	// Check if both manifold refer to the same normal
	// If have the same normal -> update points
	if (glm::dot(m_manifold.normal, other.normal) > 1.0f - c_epsilon)
	{
		std::vector<contact_point> points = other.points;
		for (auto& p1 : points)
			for (auto p2 : m_manifold.points)
			{
				if (glm::length2(p1.local_A - p2.local_A) < c_epsilon
					|| glm::length2(p1.local_B - p2.local_B) < c_epsilon)
				{
					p1.lambda_Vel = p2.lambda_Vel;
					break;
				}
			}
		m_manifold.points = points;
		m_manifold.normal = glm::normalize(other.normal);
		m_manifold.lambda_U = 0.0f;
		m_manifold.lambda_V = 0.0f;
		m_manifold.lambda_Twist = 0.0f;
		m_manifold.lambda_Roll = glm::vec3{ 0.0f, 0.0f, 0.0f };
	}
	// If the normal is different -> overwrite manifold
	else
	{
		m_manifold.normal = glm::normalize(other.normal);
		m_manifold.points = other.points;
		
		m_manifold.lambda_U = 0.0f;
		m_manifold.lambda_V = 0.0f;
		m_manifold.lambda_Twist = 0.0f ;
		m_manifold.lambda_Roll = glm::vec3{ 0.0f, 0.0f, 0.0f };
	}
}
